"""Odometry control and handoff with in-memory ROS substitutes; no robot commands."""
import json
import math
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import cv2
import numpy as np
import pytest
import yaml
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool

from parking.detection import ParkingDetection


class Recorder:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


@pytest.fixture
def node_factory(monkeypatch):
    parameters = {}
    subscriptions = {}
    clock = [10.0]
    monkeypatch.setattr(Node, '__init__', lambda *a, **kw: None)
    monkeypatch.setattr(Node, 'declare_parameter', lambda self, name, value, *a: parameters.setdefault(name, value))
    monkeypatch.setattr(Node, 'get_parameter', lambda self, name: SimpleNamespace(value=parameters[name]))
    monkeypatch.setattr(Node, 'create_publisher', lambda *a, **kw: Recorder())
    monkeypatch.setattr(Node, 'create_subscription', lambda self, kind, topic, callback, qos:
                        subscriptions.setdefault(topic, callback))
    monkeypatch.setattr(Node, 'create_timer', lambda *a, **kw: None)
    monkeypatch.setattr(Node, 'get_logger', lambda self: Mock())
    monkeypatch.setattr(Node, 'get_clock', lambda self: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=int(clock[0]*1e9))))

    def create(overrides=None):
        parameters.clear()
        subscriptions.clear()
        parameters.update(overrides or {})
        n = ParkingDetection()
        n.clock, n.parameters, n.recorded_subscriptions = clock, parameters, subscriptions
        return n
    return create


@pytest.fixture
def node(node_factory):
    return node_factory()


def image(white_line=True):
    pixels = np.zeros((480, 640, 3), np.uint8)
    if white_line:
        pixels[432:, :320] = 255  # 10% of the current left-half BEV stop-line ROI
    else:
        pixels[:, 320:] = 255  # white outside the current ROI
    return CompressedImage(data=cv2.imencode('.jpg', pixels)[1].tobytes())


def odom(n, x=None, y=None, yaw=None, v=0.0, w=0.0, dt=.05, stamp=None, frame='odom'):
    n.clock[0] += dt
    msg = Odometry()
    ns = int(n.clock[0]*1e9) if stamp is None else stamp
    msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(ns, 1000000000)
    msg.header.frame_id, msg.child_frame_id = frame, 'base_footprint'
    px, py = n.position or (0.0, 0.0)
    msg.pose.pose.position.x = float(px if x is None else x)
    msg.pose.pose.position.y = float(py if y is None else y)
    angle = (n.yaw or 0.0) if yaw is None else yaw
    msg.pose.pose.orientation.z = math.sin(angle/2)
    msg.pose.pose.orientation.w = math.cos(angle/2)
    msg.twist.twist.linear.x, msg.twist.twist.angular.z = float(v), float(w)
    n.odom_callback(msg)
    return msg


def approach(n):
    n.sign_callback(Bool(data=True))
    n.lane_state_callback(Bool(data=True))
    assert n.phase == 'APPROACH'


def start_parking(n):
    approach(n)
    odom(n)
    n.camera_callback(image())
    assert n.phase == 'PARKING' and n.settling
    odom(n)
    n.tick()  # fresh stopped odometry establishes the first reference
    assert n.step_origin is not None and not n.settling
    n.tick()


def complete_step(n):
    step = n.step
    x0, y0, yaw0 = n.step_origin
    if step in (0, 2, 3, 5):
        distance = math.copysign(n.motion_param('distances')[step], n.motion_param('linear_speeds')[step])
        odom(n, x0 + distance*math.cos(yaw0), y0 + distance*math.sin(yaw0), v=.1)
    else:
        sign = 1 if n.param('direction') == 'left' else -1
        angle = sign * math.radians(n.motion_param('angles_deg')[step])
        for delta in np.linspace(0, angle, max(2, int(abs(angle)/.5)+1))[1:]:
            odom(n, yaw=yaw0+delta, w=sign*.2)
    n.tick()
    assert n.settling and n.step == step
    assert n.velocity_pub.messages[-1] == Twist()
    n.tick()  # the same pre-stop sample must not finish the step
    assert n.step == step
    odom(n)
    n.tick()
    assert n.step == step + 1


def finish_movements(n):
    if n.step_origin is None:
        odom(n)
        n.tick()
    for _ in range(6):
        complete_step(n)


def test_sign_waits_for_permission_and_no_idle_or_approach_velocity(node):
    node.sign_callback(Bool(data=True))
    node.tick()
    node.camera_callback(image())
    assert node.phase == 'IDLE' and not node.velocity_pub.messages
    node.sign_callback(Bool(data=False))
    node.lane_state_callback(Bool(data=True))
    node.tick()
    assert node.phase == 'APPROACH' and node.mode_pub.messages[-1].data == 'left'
    assert not node.velocity_pub.messages


@pytest.mark.parametrize('direction', ['left', 'right'])
def test_six_odom_movements_automatically_handoff_left(node_factory, direction):
    n = node_factory({'direction': direction})
    approach(n)
    n.camera_callback(image(False))
    assert n.phase == 'APPROACH'
    odom(n, x=2.3, y=-1.4, yaw=math.radians(170))
    n.camera_callback(image())
    odom(n)
    n.tick()
    for i in range(6):
        n.tick()
        cmd = n.velocity_pub.messages[-1]
        assert n.step == i
        if i in (0, 2, 3, 5):
            assert cmd.linear.x == pytest.approx(n.motion_param('linear_speeds')[i])
            assert cmd.angular.z == pytest.approx(0)
        else:
            sign = 1 if direction == 'left' else -1
            assert cmd.linear.x == 0 and sign*cmd.angular.z > 0
            assert abs(cmd.angular.z) <= n.motion_param('angular_speeds')[i]
        complete_step(n)
    assert n.phase == 'DONE' and n.mode_pub.messages[-1].data == 'intersection'
    count = len(n.velocity_pub.messages)
    n.tick()
    n.sign_callback(Bool(data=True))
    n.state_callback(Bool(data=False))
    assert len(n.velocity_pub.messages) == count and n.phase == 'DONE'
    assert [m.data for m in n.mode_pub.messages] == ['', 'left', 'maneuver', 'intersection']
    assert n.phase == 'DONE' and n.mode_pub.messages[-1].data == 'intersection'
    assert json.loads(n.status_pub.messages[-1].data)['step'] == 6


def test_parking_no_longer_subscribes_to_exit_sign(node):
    assert '/left' not in node.recorded_subscriptions
    assert '/parking' in node.recorded_subscriptions
    start_parking(node)
    finish_movements(node)
    assert node.phase == 'DONE'
    assert node.mode_pub.messages[-1].data == 'intersection'


@pytest.mark.parametrize('step', range(6))
def test_external_stop_requires_explicit_resume_at_every_step(node, step):
    start_parking(node)
    for _ in range(step):
        complete_step(node)
    node.tick()
    node.lane_state_callback(Bool(data=False))
    assert node.phase == 'HALT' and node.mode_pub.messages[-1].data == 'halt'
    assert node.velocity_pub.messages[-1] == Twist()
    count = len(node.velocity_pub.messages)
    node.lane_state_callback(Bool(data=True))
    node.sign_callback(Bool(data=True))
    node.clock[0] += 5
    node.tick()
    assert node.phase == 'HALT'
    assert all(cmd == Twist() for cmd in node.velocity_pub.messages[count:])
    origin = node.step_origin
    for _ in range(3):
        odom(node)
        node.tick()
    assert node.phase == 'HALT'  # lane True and a P sign cannot release an external stop.
    node.state_callback(Bool(data=True))
    node.tick()
    assert node.phase == 'PARKING' and node.step == step and node.step_origin == origin
    assert node.mode_pub.messages[-1].data == 'maneuver'


def test_time_alone_does_not_complete_step_and_step_timeout_halts(node):
    start_parking(node)
    for _ in range(8):
        odom(node, dt=.25)
        node.tick()
    assert node.phase == 'PARKING' and node.step == 0  # original timer was 1 second
    odom(node, dt=node.param('step_timeouts')[0])
    node.tick()
    assert node.phase == 'HALT' and node.step == 0
    assert 'timed out' in node.status_pub.messages[-1].data
    assert node.velocity_pub.messages[-1] == Twist()


def test_reverse_progress_ignores_lateral_and_wrong_direction_motion(node):
    start_parking(node)
    for x, y in [(0., .4), (.3, .4)]:
        odom(node, x=x, y=y)
        node.tick()
        assert node.step == 0 and not node.settling
        assert node.velocity_pub.messages[-1].linear.x < 0
    odom(node, x=-.21, y=.4)  # overshoot also requests a stop
    node.tick()
    assert node.settling and node.step == 0


@pytest.mark.parametrize('step', [0, 2, 3, 5])
def test_straight_deceleration_and_heading_correction(node, step):
    start_parking(node)
    for _ in range(step):
        complete_step(node)
    x0, y0, yaw0 = node.step_origin
    sign = math.copysign(1, node.motion_param('linear_speeds')[step])
    distance = sign * (node.motion_param('distances')[step] - .02)
    odom(node, x0+distance*math.cos(yaw0), y0+distance*math.sin(yaw0), yaw0+.1)
    node.tick()
    cmd = node.velocity_pub.messages[-1]
    assert sign*cmd.linear.x == pytest.approx(node.param('min_linear_speed'))
    assert cmd.angular.z == pytest.approx(-.2)  # correct yaw for forward and reverse


@pytest.mark.parametrize('direction,initial', [('left', 170.), ('right', -170.)])
def test_turn_crosses_pi_decelerates_and_requires_stop(node_factory, direction, initial):
    n = node_factory({'direction': direction})
    odom(n, yaw=math.radians(initial))
    start_parking(n)
    complete_step(n)
    sign = 1 if direction == 'left' else -1
    yaw0 = n.step_origin[2]
    for delta in [30., 60., n.motion_param('angles_deg')[1]-3.]:
        odom(n, yaw=yaw0+sign*math.radians(delta), w=sign*.2)
        n.tick()
    assert not n.settling and n.step == 1
    assert n.velocity_pub.messages[-1].angular.z == pytest.approx(sign*n.param('min_angular_speed'))
    odom(n, yaw=yaw0+sign*math.radians(n.motion_param('angles_deg')[1]+2.), w=sign*.2)
    n.tick()
    assert n.settling and n.step == 1
    odom(n, w=sign*.1)
    n.tick()
    assert n.step == 1
    odom(n)
    n.tick()
    assert n.step == 2


def test_initial_stop_requires_a_new_stopped_sample(node):
    approach(node)
    odom(node, v=.1)
    node.camera_callback(image())
    node.tick()
    assert node.settling and node.step_origin is None
    odom(node, v=.1)
    node.tick()
    assert node.step_origin is None and node.velocity_pub.messages[-1] == Twist()
    odom(node)
    node.tick()
    assert node.step_origin is not None


def test_queued_sample_from_before_stop_command_cannot_confirm_stop(node):
    approach(node)
    odom(node, v=.1)
    old_stamp = node.odom_stamp
    node.clock[0] += .1
    node.camera_callback(image())
    odom(node, stamp=old_stamp+50000000)  # increasing, fresh enough, but predates stop
    node.tick()
    assert node.settling and node.step_origin is None
    odom(node)
    node.tick()
    assert node.step_origin is not None


@pytest.mark.parametrize('case', ['missing', 'stale', 'old_header', 'future_header', 'duplicate', 'invalid', 'frame', 'clock'])
def test_bad_odometry_halts_without_advancing(node, case):
    if case == 'missing':
        approach(node)
        node.camera_callback(image())
    else:
        start_parking(node)
        if case == 'stale':
            node.clock[0] += 1.
        elif case == 'old_header':
            # Still increasing stamps, but the queued message is already too old.
            old_stamp = node.odom_stamp + 1
            odom(node, dt=1., stamp=old_stamp)
        elif case == 'future_header':
            odom(node, stamp=node.odom_stamp+2000000000)
        elif case == 'duplicate':
            odom(node, dt=1., stamp=node.odom_stamp)
        elif case == 'invalid':
            odom(node, x=float('nan'))
        elif case == 'frame':
            odom(node, frame='map')
        elif case == 'clock':
            odom(node, stamp=node.odom_stamp-1)
    node.tick()
    assert node.phase == 'HALT' and node.step == 0
    assert node.velocity_pub.messages[-1] == Twist()


def test_stop_confirmation_timeout_halts(node):
    approach(node)
    odom(node, v=.1)
    node.camera_callback(image())
    for _ in range(13):
        odom(node, v=.1, dt=.25)
        node.tick()
    assert node.phase == 'HALT' and node.step == 0
    assert 'stop confirmation' in node.status_pub.messages[-1].data


def test_manual_cancel_invalid_image_and_ros_clock_reset(node):
    approach(node)
    node.state_callback(Bool(data=False))
    assert node.phase == 'HALT'
    node.phase = 'APPROACH'
    node.camera_callback(CompressedImage(data=b'invalid'))
    assert node.phase == 'HALT'
    node.phase = 'PARKING'
    node.clock[0] -= 1
    node.tick()
    assert node.phase == 'HALT' and node.velocity_pub.messages[-1] == Twist()


def test_yaml_values_and_tuned_distance_angle_speed_are_applied(node_factory):
    path = Path(__file__).resolve().parents[1]/'config/parking_sim.yaml'
    values = yaml.safe_load(path.read_text())['parking_detect_node']['ros__parameters']
    n = node_factory(values)  # The user's YAML values are tunable, not fixed defaults.
    assert set(values) == set(n.parameters) and 'durations' not in values
    for name, value in values.items():
        assert value == n.param(name)
    values['left.distances'][0], values['left.angles_deg'][1] = .40, 100.
    values['left.linear_speeds'][0] = -.1
    values['direction'] = 'left'
    n = node_factory(values)
    approach(n)
    odom(n)
    n.camera_callback(image())
    assert n.phase == 'PARKING'
    odom(n)
    n.tick()
    odom(n, x=-.2)
    n.tick()
    assert not n.settling and n.velocity_pub.messages[-1].linear.x == -.1
    complete_step(n)
    yaw0 = n.step_origin[2]
    odom(n, yaw=yaw0+math.radians(90))
    n.tick()
    assert not n.settling and n.step == 1
    odom(n, yaw=yaw0+math.radians(100))
    n.tick()
    assert n.settling


@pytest.mark.parametrize('overrides', [
    {'distances': [0.2]}, {'angles_deg': [0., float('nan'), 0., 0., 77.3, 0.]},
    {'distances': [-.2, 0., .32, .32, 0., .2]},
    {'linear_speeds': [0., 0., .2, -.2, 0., .2]},
    {'linear_speeds': [-.01, 0., .2, -.2, 0., .2]},
    {'angular_speeds': [0., -1.2, 0., 0., 1.5, 0.]},
    {'step_timeouts': [0.]*6}, {'odom_timeout': 0.},
    {'angle_kp': float('inf')}, {'direction': 'invalid'},
    {'distance_tolerance': .5}, {'heading_kp': -1.},
])
@pytest.mark.parametrize('side', ['left', 'right'])
def test_invalid_tuning_rejected_at_startup(node_factory, overrides, side):
    motion_keys = {'distances', 'angles_deg', 'linear_speeds', 'angular_speeds'}
    settings = {(side + '.' + key if key in motion_keys else key): value
                for key, value in overrides.items()}
    # Both sides are validated, including the inactive side.
    settings.setdefault('direction', 'right' if side == 'left' else 'left')
    with pytest.raises(ValueError):
        node_factory(settings)


def test_handoff_publishes_hold_before_zero_and_status(node):
    events = []
    node.mode_pub.publish = lambda msg: events.append(('mode', msg.data))
    node.velocity_pub.publish = lambda msg: events.append(('velocity', msg))
    node.status_pub.publish = lambda msg: events.append(('status', json.loads(msg.data)))
    approach(node)
    events.clear()
    node.camera_callback(image())
    assert [event[0] for event in events] == ['mode', 'velocity', 'status']
    assert events[0][1] == 'maneuver' and events[1][1] == Twist()
    events.clear()
    node.lane_state_callback(Bool(data=False))
    assert [event[0] for event in events] == ['mode', 'velocity', 'status']
    assert events[0][1] == 'halt' and events[1][1] == Twist()


@pytest.mark.parametrize('step', [0, 2, 3, 5])
@pytest.mark.parametrize('sign', [-1., 1.])
@pytest.mark.parametrize('direction', ['left', 'right'])
def test_each_straight_step_direction_uses_speed_sign(node_factory, step, sign, direction):
    speeds = [-.2, 0., .2, -.2, 0., .2]
    speeds[step] = sign * .1
    n = node_factory({'direction': direction, direction + '.linear_speeds': speeds})
    start_parking(n)
    for _ in range(step):
        complete_step(n)
    n.tick()
    assert n.velocity_pub.messages[-1].linear.x == pytest.approx(sign * .1)
    x0, y0, yaw0 = n.step_origin
    distance = n.motion_param('distances')[step]
    # Moving in the opposite direction must not complete this step.
    odom(n, x0-sign*distance*math.cos(yaw0), y0-sign*distance*math.sin(yaw0))
    n.tick()
    assert n.step == step and not n.settling
    complete_step(n)


@pytest.mark.parametrize('direction', ['left', 'right'])
@pytest.mark.parametrize('first_speed', [-.2, .2])
def test_closed_loop_finishes_with_acceleration_and_braking(node_factory, direction, first_speed):
    n = node_factory({'direction': direction,
                      direction + '.linear_speeds': [first_speed, 0., .2, -.2, 0., .2]})
    start_parking(n)
    x = y = yaw = v = w = 0.0
    seen_steps = {0}
    for _ in range(1400):
        cmd = n.velocity_pub.messages[-1]
        v += float(np.clip(cmd.linear.x-v, -.03, .03))
        w += float(np.clip(cmd.angular.z-w, -.10, .10))
        x, y = x+v*math.cos(yaw)*.05, y+v*math.sin(yaw)*.05
        yaw += w*.05
        odom(n, x, y, yaw, v, w)
        n.tick()
        seen_steps.add(n.step)
        if n.phase != 'PARKING':
            break
    assert n.phase == 'DONE', [m.data for m in n.status_pub.messages]
    assert seen_steps == set(range(7))
    assert abs(v) <= n.param('stop_linear_tolerance')
    assert abs(w) <= n.param('stop_angular_tolerance')


@pytest.mark.parametrize('direction', ['left', 'right'])
def test_each_parking_direction_uses_its_own_six_distances_angles_and_speeds(node_factory, direction):
    settings = {
        'direction': direction,
        'left.distances': [.40, 0., .20, .25, 0., .30],
        'left.angles_deg': [0., 60., 0., 0., 75., 0.],
        'left.linear_speeds': [-.12, 0., .14, -.10, 0., .16],
        'left.angular_speeds': [0., .65, 0., 0., .85, 0.],
        'right.distances': [.55, 0., .34, .28, 0., .70],
        'right.angles_deg': [0., 95., 0., 0., 100., 0.],
        'right.linear_speeds': [.18, 0., .16, -.14, 0., .12],
        'right.angular_speeds': [0., 1.10, 0., 0., 1.30, 0.],
    }
    n = node_factory(settings)
    start_parking(n)
    sign = 1 if direction == 'left' else -1
    for step in range(6):
        n.tick()
        command = n.velocity_pub.messages[-1]
        x0, y0, yaw0 = n.step_origin
        if step in (0, 2, 3, 5):
            speed = settings[direction + '.linear_speeds'][step]
            assert command.linear.x == pytest.approx(speed)
            distance = math.copysign(settings[direction + '.distances'][step] / 2, speed)
            odom(n, x0+distance*math.cos(yaw0), y0+distance*math.sin(yaw0))
        else:
            assert command.angular.z == pytest.approx(sign*settings[direction + '.angular_speeds'][step])
            angle = sign*math.radians(settings[direction + '.angles_deg'][step]) / 2
            odom(n, yaw=yaw0+angle)
        n.tick()
        assert n.phase == 'PARKING' and n.step == step and not n.settling
        complete_step(n)
    assert n.phase == 'DONE'
    assert json.loads(n.status_pub.messages[-1].data)['direction'] == direction


def test_default_motion_arrays_are_not_shared_between_directions(node):
    original = list(node.motion_param('distances', 'right'))
    node.parameters['left.distances'][0] = .80
    assert node.motion_param('distances', 'right') == original
    assert node.motion_param('distances', 'left')[0] == .80


def reach_last_target(n):
    start_parking(n)
    for _ in range(5):
        complete_step(n)
    x0, y0, yaw0 = n.step_origin
    distance = math.copysign(n.motion_param('distances')[5], n.motion_param('linear_speeds')[5])
    odom(n, x0+distance*math.cos(yaw0), y0+distance*math.sin(yaw0), v=.1)
    n.tick()
    assert n.step == 5 and n.settling and n.phase == 'PARKING'


@pytest.mark.parametrize('direction', ['left', 'right'])
def test_automatic_left_handoff_requires_fresh_final_stop_and_runs_once(node_factory, direction):
    n = node_factory({'direction': direction})
    reach_last_target(n)
    assert n.mode_pub.messages[-1].data == 'maneuver'
    n.tick()  # cached moving odometry cannot finish
    odom(n, v=.02)
    n.tick()
    odom(n, w=.1)
    n.tick()
    assert n.phase == 'PARKING' and n.step == 5
    events = []
    n.velocity_pub.publish = lambda msg: events.append(('velocity', msg))
    n.mode_pub.publish = lambda msg: events.append(('mode', msg.data))
    n.status_pub.publish = lambda msg: events.append(('status', json.loads(msg.data)))
    odom(n)
    n.tick()
    assert n.phase == 'DONE' and n.step == 6
    assert [event[0] for event in events] == ['velocity', 'mode', 'status']
    assert events[0][1] == Twist() and events[1][1] == 'intersection'
    assert events[2][1]['phase'] == 'DONE' and 'fixed left' in events[2][1]['reason']
    count = len(events)
    n.tick()
    n.sign_callback(Bool(data=True))
    n.state_callback(Bool(data=False))
    assert len(events) == count


@pytest.mark.parametrize('action', ['external_stop', 'manual_cancel', 'odom_dropout'])
def test_stop_during_final_settling_never_releases_to_left_follow(node, action):
    reach_last_target(node)
    if action == 'external_stop':
        node.lane_state_callback(Bool(data=False))
    elif action == 'manual_cancel':
        node.state_callback(Bool(data=False))
    else:
        node.clock[0] += node.param('odom_timeout') + .1
        node.tick()
    odom(node)
    node.tick()
    assert node.phase == 'HALT'
    assert node.mode_pub.messages[-1].data == 'halt'
    assert all(m.data != 'intersection' for m in node.mode_pub.messages)


@pytest.mark.parametrize('direction', ['left', 'right'])
@pytest.mark.parametrize('step', range(6))
def test_odom_recovery_preserves_remaining_goal_and_excludes_pause(node_factory, direction, step):
    n = node_factory({'direction': direction})
    start_parking(n)
    for _ in range(step):
        complete_step(n)
    origin, started = n.step_origin, n.step_started
    x0, y0, yaw0 = origin
    straight = step in (0, 2, 3, 5)
    speed = n.motion_param('linear_speeds')[step]
    sign = math.copysign(1., speed) if straight else (1. if direction == 'left' else -1.)
    goal = (n.motion_param('distances')[step] if straight
            else math.radians(n.motion_param('angles_deg')[step]))

    def progress(fraction):
        if straight:
            odom(n, x0+sign*goal*fraction*math.cos(yaw0),
                 y0+sign*goal*fraction*math.sin(yaw0))
        else:
            odom(n, yaw=yaw0+sign*goal*fraction)

    progress(.5)
    n.tick()
    n.clock[0] += n.param('odom_timeout') + .1
    n.tick()
    assert n.phase == 'HALT'
    paused_at = n.clock[0]
    n.clock[0] += 60.  # The pause is longer than every step timeout.
    for sample in range(3):
        progress(.6)  # Coasting during the stop also counts toward the original goal.
        n.tick()
        if sample < 2:
            assert n.phase == 'HALT' and n.velocity_pub.messages[-1] == Twist()
    assert n.phase == 'PARKING' and n.step == step and n.step_origin == origin
    assert n.step_started == pytest.approx(started+n.clock[0]-paused_at)
    assert not n.settling and n.mode_pub.messages[-1].data == 'maneuver'
    n.tick()
    if straight:
        expected = sign*min(abs(speed), max(n.param('min_linear_speed'),
                                           n.param('distance_kp')*goal*.4))
        assert n.velocity_pub.messages[-1].linear.x == pytest.approx(expected)
    else:
        expected = sign*min(n.motion_param('angular_speeds')[step],
                            max(n.param('min_angular_speed'), n.param('angle_kp')*goal*.4))
        assert n.velocity_pub.messages[-1].angular.z == pytest.approx(expected)
    complete_step(n)  # The original target, rather than a new full movement, finishes this step.
    assert n.phase == ('DONE' if step == 5 else 'PARKING')


def test_recovery_requires_consecutive_fresh_stopped_odometry(node):
    start_parking(node)
    node.clock[0] += 1.
    node.tick()
    for _ in range(2):
        odom(node)
        node.tick()
    for _ in range(4):
        odom(node, stamp=node.odom_stamp)  # Duplicates do not count as new confirmations.
        node.tick()
    assert node.phase == 'HALT'
    odom(node, v=.1)
    node.tick()
    for _ in range(2):
        odom(node)
        node.tick()
    assert node.phase == 'HALT'
    odom(node, dt=1.)  # A gap restarts the confirmation count, even with a fresh new stamp.
    node.tick()
    assert node.phase == 'HALT' and node.recovery_samples == 1
    odom(node, x=float('nan'))
    assert node.recovery_samples == 0
    for _ in range(3):
        odom(node)
        node.tick()
    assert node.phase == 'PARKING'


def test_camera_error_recovers_approach_only_after_good_frames_and_stopped_odom(node):
    approach(node)
    odom(node)
    node.camera_callback(CompressedImage(data=b'invalid'))
    for _ in range(2):
        odom(node)
        node.camera_callback(image())
        node.tick()
    assert node.phase == 'HALT'
    node.camera_callback(CompressedImage(data=b'invalid'))
    for _ in range(2):
        odom(node)
        node.camera_callback(image())
        node.tick()
    assert node.phase == 'HALT'
    odom(node)
    node.camera_callback(image())
    node.tick()
    assert node.phase == 'APPROACH' and node.mode_pub.messages[-1].data == 'left'
    node.camera_callback(image())
    assert node.phase == 'PARKING'


def test_manual_cancel_in_approach_requires_explicit_request_and_restores_left(node):
    approach(node)
    node.state_callback(Bool(data=False))
    for _ in range(3):
        odom(node)
        node.camera_callback(image(False))
        node.tick()
    assert node.phase == 'HALT'
    node.state_callback(Bool(data=True))
    node.tick()
    assert node.phase == 'APPROACH' and node.mode_pub.messages[-1].data == 'left'


@pytest.mark.parametrize('reason', ['external', 'manual'])
def test_explicit_stop_overrides_pending_automatic_recovery(node, reason):
    start_parking(node)
    node.clock[0] += 1.
    node.tick()
    if reason == 'external':
        node.lane_state_callback(Bool(data=False))
    else:
        node.state_callback(Bool(data=False))
    for _ in range(3):
        odom(node)
        node.tick()
    node.lane_state_callback(Bool(data=True))
    node.tick()
    assert node.phase == 'HALT'
    node.state_callback(Bool(data=True))
    node.tick()
    assert node.phase == 'PARKING'


@pytest.mark.parametrize('reset', ['frame', 'stamp', 'node_clock'])
def test_reset_during_halt_blocks_automatic_and_explicit_resume(node, reset):
    start_parking(node)
    origin = node.step_origin
    node.clock[0] += 1.
    node.tick()
    if reset == 'frame':
        odom(node, frame='map')
    elif reset == 'stamp':
        odom(node, stamp=node.odom_stamp-1)
    else:
        node.clock[0] -= .1
        node.tick()
    node.state_callback(Bool(data=True))
    for _ in range(3):
        odom(node)
        node.tick()
    assert node.phase == 'HALT' and node.step_origin == origin
    assert node.velocity_pub.messages[-1] == Twist()


@pytest.mark.parametrize('override', [None, 'external', 'manual'])
def test_timeout_retry_keeps_goal_and_grants_new_step_budget(node, override):
    start_parking(node)
    origin = node.step_origin
    odom(node, x=-.1, dt=node.param('step_timeouts')[0])
    node.tick()
    assert node.phase == 'HALT' and 'timed out' in node.halt_reason
    if override == 'external':
        node.lane_state_callback(Bool(data=False))
        node.lane_state_callback(Bool(data=True))
    elif override == 'manual':
        node.state_callback(Bool(data=False))
    node.state_callback(Bool(data=True))
    for _ in range(3):
        odom(node)
        node.tick()
    assert node.phase == 'PARKING' and node.step_origin == origin
    assert node.step_started == pytest.approx(node.clock[0])
    node.tick()
    assert node.velocity_pub.messages[-1].linear.x == pytest.approx(-.1)


def test_stop_timeout_retry_rechecks_stop_without_advancing_initial_step(node):
    approach(node)
    odom(node, v=.1)
    node.camera_callback(image())
    odom(node, v=.1, dt=node.param('stop_timeout'))
    node.tick()
    assert node.phase == 'HALT' and node.halt_reason == 'stop confirmation timed out'
    node.state_callback(Bool(data=True))
    for _ in range(3):
        odom(node)
        node.tick()
    assert node.phase == 'PARKING' and node.step_origin is None and node.settling
    odom(node)
    node.tick()
    assert node.step == 0 and node.step_origin is not None and not node.settling


@pytest.mark.parametrize('at_target', [False, True])
def test_resume_during_stop_confirmation_preserves_stage_and_requires_new_stop(node, at_target):
    if at_target:
        reach_last_target(node)
    else:
        approach(node)
        odom(node, v=.1)
        node.camera_callback(image())
    step, origin = node.step, node.step_origin
    node.clock[0] += 1.
    node.tick()
    for _ in range(3):
        odom(node)
        node.tick()
    assert node.phase == 'PARKING' and node.settling
    assert node.step == step and node.step_origin == origin
    node.tick()
    assert node.phase == 'PARKING' and node.step == step
    odom(node)
    node.tick()
    if at_target:
        assert node.phase == 'DONE' and node.step == 6
        assert [msg.data for msg in node.mode_pub.messages].count('intersection') == 1
    else:
        assert node.phase == 'PARKING' and node.step == 0 and not node.settling


def test_empty_camera_roi_halts_before_color_conversion(node, monkeypatch):
    approach(node)
    monkeypatch.setattr(cv2, 'imdecode', lambda *args: np.empty((0, 0, 3), np.uint8))
    node.camera_callback(CompressedImage(data=b'invalid'))
    assert node.phase == 'HALT' and node.halt_reason == 'invalid camera image'
