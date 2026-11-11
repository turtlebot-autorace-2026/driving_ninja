"""Exercise callbacks with in-memory publishers; no ROS context or robot is started."""
import json
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import cv2
import numpy as np
import pytest
from geometry_msgs.msg import Twist
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
def node(monkeypatch):
    parameters = {}
    clock = [10.0]
    monkeypatch.setattr(Node, '__init__', lambda *a, **kw: None)
    monkeypatch.setattr(Node, 'declare_parameter', lambda self, name, value, *a: parameters.setdefault(name, value))
    monkeypatch.setattr(Node, 'get_parameter', lambda self, name: SimpleNamespace(value=parameters[name]))
    monkeypatch.setattr(Node, 'create_publisher', lambda *a, **kw: Recorder())
    monkeypatch.setattr(Node, 'create_subscription', lambda *a, **kw: None)
    monkeypatch.setattr(Node, 'create_timer', lambda *a, **kw: None)
    monkeypatch.setattr(Node, 'get_logger', lambda self: Mock())
    monkeypatch.setattr(Node, 'get_clock', lambda self: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=int(clock[0]*1e9))))
    n = ParkingDetection()
    n.clock = clock
    return n


def approach(n):
    n.sign_callback(Bool(data=True))
    n.lane_state_callback(Bool(data=True))
    assert n.phase == 'APPROACH'


def image(white_bottom=False):
    pixels = np.zeros((480, 640, 3), np.uint8)
    if white_bottom:
        pixels[432:, 128:512] = 255
    else:
        pixels[:400] = 255  # bright outside the original stop-line ROI
    return CompressedImage(data=cv2.imencode('.jpg', pixels)[1].tobytes())


def start_parking(n):
    approach(n)
    n.camera_callback(image(True))
    assert n.phase == 'PARKING'
    assert n.mode_pub.messages[-1].data == 'maneuver'


def finish_movements(n):
    for _ in range(6):
        n.tick()
        n.clock[0] += n.param('durations')[n.step] + .01
        n.tick()
        n.clock[0] += .1


def test_sign_waits_for_permission_and_no_idle_velocity(node):
    node.sign_callback(Bool(data=True))
    node.tick()
    node.camera_callback(image(True))
    assert node.phase == 'IDLE' and not node.velocity_pub.messages
    node.sign_callback(Bool(data=False))
    node.lane_state_callback(Bool(data=True))
    assert node.phase == 'APPROACH'
    assert node.mode_pub.messages[-1].data == 'left'
    node.tick()
    assert not node.velocity_pub.messages


def test_original_white_roi_and_six_commands_then_exit_sign(node):
    approach(node)
    node.camera_callback(image(False))
    assert node.phase == 'APPROACH'
    node.camera_callback(image(True))
    assert node.phase == 'PARKING'
    expected = [(-.2, 0., 1.), (0., 1.2, 1.3), (.2, 0., 1.6),
                (-.2, 0., 1.6), (0., 1.5, .9), (.2, 0., 1.)]
    for i, (v, w, duration) in enumerate(expected):
        node.tick()
        command = node.velocity_pub.messages[-1]
        assert node.step == i and (command.linear.x, command.angular.z) == (v, w)
        node.clock[0] += duration - .01
        node.tick()
        assert node.step == i  # still moving before the original duration
        node.clock[0] += .02
        node.tick()
        command = node.velocity_pub.messages[-1]
        assert (command.linear.x, command.angular.z) == (0, 0)
        node.clock[0] += .1
    assert node.phase == 'EXIT' and node.mode_pub.messages[-1].data == 'exit'
    count = len(node.velocity_pub.messages)
    node.tick()
    node.sign_callback(Bool(data=True))
    assert node.phase == 'EXIT' and len(node.velocity_pub.messages) == count
    node.left_callback(Bool(data=True))
    assert node.phase == 'DONE' and node.mode_pub.messages[-1].data == 'intersection'
    node.sign_callback(Bool(data=True))
    node.tick()
    assert node.phase == 'DONE' and len(node.velocity_pub.messages) == count


@pytest.mark.parametrize('phase', ['IDLE', 'APPROACH', 'PARKING'])
def test_left_seen_before_exit_is_discarded_and_requires_new_detection(node, phase):
    if phase == 'IDLE':
        node.left_callback(Bool(data=True))
    approach(node)
    if phase == 'APPROACH':
        node.left_callback(Bool(data=True))
    node.camera_callback(image(True))
    if phase == 'PARKING':
        node.left_callback(Bool(data=True))
    finish_movements(node)
    assert node.phase == 'EXIT' and node.mode_pub.messages[-1].data == 'exit'
    node.left_callback(Bool(data=False))
    assert node.phase == 'EXIT'
    node.left_callback(Bool(data=True))
    assert node.phase == 'DONE'


@pytest.mark.parametrize('step', range(6))
def test_external_stop_cancels_every_step_without_restart(node, step):
    start_parking(node)
    node.step = step
    node.tick()
    node.lane_state_callback(Bool(data=False))
    assert node.phase == 'HALT' and node.mode_pub.messages[-1].data == 'halt'
    count = len(node.velocity_pub.messages)
    command = node.velocity_pub.messages[-1]
    assert (command.linear.x, command.angular.z) == (0, 0)
    node.lane_state_callback(Bool(data=True))
    node.sign_callback(Bool(data=True))
    node.clock[0] += 5
    node.tick()
    assert node.phase == 'HALT' and len(node.velocity_pub.messages) == count


def test_manual_cancel_and_invalid_camera(node):
    approach(node)
    node.state_callback(Bool(data=False))
    assert node.phase == 'HALT'
    node.phase = 'APPROACH'
    node.camera_callback(CompressedImage(data=b'invalid'))
    assert node.phase == 'HALT'


def test_clock_reset_stops_timed_movement(node):
    start_parking(node)
    node.tick()
    node.clock[0] -= 1
    node.tick()
    assert node.phase == 'HALT'
    assert node.velocity_pub.messages[-1] == Twist()


def test_config_matches_original_defaults(node):
    import yaml
    path = Path(__file__).resolve().parents[1]/'config/parking_sim.yaml'
    values = yaml.safe_load(path.read_text())['parking_detect_node']['ros__parameters']
    assert set(values) == {'direction', 'white_ratio', 'durations', 'linear_speeds', 'angular_speeds'}
    for name, value in values.items():
        assert value == node.param(name)


def test_motion_handoff_publishes_hold_before_zero_and_status(node):
    events = []
    node.mode_pub.publish = lambda msg: events.append(('mode', msg.data))
    node.velocity_pub.publish = lambda msg: events.append(('velocity', msg))
    node.status_pub.publish = lambda msg: events.append(('status', json.loads(msg.data)))
    approach(node)
    events.clear()
    node.camera_callback(image(True))
    assert [event[0] for event in events] == ['mode', 'velocity', 'status']
    assert events[0][1] == 'maneuver' and events[1][1] == Twist()
    assert events[2][1]['phase'] == 'PARKING' and events[2][1]['step'] == 0
    node.tick()
    events.clear()
    node.lane_state_callback(Bool(data=False))
    assert [event[0] for event in events] == ['mode', 'velocity', 'status']
    assert events[0][1] == 'halt' and events[1][1] == Twist()
    assert events[2][1]['phase'] == 'HALT'
