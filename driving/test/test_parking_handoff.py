"""Parking handoff/ROI tests with no ROS context, model inference, or robot."""
from types import SimpleNamespace
from unittest.mock import Mock
import math

import numpy as np
import pytest
from rclpy.node import Node
from std_msgs.msg import Bool, String
from nav_msgs.msg import Odometry

from driving.driving import DrivingNode
from driving.detection.lane_estimator import LaneConfig, LanePath


@pytest.fixture
def node(monkeypatch):
    parameters = {}
    monkeypatch.setattr(Node, '__init__', lambda *a, **kw: None)
    monkeypatch.setattr(Node, 'declare_parameter', lambda self, name, value, *a: parameters.setdefault(name, value))
    monkeypatch.setattr(Node, 'get_parameter', lambda self, name: SimpleNamespace(value=parameters[name]))
    monkeypatch.setattr(Node, 'create_publisher', lambda *a, **kw: SimpleNamespace(publish=Mock()))
    monkeypatch.setattr(Node, 'create_subscription', lambda *a, **kw: None)
    monkeypatch.setattr(Node, 'create_timer', lambda *a, **kw: None)
    monkeypatch.setattr(Node, 'get_logger', lambda self: Mock())
    monkeypatch.setattr(Node, 'get_clock', lambda self: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=10000000000)))
    monkeypatch.setattr('driving.driving.get_package_share_directory', lambda name: '/unused')
    n = DrivingNode()
    n.publish_yolo_bev = Mock()  # This fixture supplies lane geometry, not a BEV image.
    return n


def prepare_yolo(node, target=None, mode='lost'):
    node.yolo_lane = SimpleNamespace(
        detectLane=Mock(return_value=(target, mode)),
        lane=SimpleNamespace(lane_offsets=lambda: (None, None),
                             lane_headings=lambda: (None, None)),
        reset=Mock())
    node.publish_debug_yolo = Mock()
    node.yolo_startup_speed = None
    node.cmd_vel_pub.publish.reset_mock()


def process_frame(node):
    node.yolo_msg = SimpleNamespace(header=SimpleNamespace(
        stamp=SimpleNamespace(sec=10, nanosec=0)))
    node.process_yolo()


def test_parking_near_left_roi_survives_lane_resume(node):
    node.roi_side = 'right'
    node.parking_mode_callback(String(data='left'))
    node.lane_state_callback(Bool(data=True))
    node.right_sign_callback(Bool(data=True))
    side, half = node.lane_roi_settings()
    assert (side, half) == (None, 'left')
    prepare_yolo(node)
    process_frame(node)
    args = node.yolo_lane.detectLane.call_args.kwargs
    parking = LanePath(LaneConfig(roi_top_row=args['roi_top_row'], roi_half_width=250))
    construction = LanePath(LaneConfig(roi_top_row=150, roi_half_width=250))
    parking.update([], side, 1.4, half)
    construction.update([], None, 1.4, 'left')
    assert parking.roi_x_range() == construction.roi_x_range() == (76, 326)
    assert parking.roi_top() == 250 and construction.roi_top() == 150
    assert not parking.roi_mask()[150:250].any()
    assert np.array_equal(parking.roi_mask()[250:], construction.roi_mask()[250:])


@pytest.mark.parametrize('mode', ['', 'left', 'maneuver', 'exit', 'halt', 'intersection'])
def test_only_approach_changes_roi_and_uses_parking_search(node, mode):
    node.lane_state_callback(Bool(data=True))
    node.parking_mode_callback(String(data=mode))
    prepare_yolo(node)
    process_frame(node)
    args = node.yolo_lane.detectLane.call_args.kwargs
    assert args['roi_top_row'] == (250 if mode == 'left' else 150)
    if mode in ('maneuver', 'halt'):
        node.cmd_vel_pub.publish.assert_not_called()
    else:
        command = node.cmd_vel_pub.publish.call_args.args[0]
        speed_param = 'parking.search_speed' if mode == 'left' else 'pure_pursuit.v_lost'
        assert command.linear.x == pytest.approx(node.get_parameter(speed_param).value)
        assert command.angular.z == pytest.approx(
            node.get_parameter('parking.search_angular').value if mode == 'left' else 0.0)
        assert node.pure_pursuit.pursuit.w == command.angular.z


def test_reacquired_lane_immediately_replaces_straight_fallback(node):
    node.lane_state_callback(Bool(data=True))
    node.parking_mode_callback(String(data='left'))
    prepare_yolo(node)
    node.yolo_lane.detectLane.side_effect = [
        (None, 'lost'), ((.2, -.02), 'yellow_only'), (None, 'lost')]
    for _ in range(3):
        process_frame(node)
    commands = [call.args[0] for call in node.cmd_vel_pub.publish.call_args_list]
    assert commands[0].angular.z == pytest.approx(node.get_parameter('parking.search_angular').value)
    assert commands[1].angular.z < 0  # 오른쪽 경로를 찾은 프레임부터 기존 추종으로 복귀한다.
    assert commands[2].angular.z == pytest.approx(node.get_parameter('parking.search_angular').value)


def test_search_fallback_respects_external_stop_and_startup_limit(node):
    node.parking_mode_callback(String(data='left'))
    prepare_yolo(node)
    process_frame(node)
    node.cmd_vel_pub.publish.assert_not_called()
    node.lane_state_callback(Bool(data=True))
    node.yolo_startup_speed = 0.0
    process_frame(node)
    command = node.cmd_vel_pub.publish.call_args.args[0]
    assert command.linear.x == pytest.approx(.03)
    assert command.angular.z == pytest.approx(
        node.get_parameter('parking.search_angular').value * .03 /
        node.get_parameter('parking.search_speed').value)
    assert node.pure_pursuit.pursuit.w == command.angular.z
    node.lane_state_callback(Bool(data=False))
    node.cmd_vel_pub.publish.reset_mock()
    process_frame(node)
    node.cmd_vel_pub.publish.assert_not_called()


def test_inference_error_stops_instead_of_driving_straight(node):
    node.lane_state_callback(Bool(data=True))
    node.parking_mode_callback(String(data='left'))
    prepare_yolo(node)
    node.yolo_lane.detectLane.side_effect = ValueError('bad image')
    process_frame(node)
    command = node.cmd_vel_pub.publish.call_args.args[0]
    assert command.linear.x == command.angular.z == 0.0


@pytest.mark.parametrize('name,value', [
    ('parking.search_speed', -.05), ('parking.search_angular', -.5),
    ('parking.search_angular', float('nan'))])
def test_invalid_search_settings_stop_instead_of_driving(node, monkeypatch, name, value):
    node.lane_state_callback(Bool(data=True))
    node.parking_mode_callback(String(data='left'))
    prepare_yolo(node)
    original = node.get_parameter
    monkeypatch.setattr(node, 'get_parameter', lambda key:
                        SimpleNamespace(value=value) if key == name else original(key))
    process_frame(node)
    command = node.cmd_vel_pub.publish.call_args.args[0]
    assert command.linear.x == pytest.approx(0.0)
    assert command.angular.z == pytest.approx(0.0)


@pytest.mark.parametrize('left_first', [True, False])
def test_exit_handoff_works_in_either_topic_order(node, left_first):
    node.lane_state_callback(Bool(data=True))
    node.parking_mode_callback(String(data='exit'))
    if left_first:
        node.left_sign_callback(Bool(data=True))
        assert node.roi_side is None
    node.parking_mode_callback(String(data='intersection'))
    if not left_first:
        node.left_sign_callback(Bool(data=True))
    assert node.is_intersection
    assert node.roi_side == 'left'
    assert node.lane_roi_settings() == ('left', None)
    assert node.lane_driving_enabled()
    node.intersection_state_callback(Bool(data=False))
    assert not node.is_intersection  # later normal intersection messages still work


@pytest.mark.parametrize('mode, allowed', [('left', True), ('maneuver', False), ('exit', True), ('halt', False), ('intersection', True)])
def test_yolo_velocity_is_suppressed_only_while_parking_owns_motion(node, mode, allowed):
    node.lane_state_callback(Bool(data=True))
    node.parking_mode_callback(String(data=mode))
    node.yolo_lane = SimpleNamespace(
        detectLane=lambda *a, **kw: ((.2, .02), 'yellow_only'),
        lane=SimpleNamespace(lane_offsets=lambda: (.14, None), lane_headings=lambda: (0., None)),
        reset=Mock())
    node.publish_debug_yolo = Mock()
    node.cmd_vel_pub.publish.reset_mock()
    node.yolo_msg = SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=10, nanosec=0)))
    node.process_yolo()
    assert node.lane_info_pub.publish.called and node.publish_debug_yolo.called
    assert node.cmd_vel_pub.publish.called == allowed
    node.lane_state_callback(Bool(data=False))
    assert not node.lane_driving_enabled()


def test_invalid_duplicate_and_stop_modes(node):
    node.parking_mode_callback(String(data='left'))
    node.yolo_lane = SimpleNamespace(reset=Mock())
    node.parking_mode_callback(String(data='garbage'))
    node.parking_mode_callback(String(data='left'))
    assert not node.yolo_lane.reset.called
    node.parking_mode_callback(String(data='maneuver'))
    node.lane_state_callback(Bool(data=True))
    assert not node.lane_driving_enabled()
    node.parking_mode_callback(String(data='halt'))
    node.parking_mode_callback(String(data=''))
    assert node.lane_driving_enabled()
    node.right_sign_callback(Bool(data=True))
    assert node.turn_side == 'right' and node.roi_side is None


def test_parking_exit_left_follow_expires_after_five_seconds(node, monkeypatch):
    node.lane_state_callback(Bool(data=True))
    node.parking_mode_callback(String(data='maneuver'))
    node.parking_mode_callback(String(data='intersection'))
    assert node.get_parameter('parking.exit_follow_sec').value == 5.
    assert node.parking_exit_turn_since == 10.
    monkeypatch.setattr(node, 'now_seconds', lambda: 14.999)
    node.check_sign_roi_timeout()
    assert node.roi_side == 'left' and node.is_intersection
    # Neither signs nor a duplicate handoff can extend the parking window.
    node.left_sign_callback(Bool(data=True))
    node.right_sign_callback(Bool(data=True))
    node.parking_mode_callback(String(data='intersection'))
    assert node.roi_side == 'left' and node.parking_exit_turn_since == 10.
    monkeypatch.setattr(node, 'now_seconds', lambda: 15.)
    node.check_sign_roi_timeout()
    assert node.roi_side is None and not node.is_intersection
    assert node.parking_exit_turn_since is None
    assert node.lane_driving_enabled()
    prepare_yolo(node, target=(.32, .1), mode='both')
    process_frame(node)
    assert node.yolo_lane.detectLane.call_args.kwargs['roi_side'] is None
    assert node.pure_pursuit.pursuit.v_max == .40


@pytest.mark.parametrize('side', ['left', 'right'])
def test_ordinary_sign_roi_keeps_its_thirty_second_policy(node, monkeypatch, side):
    node.intersection_state_callback(Bool(data=True))
    getattr(node, side + '_sign_callback')(Bool(data=True))
    assert node.parking_exit_turn_since is None
    assert node.get_parameter('sign.roi_hold_sec').value == 30.
    monkeypatch.setattr(node, 'now_seconds', lambda: 15.)
    node.check_sign_roi_timeout()
    assert node.roi_side == side and node.is_intersection
    monkeypatch.setattr(node, 'now_seconds', lambda: 40.)
    node.check_sign_roi_timeout()
    assert node.roi_side is None and node.is_intersection  # original external-state policy


@pytest.mark.parametrize('mode', ['', 'left', 'maneuver', 'exit', 'halt'])
def test_new_parking_mode_cancels_the_exit_window(node, mode):
    node.parking_mode_callback(String(data='intersection'))
    node.parking_mode_callback(String(data=mode))
    assert node.parking_exit_turn_since is None
    assert node.roi_side is None and not node.is_intersection


def test_external_intersection_stop_ends_parking_exit_roi_early(node):
    node.parking_mode_callback(String(data='intersection'))
    node.intersection_state_callback(Bool(data=False))
    assert node.parking_exit_turn_since is None
    assert node.roi_side is None and not node.is_intersection


@pytest.fixture
def turn_node(node, monkeypatch):
    node.test_clock = [10.0]
    monkeypatch.setattr(node, 'get_clock', lambda: SimpleNamespace(now=lambda:
                        SimpleNamespace(nanoseconds=int(node.test_clock[0] * 1e9))))
    node.lane_state_callback(Bool(data=True))
    return node


def turn_odom(node, degrees, w=0.0, v=0.0, dt=0.05):
    node.test_clock[0] += dt
    msg = Odometry()
    stamp = int(node.test_clock[0] * 1e9)
    msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(stamp, 1000000000)
    msg.pose.pose.orientation.z = math.sin(math.radians(degrees) / 2)
    msg.pose.pose.orientation.w = math.cos(math.radians(degrees) / 2)
    msg.twist.twist.angular.z, msg.twist.twist.linear.x = float(w), float(v)
    node.turn_odom_callback(msg)
    return msg


def begin_sign_turn(node, side, origin=0.0):
    turn_odom(node, origin)
    getattr(node, side + '_sign_callback')(Bool(data=True))
    assert node.turn_side == side and not node.lane_driving_enabled()
    assert node.roi_side is None
    node.check_sign_roi_timeout()
    assert node.turn_origin is None  # Pre-stop odometry cannot establish the reference.
    turn_odom(node, origin)
    node.check_sign_roi_timeout()
    assert node.turn_origin == pytest.approx(math.radians(origin))
    node.check_sign_roi_timeout()


@pytest.mark.parametrize('side,angle,origin', [
    ('left', 25.0, 0.0), ('right', 35.0, 0.0),
    ('left', 30.0, 179.0), ('right', 30.0, -179.0)])
def test_sign_turn_then_existing_direction_roi(turn_node, monkeypatch, side, angle, origin):
    node = turn_node
    original = node.get_parameter
    monkeypatch.setattr(node, 'get_parameter', lambda name: SimpleNamespace(value=angle)
                        if name == 'turn.' + side + '_angle_deg' else original(name))
    begin_sign_turn(node, side, origin)
    command = node.cmd_vel_pub.publish.call_args.args[0]
    direction = 1 if side == 'left' else -1
    assert command.linear.x == 0 and command.angular.z == pytest.approx(direction * .4)
    prepare_yolo(node, target=(.3, .05), mode='both')
    process_frame(node)
    node.cmd_vel_pub.publish.assert_not_called()  # Lane inference continues without competing output.
    assert node.lane_info_pub.publish.called and node.publish_debug_yolo.called
    # A parameter update cannot change a turn already in progress.
    monkeypatch.setattr(node, 'get_parameter', lambda name: SimpleNamespace(value=60.0)
                        if name == 'turn.' + side + '_angle_deg' else original(name))
    assert node.turn_angle == pytest.approx(math.radians(angle))
    goal = origin + direction * angle
    turn_odom(node, goal, w=direction * .1)
    node.check_sign_roi_timeout()
    assert node.turn_settling and node.roi_side is None
    node.check_sign_roi_timeout()
    assert node.turn_side == side  # Wait for a new stopped odometry sample.
    turn_odom(node, goal)
    finished_at = node.test_clock[0]
    node.check_sign_roi_timeout()
    assert node.turn_side is None and node.lane_driving_enabled()
    assert node.lane_roi_settings() == (side, None)
    assert node.roi_side_since == pytest.approx(finished_at)
    assert node.yolo_msg is None
    process_frame(node)
    assert node.yolo_lane.detectLane.call_args.kwargs['roi_side'] == side
    cfg = LanePath(LaneConfig(roi_top_row=150))
    cfg.update([], side, node.get_parameter('sign.roi_height_scale').value)
    assert cfg.roi_x_range() == ((0, 200) if side == 'left' else (326, 640))
    # A repeated True cannot start another rotation or extend the ROI timer.
    getattr(node, side + '_sign_callback')(Bool(data=True))
    assert node.turn_side is None and node.roi_side_since == pytest.approx(finished_at)
    node.test_clock[0] = finished_at + 30.0
    node.check_sign_roi_timeout()
    assert node.roi_side is None
    getattr(node, side + '_sign_callback')(Bool(data=False))
    getattr(node, side + '_sign_callback')(Bool(data=True))
    assert node.turn_angle == pytest.approx(math.radians(60.0))


@pytest.mark.parametrize('fault', ['stale', 'duplicate', 'invalid', 'timeout', 'overshoot'])
def test_turn_fault_stops_but_overshoot_resumes_lane_following(turn_node, fault):
    node = turn_node
    begin_sign_turn(node, 'left')
    if fault in ('stale', 'duplicate'):
        msg = turn_odom(node, 1.0, w=.2)
        node.test_clock[0] += .6
        if fault == 'duplicate':
            node.turn_odom_callback(msg)
    elif fault == 'invalid':
        node.turn_odom_callback(Odometry())  # Zero quaternion.
    elif fault == 'timeout':
        node.test_clock[0] += 16.0
        turn_odom(node, 1.0)
    else:
        final_angle = math.degrees(node.turn_angle) + 10.0
        turn_odom(node, final_angle)
        node.check_sign_roi_timeout()
        turn_odom(node, final_angle)
    node.check_sign_roi_timeout()
    if fault == 'overshoot':
        assert node.turn_side is None and node.lane_driving_enabled()
        assert node.lane_roi_settings() == ('left', None)
        return
    assert node.turn_side == 'halt' and not node.lane_driving_enabled()
    command = node.cmd_vel_pub.publish.call_args.args[0]
    assert command.linear.x == command.angular.z == 0.0
    node.lane_state_callback(Bool(data=False))
    assert node.turn_side is None
    node.lane_state_callback(Bool(data=True))
    assert node.lane_driving_enabled()


def test_turn_cancels_for_stop_parking_or_algorithm_change(turn_node):
    node = turn_node
    begin_sign_turn(node, 'right')
    node.lane_state_callback(Bool(data=False))
    assert node.turn_side is None
    node.lane_state_callback(Bool(data=True))
    node.right_sign_callback(Bool(data=False))
    begin_sign_turn(node, 'right')
    node.parking_mode_callback(String(data='left'))
    assert node.turn_side is None and node.lane_roi_settings() == (None, 'left')
    node.parking_mode_callback(String(data=''))
    node.right_sign_callback(Bool(data=False))
    begin_sign_turn(node, 'right')
    node.lane_detection_selection_callback(String(data='basic_sat'))
    assert node.turn_side is None
