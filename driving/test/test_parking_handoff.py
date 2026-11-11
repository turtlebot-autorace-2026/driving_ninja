"""Parking handoff/ROI tests with no ROS context, model inference, or robot."""
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest
from rclpy.node import Node
from std_msgs.msg import Bool, String

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
    n.parameters = parameters
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
    assert node.intersection_side == 'left'
    assert node.lane_roi_settings() == ('left', None)
    assert node.lane_driving_enabled()
    node.intersection_state_callback(Bool(data=False))
    assert not node.is_intersection  # later normal intersection messages still work
    assert node.intersection_side is None and node.roi_side is None


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
    assert node.roi_side == 'right'


def turn_command(node, side, mode='yellow_only', target=None):
    callback = node.left_sign_callback if side == 'left' else node.right_sign_callback
    callback(Bool(data=True))
    if target is None and mode != 'lost':
        target = (.32, .10 if side == 'left' else -.10)
    prepare_yolo(node, target=target, mode=mode)
    node.pure_pursuit.reset()
    process_frame(node)
    command = node.cmd_vel_pub.publish.call_args.args[0]
    return command.linear.x, command.angular.z


@pytest.mark.parametrize('side', ['left', 'right'])
def test_turn_sign_selects_profile_and_existing_intersection_defaults(node, side):
    node.lane_state_callback(Bool(data=True))
    v, w = turn_command(node, side)
    assert node.is_intersection and node.intersection_side == node.roi_side == side
    assert node.intersection_settings() == (.30, .7, 3., .8)
    assert node.yolo_lane.detectLane.call_args.kwargs['roi_side'] == side
    # Regression against the previous controller's default calculation, including smoothing.
    curvature = 1.7 * 2 * .10 / (.32**2 + .10**2) * .7
    expected_v = .30 / (1 + .6 * curvature) * .8
    assert v == pytest.approx(expected_v)
    assert w == pytest.approx(expected_v * curvature * .6 * (1 if side == 'left' else -1))


@pytest.mark.parametrize('side', ['left', 'right'])
@pytest.mark.parametrize('key,value', [
    ('v_max', .15), ('curvature_scale', 1.2), ('max_ang', .05), ('v_single', .4)])
def test_tuning_one_direction_does_not_change_other_direction(node, side, key, value):
    node.lane_state_callback(Bool(data=True))
    other = 'right' if side == 'left' else 'left'
    before_side = turn_command(node, side)
    before_other = turn_command(node, other)
    node.parameters['intersection.' + side + '.' + key] = value
    after_side = turn_command(node, side)
    after_other = turn_command(node, other)
    assert after_side != pytest.approx(before_side)
    assert after_other == pytest.approx(before_other)
    if key == 'curvature_scale':
        assert abs(after_side[1]) > abs(before_side[1])
    elif key == 'max_ang':
        assert abs(after_side[1]) == pytest.approx(.05 * .6)
    else:
        assert after_side[0] < before_side[0]


@pytest.mark.parametrize('side', ['left', 'right'])
@pytest.mark.parametrize('key,value', [
    ('v_max', float('nan')), ('curvature_scale', -.1),
    ('max_ang', float('inf')), ('v_single', 1.1)])
def test_invalid_active_profile_stops_without_falling_back_to_other_side(node, side, key, value):
    node.lane_state_callback(Bool(data=True))
    node.parameters['intersection.' + side + '.' + key] = value
    assert turn_command(node, side) == (0., 0.)


@pytest.mark.parametrize('side', ['left', 'right'])
def test_directional_lost_policy_remains_common_straight_speed(node, side):
    node.lane_state_callback(Bool(data=True))
    node.parameters['intersection.' + side + '.v_max'] = .6
    node.parameters['intersection.' + side + '.curvature_scale'] = 1.5
    assert turn_command(node, side, mode='lost') == (.05, 0.)


@pytest.mark.parametrize('side', ['left', 'right'])
def test_sign_release_does_not_end_turn_but_timeout_clears_profile_and_roi(node, monkeypatch, side):
    node.lane_state_callback(Bool(data=True))
    callback = node.left_sign_callback if side == 'left' else node.right_sign_callback
    callback(Bool(data=True))
    callback(Bool(data=False))
    assert node.is_intersection and node.intersection_side == side
    hold = node.get_parameter('sign.roi_hold_sec').value
    monkeypatch.setattr(node, 'now_seconds', lambda: 10. + hold - .01)
    node.check_sign_roi_timeout()
    assert node.is_intersection
    monkeypatch.setattr(node, 'now_seconds', lambda: 10. + hold)
    node.check_sign_roi_timeout()
    assert not node.is_intersection
    assert node.intersection_side is None and node.roi_side is None
    prepare_yolo(node, target=(.32, .10), mode='both')
    node.pure_pursuit.reset()
    process_frame(node)
    # Normal mode resumes the common 0.40 m/s speed basis and unscaled curvature.
    curvature = 1.7 * 2 * .10 / (.32**2 + .10**2)
    command = node.cmd_vel_pub.publish.call_args.args[0]
    assert command.linear.x == pytest.approx(.40 / (1 + .6*curvature))


@pytest.mark.parametrize('side', ['left', 'right'])
def test_explicit_intersection_stop_clears_side_without_resetting_smoothing(node, side):
    getattr(node, side + '_sign_callback')(Bool(data=True))
    node.pure_pursuit.pursuit.prev_ang = .23
    node.intersection_state_callback(Bool(data=False))
    assert not node.is_intersection
    assert node.intersection_side is None and node.roi_side is None
    assert node.pure_pursuit.pursuit.prev_ang == .23
    node.intersection_state_callback(Bool(data=True))
    assert node.intersection_side is None  # no stale direction on an external restart
    assert node.intersection_settings() == (.30, .7, 3., .8)


def test_directionless_external_intersection_uses_existing_common_parameters(node):
    node.parameters['pure_pursuit.v_max_intersection'] = .22
    node.parameters['pure_pursuit.max_ang'] = .15
    node.parameters['pure_pursuit.v_single'] = .6
    node.parameters['intersection.left.v_max'] = .7
    node.intersection_state_callback(Bool(data=True))
    assert node.intersection_side is None
    assert node.intersection_settings() == (.22, .7, .15, .6)


@pytest.mark.parametrize('mode', ['left', 'maneuver', 'exit', 'halt', ''])
def test_parking_handoff_clears_old_direction_and_ignores_ineligible_signs(node, mode):
    node.right_sign_callback(Bool(data=True))
    # For empty mode, first enter another parking phase so it is a real transition.
    if mode == '':
        node.parking_mode_callback(String(data='exit'))
    node.parking_mode_callback(String(data=mode))
    assert not node.is_intersection and node.intersection_side is None
    if mode:
        node.left_sign_callback(Bool(data=True))
        node.right_sign_callback(Bool(data=True))
        assert not node.is_intersection and node.intersection_side is None


@pytest.mark.parametrize('left_first', [True, False])
def test_parking_exit_selects_left_settings_in_either_delivery_order(node, left_first):
    node.parameters['intersection.left.v_max'] = .23
    node.parameters['intersection.right.v_max'] = .41
    node.parking_mode_callback(String(data='exit'))
    if left_first:
        node.left_sign_callback(Bool(data=True))
    node.parking_mode_callback(String(data='intersection'))
    if not left_first:
        node.left_sign_callback(Bool(data=True))
    assert node.is_intersection and node.intersection_side == 'left'
    assert node.intersection_settings()[0] == .23


def test_waiting_lane_still_computes_directional_control_without_velocity_output(node):
    node.right_sign_callback(Bool(data=True))
    prepare_yolo(node, target=(.32, -.10), mode='white_only')
    process_frame(node)
    assert node.pure_pursuit.pursuit.w < 0
    node.cmd_vel_pub.publish.assert_not_called()
    node.publish_debug_yolo.assert_called_once()
