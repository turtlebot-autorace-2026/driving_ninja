"""Check command timing with an in-memory clock and publishers; no robot output."""
import json
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from rclpy.node import Node
from std_msgs.msg import Bool, String

from construction.detection import BRAKE, DONE, FOLLOW, TURN_OUT, ConstructionDetection


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
    monkeypatch.setattr(Node, 'declare_parameter', lambda self, name, value:
                        parameters.setdefault(name, value))
    monkeypatch.setattr(Node, 'get_parameter', lambda self, name:
                        SimpleNamespace(value=parameters[name]))
    monkeypatch.setattr(Node, 'create_publisher', lambda *a, **kw: Recorder())
    monkeypatch.setattr(Node, 'create_subscription', lambda *a, **kw: None)
    monkeypatch.setattr(Node, 'create_timer', lambda *a, **kw: None)
    monkeypatch.setattr(Node, 'get_logger', lambda self: Mock())
    monkeypatch.setattr(Node, 'get_clock', lambda self: SimpleNamespace(
        now=lambda: SimpleNamespace(nanoseconds=int(clock[0] * 1e9))))
    n = ConstructionDetection()
    n.clock = clock
    n.constructionSubscriber_callback(Bool(data=True))
    n.velPublisher.messages.clear()
    n.pure_pursuit.makeTwist = Mock(wraps=n.pure_pursuit.makeTwist)
    return n


def target(node, point=(0.3, 0.04), mode='both', half=None, yellow=0.14, white=-0.14):
    node.laneTargetSubscriber_callback(String(data=json.dumps({
        'valid': True, 'roi_half': half, 'target': point, 'mode': mode,
        'yellow': yellow, 'white': white,
    })))


def test_one_command_per_target_and_no_timer_acceleration(node):
    target(node)
    assert len(node.velPublisher.messages) == 1
    first = node.velPublisher.messages[-1]
    assert first.linear.x == pytest.approx(0.03)
    previous_angle = node.pure_pursuit.pursuit.prev_ang
    for _ in range(4):
        node.clock[0] += 0.05
        node.moving()
    assert len(node.velPublisher.messages) == 1
    assert node.startup_speed == pytest.approx(0.03)
    assert node.pure_pursuit.pursuit.prev_ang == previous_angle
    assert node.pure_pursuit.makeTwist.call_count == 1

    target(node)
    assert len(node.velPublisher.messages) == 2
    assert node.velPublisher.messages[-1].linear.x == pytest.approx(0.06)
    assert node.pure_pursuit.makeTwist.call_count == 2


def test_timeout_stops_without_new_target_and_next_target_restarts(node):
    target(node)
    node.clock[0] += 0.51
    node.moving()
    stopped = node.velPublisher.messages[-1]
    assert stopped.linear.x == stopped.angular.z == 0.0
    assert node.follow_twist is None
    target(node)
    assert node.velPublisher.messages[-1].linear.x == pytest.approx(0.03)


def test_first_target_starts_without_initial_wait(node):
    assert node.clock[0] == 10.0  # 공사 시작 시각에서 시간을 진행하지 않는다.
    target(node)
    assert len(node.velPublisher.messages) == 1
    assert node.velPublisher.messages[-1].linear.x == pytest.approx(0.03)
    node.pure_pursuit.makeTwist.assert_called_once()


@pytest.mark.parametrize('via_target', [False, True])
def test_obstacle_stops_before_any_new_follow_command(node, via_target):
    if not via_target:
        target(node)
        node.velPublisher.messages.clear()
    node.front_distance = 0.2
    node.yaw = 0.0
    if via_target:
        target(node)
    else:
        node.moving()
    assert node.state == BRAKE
    assert len(node.velPublisher.messages) == 1
    assert node.velPublisher.messages[-1].linear.x == 0.0
    node.linear_speed = node.angular_speed = 0.0
    node.moving()
    assert node.state == TURN_OUT
    node.moving()
    assert node.velPublisher.messages[-1].linear.x == 0.0
    assert node.velPublisher.messages[-1].angular.z != 0.0
    count = len(node.velPublisher.messages)
    target(node)
    assert len(node.velPublisher.messages) == count


def test_lost_lane_uses_existing_policy_once_per_target(node):
    target(node, point=None, mode='lost', yellow=None, white=None)
    assert node.follow_twist.linear.x == pytest.approx(node.param('lost_speed'))
    assert node.velPublisher.messages[-1].angular.z == 0.0
    node.moving()
    assert len(node.velPublisher.messages) == 1


def test_roi_handoff_rejects_previous_roi_and_resumes_on_new_target(node):
    node.publishLaneRoi('left')
    target(node)
    assert not node.velPublisher.messages
    target(node, half='left')
    assert len(node.velPublisher.messages) == 1
    node.moving()
    assert len(node.velPublisher.messages) == 1
    assert node.state == FOLLOW


def test_invalid_input_stops(node):
    target(node)
    node.laneTargetSubscriber_callback(String(data=json.dumps({'valid': False})))
    assert node.velPublisher.messages[-1].linear.x == 0.0
    assert node.follow_twist is None


@pytest.mark.parametrize('via_target', [False, True])
def test_finish_handoff_precedes_follow_publication(node, via_target):
    target(node)
    node.velPublisher.messages.clear()
    node.avoid_count = 1
    node.last_obstacle_time = node.clock[0] - node.param('finish_timeout')
    if via_target:
        target(node)
    else:
        node.moving()
    assert node.state == DONE
    assert len(node.velPublisher.messages) == 1
    assert node.velPublisher.messages[-1].linear.x == 0.0
    assert node.laneStatePublisher.messages[-1].data is True
