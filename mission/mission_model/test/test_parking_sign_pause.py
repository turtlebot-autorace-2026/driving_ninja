"""Use in-memory ROS/model substitutes to check parking pause and fresh detections."""
from types import SimpleNamespace
from unittest.mock import Mock

import cv2
import numpy as np
import pytest
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

from mission_model.detection import CLASSES, MissionDetection


class Recorder:
    def __init__(self, topic_name):
        self.topic_name = topic_name
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


@pytest.fixture
def node(monkeypatch):
    parameters = {'model_path': '/unused/signs.onnx'}
    subscriptions = {}
    monkeypatch.setattr(Node, '__init__', lambda *a, **kw: None)
    monkeypatch.setattr(Node, 'declare_parameter', lambda self, name, value:
                        parameters.setdefault(name, value))
    monkeypatch.setattr(Node, 'get_parameter', lambda self, name:
                        SimpleNamespace(value=parameters[name]))
    monkeypatch.setattr(Node, 'create_publisher', lambda self, msg_type, topic, qos:
                        Recorder(topic))
    monkeypatch.setattr(Node, 'create_subscription', lambda self, msg_type, topic, callback, qos:
                        subscriptions.setdefault(topic, (callback, qos)))
    monkeypatch.setattr(Node, 'create_timer', lambda *a, **kw: None)
    monkeypatch.setattr(Node, 'get_logger', lambda self: Mock())
    monkeypatch.setattr('mission_model.yolo_onnx.YoloOnnx', lambda *a:
                        SimpleNamespace(names=CLASSES, imgsz=320, providers=['test']))
    n = MissionDetection()
    n.detect = Mock(return_value=[(0, .9, 10, 10, 120, 120),
                                 (1, .9, 200, 10, 310, 120)])
    callback, qos = subscriptions['/parking/lane_mode']
    assert qos.durability == DurabilityPolicy.TRANSIENT_LOCAL and qos.depth == 1
    assert callback == n.parking_mode_callback
    return n


def frame():
    pixels = np.zeros((480, 640, 3), np.uint8)
    return CompressedImage(data=cv2.imencode('.jpg', pixels)[1].tobytes())


def process_frame(node):
    node.videoSubscriber_callback(frame())
    node.process()


@pytest.mark.parametrize('mode', ['left', 'maneuver', 'halt'])
def test_parking_pauses_inference_clears_signs_and_keeps_live_debug(node, mode):
    for _ in range(node.confirm_frames):
        process_frame(node)
    assert node.active[:2] == [True, True]
    node.videoSubscriber_callback(frame())
    node.parking_mode_callback(String(data=mode))
    assert node.latest_msg is None and not any(node.active)
    assert not any(node.seen_count) and not any(node.missed_count)
    assert [m.data for m in node.signPublishers[1].messages] == [False, True, False]
    node.detect.reset_mock()
    debug_count = len(node.debugPublisher.messages)
    for _ in range(node.confirm_frames + 1):
        process_frame(node)
    node.detect.assert_not_called()
    assert not any(node.active) and not any(node.seen_count) and not any(node.missed_count)
    assert len(node.debugPublisher.messages) == debug_count + node.confirm_frames + 1
    debug = node.debugPublisher.messages[-1]
    assert cv2.imdecode(np.frombuffer(debug.data, np.uint8), cv2.IMREAD_COLOR).shape == (480, 640, 3)


@pytest.mark.parametrize('mode', ['exit', '', 'intersection'])
def test_resume_discards_pending_frame_and_reconfirms_even_a_continuously_visible_sign(node, mode):
    for _ in range(node.confirm_frames):
        process_frame(node)
    node.parking_mode_callback(String(data='left'))
    node.parking_mode_callback(String(data='maneuver'))
    node.videoSubscriber_callback(frame())  # parked image must not count at resume
    node.parking_mode_callback(String(data=mode))
    node.detect.reset_mock()
    node.process()
    node.detect.assert_not_called()
    assert not node.signPublishers[1].messages[-1].data
    for _ in range(node.confirm_frames - 1):
        process_frame(node)
        assert not node.active[1]
    process_frame(node)
    assert node.active[1]
    assert [m.data for m in node.signPublishers[1].messages] == [False, True, False, True]


def test_partial_confirmation_before_pause_cannot_complete_after_resume(node):
    for _ in range(node.confirm_frames - 1):
        process_frame(node)
    node.parking_mode_callback(String(data='left'))
    node.parking_mode_callback(String(data='exit'))
    process_frame(node)
    assert not any(node.active)
    assert node.seen_count[1] == 1


def test_duplicate_resume_preserves_debounce_and_normal_release_still_works(node):
    process_frame(node)
    node.parking_mode_callback(String(data=''))
    for _ in range(node.confirm_frames - 1):
        process_frame(node)
    assert node.active[1]
    node.detect.return_value = []
    for _ in range(node.release_frames - 1):
        process_frame(node)
        assert node.active[1]
    process_frame(node)
    assert not any(node.active)


def test_unknown_mode_does_not_resume_paused_detection(node):
    node.parking_mode_callback(String(data='left'))
    node.parking_mode_callback(String(data='unknown'))
    process_frame(node)
    node.detect.assert_not_called()
