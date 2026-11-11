"""P sign -> left half ROI -> white line -> six movements -> left sign handoff."""
import json
import math
import signal

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool, String


class ParkingDetection(Node):
    def __init__(self):
        super().__init__('parking_detect_node')
        # 원본 순서: 후진, 회전1, 전진, 후진, 회전2, 전진. 시작 시 적용한다.
        defaults = {
            'direction': 'left', 'white_ratio': 0.60,
            'durations': [1.0, 1.3, 1.6, 1.6, 0.9, 1.0],
            'linear_speeds': [-0.2, 0.0, 0.2, -0.2, 0.0, 0.2],
            # 회전1: 1.2rad/s × 1.3초 ≈ 89.4°, 회전2: 1.5 × 0.9 ≈ 77.3°.
            # 원본처럼 두 회전은 같은 방향이다. 각도 도달이 아니라 시간으로 끝난다.
            'angular_speeds': [0.0, 1.2, 0.0, 0.0, 1.5, 0.0],
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value, ParameterDescriptor(read_only=True))
        if self.param('direction') not in ('left', 'right'):
            raise ValueError('direction must be left or right')
        if not 0 < self.param('white_ratio') <= 1:
            raise ValueError('white_ratio must be in (0, 1]')
        for name in ('durations', 'linear_speeds', 'angular_speeds'):
            values = self.param(name)
            if len(values) != 6 or not all(math.isfinite(v) for v in values):
                raise ValueError(name + ' must contain six finite values')
        if min(self.param('durations')) <= 0:
            raise ValueError('durations must be positive')

        self.phase = 'IDLE'  # IDLE -> APPROACH -> PARKING -> EXIT -> DONE; cancel -> HALT
        self.parking_requested = False
        self.lane_enabled = False
        self.step = 0
        self.step_started = None
        self.last_tick = self.now()
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.velocity_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.mode_pub = self.create_publisher(String, '/parking/lane_mode', latched)
        self.status_pub = self.create_publisher(String, '/parking/status', latched)
        self.create_subscription(Bool, '/parking', self.sign_callback, latched)
        self.create_subscription(Bool, '/left', self.left_callback, latched)
        self.create_subscription(Bool, '/parking_state', self.state_callback, 10)
        self.create_subscription(Bool, '/lane_state', self.lane_state_callback, 10)
        self.create_subscription(CompressedImage, '/camera', self.camera_callback,
                                 qos_profile_sensor_data)
        self.set_phase('IDLE', 'ready')
        self.create_timer(0.1, self.tick)

    def param(self, name):
        return self.get_parameter(name).value

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def set_phase(self, phase, reason):
        mode = {'IDLE': '', 'APPROACH': 'left', 'PARKING': 'maneuver',
                'EXIT': 'exit', 'DONE': 'intersection', 'HALT': 'halt'}[phase]
        self.mode_pub.publish(String(data=mode))
        # 차선 출력 차단을 먼저 알리고 정지한 뒤 상태/단계와 로그를 갱신한다.
        if phase in ('PARKING', 'HALT'):
            self.velocity_pub.publish(Twist())
        if phase == 'PARKING':
            self.step, self.step_started = 0, None
        self.phase = phase
        self.status_pub.publish(String(data=json.dumps({
            'phase': phase, 'step': self.step, 'reason': reason,
        })))
        self.get_logger().info('Parking {}: {}'.format(phase, reason))

    def sign_callback(self, msg):
        if msg.data and self.phase == 'IDLE':
            self.parking_requested = True
            self.start_if_ready()

    def start_if_ready(self):
        if self.phase == 'IDLE' and self.parking_requested and self.lane_enabled:
            self.set_phase('APPROACH', 'follow left half ROI to white stop line')

    def lane_state_callback(self, msg):
        # /lane_state는 외부 허용 입력이다. 주차가 같은 토픽에 False를 재발행하지
        # 않고 maneuver 모드로 차선 출력만 막으므로 외부 정지와 구분할 수 있다.
        self.lane_enabled = msg.data
        if not msg.data and self.phase in ('APPROACH', 'PARKING', 'EXIT'):
            self.halt('external lane stop')
        elif msg.data:
            self.start_if_ready()

    def state_callback(self, msg):
        if msg.data:
            self.sign_callback(msg)
        elif self.phase != 'DONE':
            self.halt('manual cancel')

    def left_callback(self, msg):
        # 진입 전에 본 표지는 기억하지 않고 출차 때 재개한 새 검출만 사용한다.
        if self.phase == 'EXIT' and msg.data:
            self.finish()

    def camera_callback(self, msg):
        if self.phase != 'APPROACH' or not self.lane_enabled:
            return
        try:
            src = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
            if src is None:
                raise ValueError('invalid JPEG')
            h, w = src.shape[:2]
            roi = src[int(h * .9):h, int(w * .2):int(w * .8)]
            white = cv2.inRange(cv2.cvtColor(roi, cv2.COLOR_BGR2HSV),
                                np.array([0, 0, 160]), np.array([180, 40, 255]))
            ratio = cv2.countNonZero(white) / white.size
        except (ValueError, cv2.error, ZeroDivisionError):
            self.halt('invalid camera image')
            return
        if ratio > self.param('white_ratio'):
            self.set_phase('PARKING', 'white stop line detected')

    def finish(self):
        # 한 메시지로 주차 ROI 해제 + 기존 intersection/좌회전 처리를 호출한다.
        # /left가 driving에 먼저 도착해 무시됐어도 인계가 누락되지 않는다.
        self.set_phase('DONE', 'exit left sign; hand off to intersection')

    def halt(self, reason):
        if self.phase == 'HALT':
            return
        self.set_phase('HALT', reason)

    def tick(self):
        now = self.now()
        if now < self.last_tick and self.phase in ('APPROACH', 'PARKING', 'EXIT'):
            self.halt('clock reset; restart parking node')
        self.last_tick = now
        if self.phase != 'PARKING':
            return  # 대기/접근/출차/완료 때 다른 노드의 속도를 덮어쓰지 않는다.
        if self.step_started is None:
            self.step_started = now
        if now - self.step_started >= self.param('durations')[self.step]:
            self.velocity_pub.publish(Twist())
            self.step += 1
            self.step_started = None
            if self.step == 6:
                self.set_phase('EXIT', 'six movements complete; follow left half ROI')
            return  # 동작 사이에 원본처럼 0속도를 한 번 보내고 다음 tick에 진행한다.
        twist = Twist()
        twist.linear.x = self.param('linear_speeds')[self.step]
        sign = 1.0 if self.param('direction') == 'left' else -1.0
        twist.angular.z = sign * self.param('angular_speeds')[self.step]
        self.velocity_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)

    def stop_signal(signum, frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, stop_signal)
    signal.signal(signal.SIGTERM, stop_signal)
    node = ParkingDetection()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok() and node.phase in ('APPROACH', 'PARKING', 'EXIT'):
            node.halt('node shutdown')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
