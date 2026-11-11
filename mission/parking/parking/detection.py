"""P sign -> left half ROI -> white line -> six movements -> left sign handoff."""
import json
import math
import signal

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool, String


class ParkingDetection(Node):
    def __init__(self):
        super().__init__('parking_detect_node')
        # 기본 순서: 0 후진, 1 회전1, 2 전진, 3 후진, 4 회전2, 5 전진.
        # 직선 단계의 전진/후진은 linear_speeds의 부호로 바꿀 수 있다.
        # 거리/각도는 양수 크기, 이동 방향은 linear_speeds와 direction으로 지정한다.
        # 사용자는 config/parking_sim.yaml의 숫자를 수정하고 노드를 재시작한다.
        defaults = {
            'direction': 'left', 'white_ratio': 0.04,
            'distances': [0.20, 0.0, 0.32, 0.32, 0.0, 0.20],  # m
            'angles_deg': [0.0, 89.4, 0.0, 0.0, 77.3, 0.0],
            'linear_speeds': [-0.2, 0.0, 0.2, -0.2, 0.0, 0.2],
            'angular_speeds': [0.0, 1.2, 0.0, 0.0, 1.5, 0.0],
            'distance_tolerance': 0.01, 'angle_tolerance_deg': 1.0,
            'distance_kp': 1.0, 'angle_kp': 1.5, 'heading_kp': 2.0,
            'min_linear_speed': 0.03, 'min_angular_speed': 0.10,
            'max_heading_angular': 0.4,
            'stop_linear_tolerance': 0.01, 'stop_angular_tolerance': 0.03,
            'odom_timeout': 0.5, 'stop_timeout': 3.0,
            # 시간은 실패 감시용이다. 시간이 지나도 다음 단계로 넘어가지 않는다.
            'step_timeouts': [10.0, 10.0, 12.0, 12.0, 10.0, 10.0],
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value, ParameterDescriptor(read_only=True))
        if self.param('direction') not in ('left', 'right'):
            raise ValueError('direction must be left or right')
        if not 0 < self.param('white_ratio') <= 1:
            raise ValueError('white_ratio must be in (0, 1]')
        for name in ('distances', 'angles_deg', 'linear_speeds', 'angular_speeds',
                     'step_timeouts'):
            values = self.param(name)
            if len(values) != 6 or not all(math.isfinite(v) for v in values):
                raise ValueError(name + ' must contain six finite values')
        for name in ('distance_tolerance', 'angle_tolerance_deg', 'distance_kp',
                     'angle_kp', 'min_linear_speed', 'min_angular_speed',
                     'max_heading_angular', 'stop_linear_tolerance',
                     'stop_angular_tolerance', 'odom_timeout', 'stop_timeout'):
            if not math.isfinite(self.param(name)) or self.param(name) <= 0:
                raise ValueError(name + ' must be finite and positive')
        if not math.isfinite(self.param('heading_kp')) or self.param('heading_kp') < 0:
            raise ValueError('heading_kp must be finite and non-negative')
        if min(self.param('step_timeouts')) <= 0:
            raise ValueError('step_timeouts must be positive')
        for i in range(6):
            distance, angle = self.param('distances')[i], self.param('angles_deg')[i]
            v, w = self.param('linear_speeds')[i], self.param('angular_speeds')[i]
            if i in (0, 2, 3, 5):
                if (distance <= self.param('distance_tolerance') or angle != 0 or w != 0
                        or abs(v) < self.param('min_linear_speed')):
                    raise ValueError('Invalid straight movement settings at step ' + str(i))
            elif (distance != 0 or angle <= self.param('angle_tolerance_deg') or v != 0
                  or w < self.param('min_angular_speed')):
                raise ValueError('Invalid turn settings at step ' + str(i))

        self.phase = 'IDLE'  # IDLE -> APPROACH -> PARKING -> EXIT -> DONE; cancel -> HALT
        self.parking_requested = False
        self.lane_enabled = False
        self.step = 0
        self.step_started = None
        self.step_origin = None
        self.settling = False
        self.stop_started = None
        self.stop_odom_stamp = None
        self.position = None
        self.yaw = None
        self.unwrapped_yaw = 0.0
        self.odom_stamp = None
        self.odom_time = None
        self.odom_frames = None
        self.linear_speed = self.angular_speed = None
        self.last_tick = self.now()
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.velocity_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.mode_pub = self.create_publisher(String, '/parking/lane_mode', latched)
        self.status_pub = self.create_publisher(String, '/parking/status', latched)
        self.create_subscription(Bool, '/parking', self.sign_callback, latched)
        self.create_subscription(Bool, '/left', self.left_callback, latched)
        self.create_subscription(Bool, '/parking_state', self.state_callback, 10)
        self.create_subscription(Bool, '/lane_state', self.lane_state_callback, 10)
        self.create_subscription(CompressedImage, '/bev', self.camera_callback,
                                 qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self.odom_callback,
                                 qos_profile_sensor_data)
        self.set_phase('IDLE', 'ready')
        self.create_timer(0.05, self.tick)

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
            self.step_origin = None
            self.settling = True
            self.stop_started = self.now()
            self.stop_odom_stamp = self.get_clock().now().nanoseconds
        self.phase = phase
        self.publish_status(reason)
        self.get_logger().info('Parking {}: {}'.format(phase, reason))

    def publish_status(self, reason):
        self.status_pub.publish(String(data=json.dumps({
            'phase': self.phase, 'step': self.step, 'reason': reason,
        })))
        self.get_logger().info(f'Parking phase={self.phase}, step={self.step}: {reason}')

    def odom_callback(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        v, w = msg.twist.twist.linear, msg.twist.twist.angular
        values = (p.x, p.y, q.x, q.y, q.z, q.w, v.x, v.y, w.z)
        norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
        if (not all(math.isfinite(value) for value in values)
                or not math.isfinite(norm) or norm < 1e-6):
            self.odom_time = None
            if self.phase == 'PARKING':
                self.halt('invalid odometry')
            return
        stamp = msg.header.stamp.sec * 1000000000 + msg.header.stamp.nanosec
        frames = (msg.header.frame_id, msg.child_frame_id)
        reset = (self.odom_stamp is not None and stamp < self.odom_stamp
                 or self.odom_frames is not None and frames != self.odom_frames)
        if reset and self.phase == 'PARKING':
            self.halt('odometry clock/frame reset')
            return
        # 같은 샘플로 수신 시각이나 정지 확인을 갱신하지 않는다.
        if not reset and stamp == self.odom_stamp:
            return
        x, y, z, qw = q.x/norm, q.y/norm, q.z/norm, q.w/norm
        yaw = math.atan2(2.0 * (qw*z + x*y), 1.0 - 2.0 * (y*y + z*z))
        if self.yaw is None or reset:
            self.unwrapped_yaw = yaw
        else:
            delta = yaw - self.yaw
            self.unwrapped_yaw += math.atan2(math.sin(delta), math.cos(delta))
        self.position, self.yaw = (p.x, p.y), yaw
        self.linear_speed, self.angular_speed = math.hypot(v.x, v.y), abs(w.z)
        self.odom_stamp, self.odom_time, self.odom_frames = stamp, self.now(), frames

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
            # roi = src[int(h * .9):h, int(w * .2):int(w * .8)]
            roi = src[:, :285]

            # cv2.imshow("bev", src)
            # cv2.waitKey(1)
            
            white = cv2.inRange(cv2.cvtColor(roi, cv2.COLOR_BGR2HSV),
                                np.array([0, 0, 160]), np.array([180, 40, 255]))
            ratio = cv2.countNonZero(white) / white.size
        except (ValueError, cv2.error, ZeroDivisionError):
            self.halt('invalid camera image')
            return
        if ratio > self.param('white_ratio'):
            self.get_logger().info(f"parking!parking!parking!parking!parking! white: {ratio}")
            self.set_phase('PARKING', 'white stop line detected')
        else:
            self.get_logger().info(f'white: {ratio}')


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
        if (self.odom_time is None or now - self.odom_time > self.param('odom_timeout')
                or abs(now - self.odom_stamp * 1e-9) > self.param('odom_timeout')):
            self.halt('odometry missing or stale')
            return
        if (self.step_started is not None
                and now - self.step_started >= self.param('step_timeouts')[self.step]):
            self.halt('step {} timed out'.format(self.step))
            return

        if self.settling:
            # 첫 후진 전과 모든 동작 사이에 실제 정지를 확인한다.
            self.velocity_pub.publish(Twist())
            if now - self.stop_started >= self.param('stop_timeout'):
                self.halt('stop confirmation timed out')
                return
            if (self.stop_odom_stamp is not None and self.odom_stamp <= self.stop_odom_stamp
                    or self.linear_speed > self.param('stop_linear_tolerance')
                    or self.angular_speed > self.param('stop_angular_tolerance')):
                return
            if self.step_origin is not None:
                self.step += 1
                if self.step == 6:
                    self.set_phase('EXIT', 'six odometry movements complete; follow left half ROI')
                    return
            self.step_origin = (*self.position, self.unwrapped_yaw)
            self.step_started = now
            self.settling = False
            self.publish_status('odometry step {} started'.format(self.step))
            return

        x0, y0, yaw0 = self.step_origin
        twist = Twist()
        if self.step in (0, 2, 3, 5):
            speed = self.param('linear_speeds')[self.step]
            sign = math.copysign(1.0, speed)
            dx, dy = self.position[0] - x0, self.position[1] - y0
            progress = sign * (dx * math.cos(yaw0) + dy * math.sin(yaw0))
            remaining = self.param('distances')[self.step] - progress
            reached = remaining <= self.param('distance_tolerance')
            twist.linear.x = sign * min(abs(speed), max(self.param('min_linear_speed'),
                                                       self.param('distance_kp') * remaining))
            heading_error = yaw0 - self.unwrapped_yaw
            heading_error = math.atan2(math.sin(heading_error), math.cos(heading_error))
            limit = self.param('max_heading_angular')
            twist.angular.z = max(-limit, min(limit, self.param('heading_kp') * heading_error))
        else:
            sign = 1.0 if self.param('direction') == 'left' else -1.0
            progress = sign * (self.unwrapped_yaw - yaw0)
            remaining = math.radians(self.param('angles_deg')[self.step]) - progress
            reached = remaining <= math.radians(self.param('angle_tolerance_deg'))
            twist.angular.z = sign * min(self.param('angular_speeds')[self.step],
                                         max(self.param('min_angular_speed'),
                                             self.param('angle_kp') * remaining))
        if reached:
            self.settling = True
            self.velocity_pub.publish(Twist())
            self.stop_started = now
            self.stop_odom_stamp = self.get_clock().now().nanoseconds
            self.publish_status('step {} reached target; waiting for stop'.format(self.step))
            return
        self.velocity_pub.publish(twist)


def main(args=None):
    # Galactic has no signal_handler_options; install our handlers after init
    # so shutdown can publish the stop command before closing the ROS context.
    rclpy.init(args=args)

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
