import json
import math

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String

# 상태: 대기 -> 차선 추종 <-> (회전 -> 저속 직진 -> 복귀 회전) -> 종료
# 저속 직진 거리 제한을 넘으면 HALT에서 정지 상태를 유지한다.
WAIT, FOLLOW, TURN_OUT, CREEP, TURN_BACK, DONE, HALT = range(7)
LEFT, RIGHT = 1.0, -1.0


def wrapAngle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class ConstructionDetection(Node):
    def __init__(self):
        super().__init__('construct_detect_node')
        self.declare_parameter('lane_offset', 0.143)        # 따라갈 차선과의 거리 [m]
        self.declare_parameter('obstacle_distance', 0.3)    # 전방 물체 판단 거리 [m]
        self.declare_parameter('min_obstacle_points', 40)   # 물체로 볼 최소 점 개수 (전방 60개 중)
        self.declare_parameter('finish_timeout', 10.0)    # 물체 미감지 종료 시간 [s]
        self.declare_parameter('both_confirm_frames', 3)    # 양쪽 차선 종료 판단 연속 프레임
        self.declare_parameter('last_lane_hold', 3.0)       # 물체 앞에서 차선이 없을 때 직전 차선을 쓸 시간 [s]
        self.declare_parameter('lane_info_timeout', 0.5)    # 이보다 오래된 /lane_info는 미검출 [s]
        self.declare_parameter('follow_speed', 0.1)         # 차선 추종 선속도 [m/s]
        self.declare_parameter('lost_speed', 0.05)          # 차선 미검출 직진 선속도 [m/s]
        self.declare_parameter('creep_speed', 0.05)         # 회피 저속 직진 선속도 [m/s]
        self.declare_parameter('creep_stop_distance', 0.1)  # 목표 차선이 이 거리 안에 오면 직진 종료 [m]
        self.declare_parameter('creep_max_distance', 0.6)   # 차선 없이 저속 직진할 최대 거리 [m]
        self.declare_parameter('kp', 3.0)                 # 차선 offset P 게인
        self.declare_parameter('kh', 1.5)                   # 차선 각도 게인
        self.declare_parameter('max_angular', 1.0)          # 차선 추종 최대 각속도 [rad/s]
        self.declare_parameter('turn_speed', 0.8)           # 제자리 회전 최대 각속도 [rad/s]
        self.declare_parameter('turn_min_speed', 0.2)       # 제자리 회전 최소 각속도 [rad/s]
        self.declare_parameter('turn_tolerance', 0.02)      # 회전 완료 허용 오차 [rad]

        # mission_model은 표지판 토픽을 latched(transient local)로 발행하므로 같은 QoS로 받는다.
        sign_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.constructionSubscriber = self.create_subscription(
            Bool,
            '/construction',
            self.constructionSubscriber_callback,
            sign_qos
        )
        self.scanSubscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.scanSubscriber_callback,
            qos_profile=qos_profile_sensor_data
        )
        self.laneInfoSubscriber = self.create_subscription(
            String,
            '/lane_info',
            self.laneInfoSubscriber_callback,
            10
        )
        self.odomSubscriber = self.create_subscription(
            Odometry,
            '/odom',
            self.odomSubscriber_callback,
            qos_profile=qos_profile_sensor_data
        )
        self.velPublisher = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )
        self.laneModePublisher = self.create_publisher(
            Bool,
            '/lane_mode',
            10
        )
        self.laneStatePublisher = self.create_publisher(
            Bool,
            '/lane_state',
            10
        )

        self.state = WAIT
        self.front_distance = None
        self.yellow = None
        self.white = None
        self.lane_time = None
        self.last_lane = None           # 마지막으로 차선을 본 (yellow, white, 시각)
        self.both_frames = 0
        self.yaw = None
        self.position = None
        self.creep_start = None
        self.creep_target = None        # 목표 차선 앞에서 멈출 누적 직진 거리 [m]
        self.turn_target = None
        self.turn_direction = LEFT      # 첫 회전 방향
        self.creep_line = 'yellow'      # 저속 직진을 멈출 기준 차선
        self.avoid_count = 0
        self.last_obstacle_time = None
        self.lane_error = None          # 현재 따라가는 차선의 offset 오차 [m]
        self.lane_heading = None        # 현재 따라가는 차선의 로봇 기준 방향 [rad]
        self.timer = self.create_timer(0.05, self.moving)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def param(self, name):
        return self.get_parameter(name).value

    def publishLaneState(self, data):
        msg = Bool()
        msg.data = data
        self.laneStatePublisher.publish(msg)

    def publishVelocity(self, linear=0.0, angular=0.0):
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self.velPublisher.publish(twist)

    def constructionSubscriber_callback(self, msg):
        # 한 번 시작하면 이후 /construction이 False여도 끝날 때까지 유지한다.
        if not msg.data or self.state != WAIT:
            return
        self.get_logger().info('Construction start: lane_state False')
        self.publishLaneState(False)
        self.state = FOLLOW

    def scanSubscriber_callback(self, msg):
        ranges = np.array(msg.ranges)
        front_sector = ranges[510:570]   # 정면 ±10도
        valid_ranges = front_sector[np.isfinite(front_sector)]
        valid_ranges = valid_ranges[valid_ranges > 0.0]
        # 거리 안에 들어온 점이 충분할 때만 물체로 본다. 점 개수가 물체의 폭에 해당한다.
        close_ranges = valid_ranges[valid_ranges < self.param('obstacle_distance')]
        enough = len(close_ranges) >= self.param('min_obstacle_points')
        self.front_distance = float(np.mean(close_ranges)) if enough else None

    def laneInfoSubscriber_callback(self, msg):
        info = json.loads(msg.data)
        self.yellow = info['yellow']
        self.white = info['white']
        self.yellow_heading = info['yellow_heading']
        self.white_heading = info['white_heading']
        self.lane_time = self.now()
        if self.yellow is not None or self.white is not None:
            self.last_lane = (self.yellow, self.white, self.lane_time)
        self.both_frames = self.both_frames + 1 if self.yellow is not None and self.white is not None else 0
        self.updateLaneError()

    def odomSubscriber_callback(self, msg):
        self.position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        q = msg.pose.pose.orientation
        self.yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def visibleLanes(self):
        '''(yellow, white). 오래된 /lane_info는 두 차선 모두 미검출로 본다.'''
        if self.lane_time is None or self.now() - self.lane_time > self.param('lane_info_timeout'):
            return None, None
        return self.yellow, self.white

    def updateLaneError(self):
        '''양쪽이 보이면 흰선, 한쪽만 보이면 보이는 차선 기준 offset 오차와 각도를 갱신한다.'''
        offset = self.param('lane_offset')
        if self.white is not None:
            self.lane_error, self.lane_heading = self.white + offset, self.white_heading    # 흰선은 오른쪽(-)
        elif self.yellow is not None:
            self.lane_error, self.lane_heading = self.yellow - offset, self.yellow_heading  # 노란선은 왼쪽(+)
        else:
            self.lane_error = self.lane_heading = None

    def followLane(self, yellow, white):
        if yellow is None and white is None:
            self.publishVelocity(self.param('lost_speed'), 0.0)
            return
        max_angular = self.param('max_angular')
        angular = self.param('kp') * self.lane_error + self.param('kh') * self.lane_heading
        self.publishVelocity(self.param('follow_speed'), float(np.clip(angular, -max_angular, max_angular)))

    def startAvoid(self, yellow, white):
        '''전방 물체를 만났을 때 보이는 차선으로 회피 방향을 정한다. 차선이 없으면 False.'''
        # 물체에 가려 차선이 사라졌으면 직전에 본 차선으로 판단한다.
        if yellow is None and white is None and self.last_lane is not None \
                and self.now() - self.last_lane[2] <= self.param('last_lane_hold'):
            yellow, white = self.last_lane[:2]
        if white is not None and (yellow is None or abs(white) < abs(yellow)):
            self.turn_direction, self.creep_line = LEFT, 'yellow'
        elif yellow is not None:
            self.turn_direction, self.creep_line = RIGHT, 'white'
        else:
            return False
        if not self.startTurn(self.turn_direction):
            return False
        self.get_logger().info(
            f'Avoid {self.avoid_count + 1}: turn {"left" if self.turn_direction == LEFT else "right"}, '
            f'creep until {self.creep_line}')
        self.state = TURN_OUT
        return True

    def startTurn(self, direction):
        if self.yaw is None:
            return False
        self.turn_target = wrapAngle(self.yaw + direction * math.pi / 2.0)
        return True

    def turnInPlace(self):
        '''목표 각도까지 제자리 회전한다. 도달하면 True.'''
        error = wrapAngle(self.turn_target - self.yaw)
        if abs(error) <= self.param('turn_tolerance'):
            self.publishVelocity()
            return True
        speed = min(self.param('turn_speed'), max(self.param('turn_min_speed'), 2.0 * abs(error)))
        self.publishVelocity(0.0, math.copysign(speed, error))
        return False

    def finish(self, reason):
        self.get_logger().info(f'Construction end ({reason}): lane_state True')
        self.publishVelocity()
        self.publishLaneState(True)
        self.state = DONE

    def moving(self):
        if self.state in (WAIT, DONE, HALT):
            return
        yellow, white = self.visibleLanes()
        obstacle = self.front_distance is not None and self.front_distance < self.param('obstacle_distance')

        if self.state == FOLLOW:
            if obstacle:
                self.get_logger().info(f'avg distance : {self.front_distance:.2f}')
                if not self.startAvoid(yellow, white):
                    self.publishVelocity(self.param('lost_speed'), 0.0)     # 판단할 차선이 보일 때까지 저속 직진
                return
            # 종료 조건은 첫 회피 이후부터 본다. 그 전에는 양쪽 차선이 보여도 흰선을 따라간다.
            if self.avoid_count > 0:
                if self.both_frames >= self.param('both_confirm_frames'):
                    self.finish('both lanes')
                    return
                if self.now() - self.last_obstacle_time >= self.param('finish_timeout'):
                    self.finish('no obstacle')
                    return
            self.followLane(yellow, white)

        elif self.state == TURN_OUT:
            if self.turnInPlace():
                self.creep_start = self.position
                self.creep_target = None
                self.state = CREEP

        elif self.state == CREEP:
            line = yellow if self.creep_line == 'yellow' else white
            stop_distance = self.param('creep_stop_distance')
            traveled = math.dist(self.position, self.creep_start)
            # 차선을 정면으로 보면 검출이 끊기므로, 보일 때 계산한 남은 거리를 odom으로 채운다.
            if line is not None:
                self.creep_target = traveled + max(abs(line) - stop_distance, 0.0)
                reached = abs(line) <= stop_distance
            else:
                reached = self.creep_target is not None and traveled >= self.creep_target
            if not reached:
                if traveled >= self.param('creep_max_distance'):
                    self.get_logger().warning(f'{self.creep_line} lane not found while creeping: stop')
                    self.publishVelocity()
                    self.state = HALT
                    return
                self.publishVelocity(self.param('creep_speed'), 0.0)
                return
            self.publishVelocity()
            if self.startTurn(-self.turn_direction):
                self.state = TURN_BACK

        elif self.state == TURN_BACK:
            if self.turnInPlace():
                self.avoid_count += 1
                self.last_obstacle_time = self.now()
                self.both_frames = 0
                self.state = FOLLOW


def main():
    rclpy.init()
    node = ConstructionDetection()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
