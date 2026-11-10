import cv2
import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data, DurabilityPolicy, QoSProfile
from rclpy.duration import Duration

class ConstructionDetection(Node):
    def __init__(self):
        super().__init__('construct_detect_node')
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
        self.velPublisher = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )
        self.laneStatePublisher = self.create_publisher(
            Bool,
            '/lane_state',
            10
        )

        self.timer = self.create_timer(0.1, self.moving)
        self.construction_seen = False
        self.moving_start = False
        self.moving_step = 0
        self.tmp_time = None

    def constructionSubscriber_callback(self, msg):
        # 한 번이라도 True가 되면 이후 /construction이 False여도 유지한다.
        if msg.data:
            self.construction_seen = True

    def scanSubscriber_callback(self, msg):
        # 주행이 시작되면 끝날 때까지 다시 판단하지 않는다.
        if not self.construction_seen or self.moving_start: return
        # ranges = np.array(msg.ranges)
        # front_sector = ranges[480:600]
        # valid_ranges = front_sector[np.isfinite(front_sector)]
        # valid_ranges = valid_ranges[valid_ranges > 0.0]
        ranges = np.asarray(msg.ranges)
        angles = msg.angle_min + np.arange(ranges.size) * msg.angle_increment

        # 350° 같은 각도를 -10°로 바꿔 전방 양쪽을 함께 선택한다.
        angles = (angles + np.pi) % (2 * np.pi) - np.pi
        front_indices = np.flatnonzero(np.abs(angles) <= np.deg2rad(20.0))
        front_ranges = ranges[front_indices]

        valid_ranges = front_ranges[
            np.isfinite(front_ranges)
            & (front_ranges > 0.0)
            & (front_ranges >= msg.range_min)
            & (front_ranges <= msg.range_max)
        ]
        
        if len(valid_ranges) == 0:
            return
        front_distance_avg = np.mean(valid_ranges)
        if front_distance_avg < 0.3:
            
            self.get_logger().info(f'avg distance : {front_distance_avg:.2f}')
            msg = Bool()
            msg.data = False
            self.laneStatePublisher.publish(msg)
            self.moving_start = True

    def moving(self):
        if not self.moving_start: return
        twist = Twist()
        now = self.get_clock().now()

        if self.moving_step == 0:   # 왼쪽으로 90도 회전
            twist.angular.z = 1.2
            if self.tmp_time is None:
                self.tmp_time = now
            elif now - self.tmp_time > Duration(seconds=1.45):
                twist.angular.z = 0.0
                self.moving_step = 1
                self.tmp_time = None
            self.velPublisher.publish(twist)

        elif self.moving_step == 1:  # 직진 0.2m
            twist.linear.x = 0.2
            if self.tmp_time is None:
                self.tmp_time = now
            elif now - self.tmp_time > Duration(seconds=1.5):
                twist.linear.x = 0.0
                self.moving_step = 2
                self.tmp_time = None
            self.velPublisher.publish(twist)

        elif self.moving_step == 2:  # 오른쪽으로 90도 회전
            twist.angular.z = -1.2
            if self.tmp_time is None:
                self.tmp_time = now
            elif now - self.tmp_time > Duration(seconds=1.26):#------------------
                twist.angular.z = 0.0
                self.moving_step = 3
                self.tmp_time = None
            self.velPublisher.publish(twist)

        elif self.moving_step == 3:  # 직진 0.5m
            twist.linear.x = 0.2
            if self.tmp_time is None:
                self.tmp_time = now
            elif now - self.tmp_time > Duration(seconds=2.5):
                twist.linear.x = 0.0
                self.moving_step = 4
                self.tmp_time = None
            self.velPublisher.publish(twist)

        elif self.moving_step == 4:  # 오른쪽으로 90도 회전
            twist.angular.z = -1.2
            if self.tmp_time is None:
                self.tmp_time = now
            elif now - self.tmp_time > Duration(seconds=1.3):#-----------------
                twist.angular.z = 0.0
                self.moving_step = 5
                self.tmp_time = None
            self.velPublisher.publish(twist)

        elif self.moving_step == 5:  # 직진 0.2m
            twist.linear.x = 0.2
            if self.tmp_time is None:
                self.tmp_time = now
            elif now - self.tmp_time > Duration(seconds=1.25):#-------------------------
                twist.linear.x = 0.0
                self.moving_step = 6
                self.tmp_time = None
            self.velPublisher.publish(twist)
        
        elif self.moving_step == 6:   # 왼쪽으로 90도 회전
            twist.angular.z = 1.2
            if self.tmp_time is None:
                self.tmp_time = now
            elif now - self.tmp_time > Duration(seconds=1.4):#----------------------
                twist.angular.z = 0.0
                self.moving_step = 7
                self.tmp_time = None
            self.velPublisher.publish(twist)
        
        elif self.moving_step == 7:  # 직진 0.3m
            twist.linear.x = 0.2
            if self.tmp_time is None:
                self.tmp_time = now
            elif now - self.tmp_time > Duration(seconds=1.0):#------------------------
                twist.linear.x = 0.0
                self.moving_step = 8
                self.tmp_time = None
            self.velPublisher.publish(twist)

        else:  # 곡선 주행 (왼쪽 차선으로 이동)
            twist.linear.x = 0.145
            twist.angular.z = 3.14 / 7
            if self.tmp_time is None:
                self.tmp_time = now
            elif now - self.tmp_time > Duration(seconds=3.8):
                twist.linear.x = 0.1
                twist.angular.z = 0.0
                self.moving_step = -1
                self.tmp_time = None
                self.velPublisher.publish(twist)
                msg = Bool()
                msg.data = True
                self.laneStatePublisher.publish(msg)
                self.destroy_node()
                rclpy.shutdown()
                return
            self.velPublisher.publish(twist)
    
def main():
    rclpy.init()
    node = ConstructionDetection()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
