import cv2
import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data, QoSProfile
from rclpy.duration import Duration

class ConstructionDetection(Node):
    def __init__(self):
        super().__init__('construct_detect_node')
        qos = QoSProfile(depth=10)
        self.scanSubscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.scanSubscriber_callback,
            qos_profile=qos_profile_sensor_data
        )
        self.stateSubscriber = self.create_subscription(
            Bool,
            '/construction_state',
            self.stateSubscriber_callback,
            10
        )
        self.statePublisher = self.create_publisher(
            Bool,
            'construction_state',
            10
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
        
        self.state = False
        self.timer = self.create_timer(0.1, self.moving)
        self.moving_start = False
        self.moving_step = 0
        self.tmp_time = None
    
    def stateSubscriber_callback(self, msg):
        self.state = msg.data
        if self.state == True:
            self.get_logger().info(f'Start Construction Detection')
            msg = Bool()
            msg.data = True
            self.statePublisher.publish(msg)
        else: self.get_logger().info('Terminate Construction Detection')
    
    def scanSubscriber_callback(self, msg):
        if not self.state: return
        ranges = np.array(msg.ranges)
        front_sector = ranges[480:600]
        #self.get_logger().info(f'{len(ranges)}')
        valid_ranges = front_sector[np.isfinite(front_sector)]
        valid_ranges = valid_ranges[valid_ranges > 0.0]
        if len(valid_ranges) == 0:
            self.front_distance_avg = float('inf')
            return
        front_distance_avg = np.mean(valid_ranges)
        #self.get_logger().info(f'Front distance average: {self.front_distance_avg:.2f}')
        if front_distance_avg < 0.35:
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
            elif now - self.tmp_time > Duration(seconds=1.4):
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
            elif now - self.tmp_time > Duration(seconds=1.3):
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
            elif now - self.tmp_time > Duration(seconds=1.3):
                twist.angular.z = 0.0
                self.moving_step = 5
                self.tmp_time = None
            self.velPublisher.publish(twist)

        elif self.moving_step == 5:  # 직진 0.2m
            twist.linear.x = 0.2
            if self.tmp_time is None:
                self.tmp_time = now
            elif now - self.tmp_time > Duration(seconds=1.3):
                twist.linear.x = 0.0
                self.moving_step = 6
                self.tmp_time = None
            self.velPublisher.publish(twist)
        
        elif self.moving_step == 6:   # 왼쪽으로 90도 회전
            twist.angular.z = 1.2
            if self.tmp_time is None:
                self.tmp_time = now
            elif now - self.tmp_time > Duration(seconds=1.3):
                twist.angular.z = 0.0
                self.moving_step = 7
                self.tmp_time = None
            self.velPublisher.publish(twist)
        
        elif self.moving_step == 7:  # 직진 0.3m
            twist.linear.x = 0.2
            if self.tmp_time is None:
                self.tmp_time = now
            elif now - self.tmp_time > Duration(seconds=1.3):
                twist.linear.x = 0.0
                self.moving_step = 8
                self.tmp_time = None
            self.velPublisher.publish(twist)

        else:  # 곡선 주행 (왼쪽 차선으로 이동)
            twist.linear.x = 0.145
            twist.angular.z = 3.14 / 7
            if self.tmp_time is None:
                self.tmp_time = now
            elif now - self.tmp_time > Duration(seconds=4.4):
                twist.linear.x = 0.1
                twist.angular.z = 0.0
                self.moving_step = -1
                self.tmp_time = None
                msg = Bool()
                msg.data = False
                self.statePublisher.publish(msg)
                msg.data = True
                self.laneModePublisher.publish(msg)
                self.laneStatePublisher.publish(msg)
                self.state = False
                self.destroy_node()
                rclpy.shutdown()
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
