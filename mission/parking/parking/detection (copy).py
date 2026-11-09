import cv2
import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.duration import Duration
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist

DIRECTION = 'left'   # 'left' 또는 'right' 로 변경 가능

class ParkingDetection(Node):
    def __init__(self):
        super().__init__('parking_detect_node')
        
        self.videoSubscriber = self.create_subscription(
            CompressedImage,
            '/camera',
            self.videoSubscriber_callback,
            10
        )
        self.stateSubscriber = self.create_subscription(
            Bool,
            '/parking_state',
            self.stateSubscriber_callback,
            10
        )
        
        self.statePublisher = self.create_publisher(
            Bool,
            'parking_state',
            10
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
        self.laneModePublisher = self.create_publisher(
            Bool,
            '/lane_mode',
            10
        )
        
        self.state = False
        self.parking_start = False
        self.timer = self.create_timer(0.1, self.moving)
        self.moving_step = 0
        self.tmp_time = None
        self.turn_sign = 1.0 if DIRECTION == 'left' else -1.0

    def detect_white_ratio(self, roi):
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        lower_w = np.array([0, 0, 200])
        upper_w = np.array([180, 40, 255])
        mask_w = cv2.inRange(hsv, lower_w, upper_w)
        ratio = cv2.countNonZero(mask_w) / (roi.shape[0] * roi.shape[1])
        return ratio

    def videoSubscriber_callback(self, msg):
        if not self.state: return
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            src = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except Exception as e:
            self.get_logger().error(f'Erorr decoding compressed image: {e}')
            return
        
        h, w = src.shape[:2]
        roi = src[int(h * 0.9):h, int(w * 0.2):int(w * 0.8)]
        white_ratio = self.detect_white_ratio(roi)
        if white_ratio > 0.6:
            msg = Bool()
            msg.data = False
            self.laneStatePublisher.publish(msg)
            self.parking_start = True
            cv2.destroyAllWindows()
        
        if not self.parking_start:
            cv2.putText(src, f'White ratio: {white_ratio*100:.1f}%',
                (10,30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2)
            cv2.rectangle(src, (int(w * 0.2), int(h * 0.9)), (int(w * 0.8), int(h)), (0, 255, 0), 2)
            cv2.imshow('src', src)
            cv2.imshow('ROI', roi)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                cv2.destroyAllWindows()
                self.destroy_node()
                rclpy.shutdown()

    def stateSubscriber_callback(self, msg):
        self.state = msg.data
        if self.state == True:
            self.get_logger().info(f'Start Parking Detection')
            msg = Bool()
            msg.data = True
            self.statePublisher.publish(msg)
        else: self.get_logger().info('Terminate Parking Detection')

    def moving(self):
        if not self.parking_start: return
        twist = Twist()
        now = self.get_clock().now()

        if self.moving_step == 0:  # 후진
            twist.linear.x = -0.2
            if self.tmp_time is None:
                self.tmp_time = now
            elif now - self.tmp_time > Duration(seconds=1.0):
                twist.linear.x = 0.0
                self.moving_step = 1
                self.tmp_time = None
            self.velPublisher.publish(twist)

        elif self.moving_step == 1:  # 방향에 따른 90도 회전
            twist.angular.z = 1.2 * self.turn_sign
            if self.tmp_time is None:
                self.tmp_time = now
            elif now - self.tmp_time > Duration(seconds=1.3):
                twist.angular.z = 0.0
                self.moving_step = 2
                self.tmp_time = None
            self.velPublisher.publish(twist)

        elif self.moving_step == 2:  # 전진
            twist.linear.x = 0.2
            if self.tmp_time is None:
                self.tmp_time = now
            elif now - self.tmp_time > Duration(seconds=1.6):
                twist.linear.x = 0.0
                self.moving_step = 3
                self.tmp_time = None
            self.velPublisher.publish(twist)

        elif self.moving_step == 3:  # 후진
            twist.linear.x = -0.2
            if self.tmp_time is None:
                self.tmp_time = now
            elif now - self.tmp_time > Duration(seconds=1.6):
                twist.linear.x = 0.0
                self.moving_step = 4
                self.tmp_time = None
            self.velPublisher.publish(twist)

        elif self.moving_step == 4:  # 방향에 따른 반대 회전
            twist.angular.z = 1.5 * self.turn_sign
            if self.tmp_time is None:
                self.tmp_time = now
            elif now - self.tmp_time > Duration(seconds=1.3):
                twist.angular.z = 0.0
                self.moving_step = 5
                self.tmp_time = None
            self.velPublisher.publish(twist)

        elif self.moving_step == 5:  # 직진 후 종료
            twist.linear.x = 0.2
            if self.tmp_time is None:
                self.tmp_time = now
            elif now - self.tmp_time > Duration(seconds=1.0):
                twist.linear.x = 0.0
                self.moving_step = -1
                self.tmp_time = None
                msg = Bool()
                msg.data = False
                self.statePublisher.publish(msg)
                msg.data = True
                self.laneModePublisher.publish(msg)
                self.laneStatePublisher.publish(msg)
                self.state = False
                cv2.destroyAllWindows()
                self.get_logger().info(f'Terminate Parking Detection')
                self.destroy_node()
                rclpy.shutdown()
            self.velPublisher.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = ParkingDetection()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

