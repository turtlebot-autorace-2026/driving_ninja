import cv2
import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage

class LevelCrossingDetection(Node):
    def __init__(self):
        super().__init__('level_crossing_detection_node')
        self.videoSubscriber = self.create_subscription(
            CompressedImage,
            '/camera2',
            self.videoSubscriber_callback,
            10
        )
        self.stateSubscriber = self.create_subscription(
            Bool,
            '/level_crossing_state',
            self.stateSubscriber_callback,
            10
        )
        self.statePublisher = self.create_publisher(
            Bool,
            'level_crossing_state',
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
        self.velPublisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        
        self.state = False
        self.first = True
        self.bar_closed = False
        
        self.roi_y1, self.roi_y2 = 250, 320
        self.roi_x1, self.roi_x2 = 0, 440
        self.close_threshold = 0.30
        self.open_threshold = 0.02
    
    def stateSubscriber_callback(self, msg):
        self.state = msg.data
        if self.state == True:
            if self.first:
                print('\nStart Level Crossing Detection')
                self.first = False
            msg = Bool()
            msg.data = True
            self.statePublisher.publish(msg)
        else: print('Terminate Level Crossing Detection\n')

    def gaussianBlur(self, src):
        gaussian_src = cv2.GaussianBlur(src, (9,9), sigmaX=0, sigmaY=0)
        return gaussian_src
    
    def redHsvInrange(self, src):
        red_lower_bound = np.array([0,100,70], dtype=np.uint8)
        red_upper_bound = np.array([15,255,170], dtype=np.uint8)
        red_lower_bound2 = np.array([170,100,70], dtype=np.uint8)
        red_upper_bound2 = np.array([179,255,170], dtype=np.uint8)
        hsv_src = cv2.cvtColor(src, cv2.COLOR_BGR2HSV)
        hsv_dst = cv2.inRange(hsv_src, red_lower_bound, red_upper_bound)
        hsv_dst2 = cv2.inRange(hsv_src, red_lower_bound2, red_upper_bound2)
        return hsv_dst | hsv_dst2
    
    def componentsWithStatsFilter(self, src):
        min_area = 400
        num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(src, connectivity=8)
        valid_labels = np.where(stats[1:, cv2.CC_STAT_AREA] >= min_area)[0] + 1
        mask = np.isin(labels, valid_labels)
        filtered = (mask * 255).astype(np.uint8)
        return filtered
    
    def stop(self):
        twist = Twist()
        self.velPublisher.publish(twist)
    
    def videoSubscriber_callback(self, msg):
        if not self.state: return
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            src = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            roi = src[self.roi_y1:self.roi_y2, self.roi_x1:self.roi_x2]
            roi = self.gaussianBlur(roi)
            roi = self.redHsvInrange(roi)
            roi = self.componentsWithStatsFilter(roi)
            red_ratio = np.count_nonzero(roi)/(roi.shape[0]*roi.shape[1])
        except Exception as e:
            self.get_logger().error(f'Error decoding compressed image: {e}')
            return
        
        if not self.bar_closed and red_ratio > self.close_threshold:
            self.bar_closed = True
            self.get_logger().info(f'Bar Closed - ratio={red_ratio:.3f}')
            msg = Bool()
            msg.data = False
            self.laneStatePublisher.publish(msg)
            self.stop()
        
        elif self.bar_closed and red_ratio < self.open_threshold:
            self.get_logger().info(f'Bar Opened - ratio={red_ratio:.3f}')
            print('Terminate Level Crossing Detection\n')
            msg = Bool()
            msg.data = False
            self.statePublisher.publish(msg)
            msg.data = True
            self.laneStatePublisher.publish(msg)
            cv2.destroyAllWindows()
            self.destroy_node()
            rclpy.shutdown()
        
        else: self.get_logger().info(f'Bar Waiting - ratio={red_ratio:.3f}')
        
        cv2.rectangle(src, (self.roi_x1, self.roi_y1), (self.roi_x2, self.roi_y2), (0,255,0), 2)
        cv2.imshow('src', src)
        cv2.imshow('roi', roi)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            cv2.destroyAllWindows()
            self.destroy_node()
            rclpy.shutdown()

def main():
    rclpy.init()
    node = LevelCrossingDetection()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
