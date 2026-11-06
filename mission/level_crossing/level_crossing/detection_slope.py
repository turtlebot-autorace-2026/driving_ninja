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
        self.first_try = True
    
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
        min_area = 600
        num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(src, connectivity=8)
        valid_labels = np.where(stats[1:, cv2.CC_STAT_AREA] >= min_area)[0] + 1
        mask = np.isin(labels, valid_labels)
        filtered = (mask * 255).astype(np.uint8)
        return filtered
    
    def find_moments(self, src):
        centers = []
        contours, _ = cv2.findContours(src, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)
        for contour in contours:
            if cv2.contourArea(contour) < 800: continue
            m = cv2.moments(contour)
            cx = int(m['m10']/m['m00'])
            cy = int(m['m01']/m['m00'])
            centers.append((cx, cy))
        if len(contours) < 2: return []
        else: return centers
    
    def stop(self):
        twist = Twist()
        self.velPublisher.publish(twist)
    
    def check_slope(self, moments):
        slope = []
        inf_count = 0
        if len(moments) >= 2:
            for i in range(len(moments) - 1):
                dx = moments[i][0] - moments[i+1][0]
                dy = moments[i][1] - moments[i+1][1]
                if dx == 0: inf_count += 1
                else: slope.append(abs(dy/dx))
            if inf_count > 0: return False
            if slope:
                slope_avg = sum(slope)/len(slope)
                if slope_avg > 0.7: return False
                elif slope_avg < 0.4: return True
        return# True
    
    def videoSubscriber_callback(self, msg):
        if not self.state: return
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            src = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            src2 = self.gaussianBlur(src)
            src2 = self.redHsvInrange(src2)
            src2 = self.componentsWithStatsFilter(src2)
        except Exception as e:
            self.get_logger().error(f'Error decoding compressed image: {e}')
            return
        
        moments = self.find_moments(src2)
        is_closed = self.check_slope(moments)
        if len(moments) > 1 and is_closed:
            self.get_logger().info('Bar Closed')
            msg = Bool()
            msg.data = False
            self.laneStatePublisher.publish(msg)
            self.stop()
            
        elif len(moments) > 1 and not is_closed:
            self.get_logger().info('Bar Opened')
            print('Terminate Level Crossing Detection\n')
            msg = Bool()
            msg.data = False
            self.statePublisher.publish(msg)
            msg.data = True
            self.laneStatePublisher.publish(msg)
            cv2.destroyAllWindows()
            self.destroy_node()
            rclpy.shutdown()
        
        else: self.get_logger().info('Bar Not Detected')
            
        cv2.imshow('src', src)
        cv2.imshow('src2', src2)
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
