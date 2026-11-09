import cv2
import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data, QoSProfile

class LidarTest(Node):
    def __init__(self):
        super().__init__('lidar_test_node')
        qos = QoSProfile(depth=10)
        self.scanSubscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.scanSubscriber_callback,
            qos_profile=qos_profile_sensor_data
        )
    
    def scanSubscriber_callback(self, msg):
        img_size = 200
        scale = 100  # Scale for converting meters to pixels (1m = 100px)
        center = img_size // 2
        image = np.zeros((img_size, img_size, 3), dtype=np.uint8)
        
        angle = msg.angle_min
        for r in msg.ranges:
            if r > 1.0: continue
            if np.isinf(r) or np.isnan(r):
                angle += msg.angle_increment
                continue
            
            x = r * np.cos(270 - angle)  # forward-backward (0° is forward)
            y = r * np.sin(270 - angle)  # left-right
            px = int(center + y * scale)  # horizontal axis
            py = int(center - x * scale)  # vertical axis (invert to make 0° point up)

            if 0 <= px < img_size and 0 <= py < img_size:
                if angle in range(0, 10):
                    cv2.circle(image, (px, py), 2, (255, 0, 255), -1)
                elif angle in range(350, 360):
                    cv2.circle(image, (px, py), 2, (150, 0, 150), -1)
                else:
                    cv2.circle(image, (px, py), 2, (0, 255, 0), -1)
            angle += msg.angle_increment
            
        cv2.circle(image, (100,100), 3, (255,0,0), 2)
        cv2.circle(image, (center,center), 5, (0,0,255), -1)
        cv2.imshow("LIDAR Scan", image)
        cv2.waitKey(1)

def main():
    rclpy.init()
    node = LidarTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
