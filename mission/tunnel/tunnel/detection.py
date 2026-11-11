import cv2
import time
import rclpy
import threading
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Bool
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Twist
from rclpy.qos import QoSProfile, DurabilityPolicy


INITIAL_POSE = [-0.486, -0.696, -0.081, 0.997]
GOAL_POSE = [0.745, 1.012, 0.734, 0.679]

class TunnelDetection(Node):
    def __init__(self):
        super().__init__('tunnel_detection_node')
        # self.videoSubscriber = self.create_subscription(
        #     CompressedImage,
        #     '/camera',
        #     self.videoSubscriber_callback,
        #     10
        # )

        tunnel_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )



        self.tunnelSubscriber = self.create_subscription(
            Bool,
            '/tunnel',
            self.tunnelStart,
            tunnel_qos
        )
        self.stateSubscriber = self.create_subscription(
            Bool,
            '/tunnel_state',
            self.stateSubscriber_callback,
            10
        )
        self.statePublisher = self.create_publisher(
            Bool,
            'tunnel_state',
            10
        )
        self.laneModePublisher = self.create_publisher(
            Bool,
            'lane_mode',
            10
        )
        self.laneStatePublisher = self.create_publisher(
            Bool,
            'lane_state',
            10
        )
        self.velPublisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        self.initPosePublisher = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            10
        )
        
        self.state = True
        self.first = True
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
    def stateSubscriber_callback(self, msg):
        self.state = msg.data
        if self.state == True:
            if self.first:
                print('\nStart Intersection Detection')
                self.first = False
            msg = Bool()
            msg.data = True
            self.statePublisher.publish(msg)
        else: print('Terminate Intersection Detection\n')
        
    def gaussianBlur(self, src):
        gaussian_src = cv2.GaussianBlur(src, (9,9), sigmaX=0, sigmaY=0)
        return gaussian_src
    
    def morphology(self, src):
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (9,9))
        src = cv2.dilate(src, kernel)
        return src
    
    def detectSign(self, src):
        src = self.gaussianBlur(src[:120,:])
        hsv = cv2.cvtColor(src, cv2.COLOR_BGR2HSV)
        lower_bound = np.array([0, 120, 70], dtype=np.uint8)
        upper_bound = np.array([40, 255, 200], dtype=np.uint8)
        lower_bound2 = np.array([165, 120, 70], dtype=np.uint8)
        upper_bound2 = np.array([179, 255, 200], dtype=np.uint8)
        dst = cv2.inRange(hsv, lower_bound, upper_bound)
        dst2 = cv2.inRange(hsv, lower_bound2, upper_bound2)
        src = self.morphology(dst | dst2)
        if np.count_nonzero(src) > 1200: dst = True
        #print(np.count_nonzero(src))
        else: dst = False
        return src, dst
    
    def publish_initial_pose(self):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = INITIAL_POSE[0]
        msg.pose.pose.position.y = INITIAL_POSE[1]
        msg.pose.pose.orientation.z = INITIAL_POSE[2]
        msg.pose.pose.orientation.w = INITIAL_POSE[3]
        msg.pose.covariance = [0.0]*36
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.1
        self.initPosePublisher.publish(msg)
        self.get_logger().info('Initialized Navigation start Position')
    
    def publish_goal_pose(self):
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn('Could not find NavigateToPose Server')
            return
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = GOAL_POSE[0]
        pose.pose.position.y = GOAL_POSE[1]
        pose.pose.orientation.z = GOAL_POSE[2]
        pose.pose.orientation.w = GOAL_POSE[3]
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        self.goal_sent = True
    
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.publish_goal_pose()
        self.get_logger().info('Goal Accepted...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
    
    def feedback_callback(self, feedback_msg):
        pass
    
    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        if status == 4:
            self.get_logger().info('Arrived!!')
            print('Terminate Intersection Detection\n')
            msg = Bool()
            msg.data = True
            self.laneStatePublisher.publish(msg)
            msg.data = False
            self.statePublisher.publish(msg)
            self.destroy_node()
            rclpy.shutdown()
        else:
            self.get_logger().warn(f'Failed for arriving')
            self.publish_goal_pose()
    
    def front_moving(self):
        twist = Twist()
        twist.linear.x = 0.2
        duration = 2.8
        end_time = time.time() + duration
        while time.time() < end_time:
            self.velPublisher.publish(twist)
            time.sleep(0.1)
        twist = Twist()
        self.velPublisher.publish(twist)
        self.get_logger().info(f'Finished moving')
        cv2.destroyAllWindows()
        threading.Timer(2.0, self.publish_initial_pose).start()
        threading.Timer(4.0, self.publish_goal_pose).start()

    def tunnelStart(self, msg):
        if not self.state: return
        if not msg.data: return
        self.get_logger().info(f'Sign Detected')
        def function():
            msg = Bool()
            msg.data = False
            self.laneStatePublisher.publish(msg)
            self.front_moving()
            self.state = False
        threading.Timer(2.0, function).start()

    
    def videoSubscriber_callback(self, msg):
        if not self.state: return
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            src = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            src2, sign_detected = self.detectSign(src)
        except Exception as e:
            self.get_logger().error(f'Error decoding compressed image: {e}')
            return
        
        if sign_detected:
            self.get_logger().info(f'Sign Detected')
            def function():
                msg = Bool()
                msg.data = False
                self.laneStatePublisher.publish(msg)
                self.front_moving()
                self.state = False
            threading.Timer(2.0, function).start()
        
        cv2.imshow('src', src)
        cv2.imshow('src2', src2)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            cv2.destroyAllWindows()
            self.destroy_node()
            rclpy.shutdown()

def main():
    rclpy.init()
    node = TunnelDetection()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
        
