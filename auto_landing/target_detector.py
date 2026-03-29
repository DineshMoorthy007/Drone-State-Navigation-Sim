import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np

class TargetDetector(Node):
    def __init__(self):
        super().__init__('target_detector')
        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.state_sub = self.create_subscription(String, '/mission_state', self.state_callback, 10)
        
        self.publisher_ = self.create_publisher(Point, '/landing_target_error', 10)
        self.image_pub = self.create_publisher(Image, '/camera/image_annotated', 10)
        
        self.bridge = CvBridge()
        self.current_state = "INITIALIZING..."
        self.get_logger().info("HUD Vision Node Started.")

    def state_callback(self, msg):
        self.current_state = msg.data

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8").copy()
        except Exception as e:
            return

        h, w, _ = cv_image.shape
        c_x, c_y = w // 2, h // 2

        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        lower_blue = np.array([100,100,20])
        upper_blue = np.array([140,255,255])
        mask = cv2.inRange(hsv_image, lower_blue, upper_blue)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            if cv2.contourArea(largest_contour) > 500:
                M = cv2.moments(largest_contour)
                if M["m00"] != 0:
                    t_x = int(M["m10"] / M["m00"])
                    t_y = int(M["m01"] / M["m00"])
                    e_x = t_x - c_x
                    e_y = t_y - c_y

                    cv2.line(cv_image, (c_x, c_y), (t_x, t_y), (0, 0, 255), 2)
                    cv2.circle(cv_image, (t_x, t_y), 5, (0, 255, 0), -1)

                    error_msg = Point()
                    error_msg.x = float(e_x)
                    error_msg.y = float(e_y)
                    error_msg.z = 0.0
                    self.publisher_.publish(error_msg)

        cv2.rectangle(cv_image, (5, h - 45), (350, h - 5), (0, 0, 0), -1)
        
        text_color = (0, 255, 255) 
        if self.current_state == "LANDING":
            text_color = (0, 0, 255) 
            
        cv2.putText(cv_image, f"MODE: {self.current_state}", (15, h - 15), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, text_color, 2)

        rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

        annotated_msg = self.bridge.cv2_to_imgmsg(rgb_image, encoding="rgb8")
        annotated_msg.header = msg.header

        self.image_pub.publish(annotated_msg)

def main(args=None):
    rclpy.init(args=args)
    node = TargetDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    
