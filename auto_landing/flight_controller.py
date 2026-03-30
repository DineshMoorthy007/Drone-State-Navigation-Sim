import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from rclpy.parameter import Parameter
import math

class FlightController(Node):
    def __init__(self):
        super().__init__('flight_controller')
        self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.state_sub = self.create_subscription(String, '/mission_state', self.state_callback, 10)
        self.error_sub = self.create_subscription(Point, '/landing_target_error', self.error_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.current_state = "SEARCHING"
        self.error_x = 0.0
        self.error_y = 0.0
        self.time_entered_search = 0.0
        
        self.escaping_wall_until = 0.0 
        
        self.search_target_x = 0.0
        self.search_target_y = 0.0
        self.search_phase = "SWEEP_Y"
        
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 3.0
        self.current_yaw = 0.0

        self.timer = self.create_timer(0.1, self.control_loop)

    def get_time(self):
        return self.get_clock().now().nanoseconds / 1e9

    def state_callback(self, msg):
        if self.current_state != msg.data:
            self.current_state = msg.data
            if self.current_state == "SEARCHING":
                self.time_entered_search = self.get_time()

    def error_callback(self, msg):
        self.error_x = msg.x
        self.error_y = msg.y

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_z = msg.pose.pose.position.z
        q = msg.pose.pose.orientation
        self.current_yaw = math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))

    def control_loop(self):
        current_time = self.get_time()
        if current_time <= 0.0:
            return 

        twist = Twist()
        BOUNDARY = 4.5
        TARGET_ALT = 3.0

        if self.current_state in ["WANDERING", "SEARCHING"]:
            alt_error = TARGET_ALT - self.current_z
            twist.linear.z = max(-0.5, min(0.5, alt_error * 1.5))

        if self.current_state == "TAKEOFF":
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.linear.z = 0.6 
            twist.angular.z = 0.0

        elif self.current_state == "WANDERING":
            if abs(self.current_x) > BOUNDARY or abs(self.current_y) > BOUNDARY:
                if current_time >= self.escaping_wall_until:
                    self.escaping_wall_until = current_time + 2.5
                    
            if current_time < self.escaping_wall_until:
                angle_to_center = math.atan2(-self.current_y, -self.current_x)
                angle_error = (angle_to_center - self.current_yaw + math.pi) % (2 * math.pi) - math.pi
                twist.linear.x = 0.6
                twist.angular.z = angle_error * 2.5
            else:
                twist.linear.x = 0.6
                twist.angular.z = 0.5 * math.sin(current_time * 0.7) 

        elif self.current_state == "SEARCHING":
            time_in_search = current_time - self.time_entered_search
            
            if time_in_search < 0.2:
                self.search_target_x = round(self.current_x / 1.5) * 1.5
                self.search_target_y = 4.0 if self.current_y < 0 else -4.0
                self.search_phase = "SWEEP_Y"

            target_x = self.search_target_x
            target_y = self.search_target_y if self.search_phase == "SWEEP_Y" else self.current_y

            distance_to_target = math.hypot(target_x - self.current_x, target_y - self.current_y)

            if distance_to_target < 0.6:
                if self.search_phase == "SWEEP_Y":
                    self.search_phase = "SHIFT_X"
                    self.search_target_x += 1.5
                    if self.search_target_x > 4.5: 
                        self.search_target_x = -4.5
                    self.search_target_y *= -1.0 
                else:
                    self.search_phase = "SWEEP_Y"

            angle_to_target = math.atan2(target_y - self.current_y, target_x - self.current_x)
            angle_error = (angle_to_target - self.current_yaw + math.pi) % (2 * math.pi) - math.pi
            
            if abs(angle_error) > 0.5:
                twist.linear.x = 0.2
                twist.angular.z = angle_error * 2.5
            else:
                twist.linear.x = 0.8 
                twist.angular.z = angle_error * 1.5

        elif self.current_state == "LANDING":
            if self.current_z < 1.0:
                twist.linear.x = 0.0
                twist.linear.y = 0.0
                twist.linear.z = -0.3
                twist.angular.z = 0.0
            else:
                Kp = 0.004
                err_x = self.error_x if abs(self.error_x) > 10 else 0.0
                err_y = self.error_y if abs(self.error_y) > 10 else 0.0
                twist.linear.x = -err_y * Kp
                twist.linear.y = -err_x * Kp
                twist.angular.z = 0.0
                pixel_distance = math.sqrt(err_x**2 + err_y**2)
                
                if pixel_distance < 40:
                    twist.linear.z = -0.4 
                else:
                    twist.linear.z = 0.0 

        elif self.current_state in ["DWELLING", "STOPPED"]:
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.linear.z = 0.0 
            twist.angular.z = 0.0

        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = FlightController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
