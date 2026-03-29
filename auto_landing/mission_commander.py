import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from rclpy.parameter import Parameter

class MissionCommander(Node):
    def __init__(self):
        super().__init__('mission_commander')
        self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])

        self.state_pub = self.create_publisher(String, '/mission_state', 10)
        self.error_sub = self.create_subscription(Point, '/landing_target_error', self.error_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.current_state = "SEARCHING"
        self.state_start_time = 0.0
        self.last_target_time = 0.0
        self.current_z = 3.0
        self.clock_initialized = False

        self.timer = self.create_timer(0.1, self.fsm_loop)
        self.get_logger().info("Hardened Brain Online. Awaiting Gazebo Play...")

    def get_time(self):
        return self.get_clock().now().nanoseconds / 1e9

    def odom_callback(self, msg):
        self.current_z = msg.pose.pose.position.z

    def error_callback(self, msg):
        if self.current_state in ["SEARCHING", "LANDING"]:
            self.last_target_time = self.get_time()
            if self.current_state == "SEARCHING":
                self.change_state("LANDING")

    def change_state(self, new_state):
        if self.current_state != new_state:
            self.current_state = new_state
            self.state_start_time = self.get_time()
            self.get_logger().info(f"--- STATE CHANGED TO: {self.current_state} ---")

    def fsm_loop(self):
        current_time = self.get_time()
        if current_time <= 0.0:
            return 

        if not self.clock_initialized:
            self.state_start_time = current_time
            self.last_target_time = current_time
            self.clock_initialized = True
            self.get_logger().info("Simulation Active!")

        elapsed_time = current_time - self.state_start_time
        
        if self.current_state == "TAKEOFF":
            if self.current_z >= 2.8:
                self.change_state("WANDERING")

        elif self.current_state == "WANDERING":
            if elapsed_time >= 30.0:
                self.change_state("SEARCHING")

        elif self.current_state == "SEARCHING":
            pass 

        elif self.current_state == "LANDING":
            time_since_target = current_time - self.last_target_time
            
            if time_since_target > 0.5:
                if self.current_z < 0.6: 
                    self.get_logger().info("Touchdown. Triggering 5-second dwell.")
                    self.change_state("DWELLING")
                elif time_since_target > 2.0:
                    self.get_logger().warn("Target lost mid-air. Resuming search.")
                    self.change_state("SEARCHING")

        elif self.current_state == "DWELLING":
            if elapsed_time >= 5.0:
                self.change_state("TAKEOFF")

        msg = String()
        msg.data = self.current_state
        self.state_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MissionCommander()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
