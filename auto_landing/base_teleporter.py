import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading
import subprocess
import traceback

class BaseTeleporter(Node):
    def __init__(self):
        super().__init__('base_teleporter')
        self.current_state = "UNKNOWN"
        self.sub = self.create_subscription(String, '/mission_state', self.state_callback, 10)
        
        self.input_thread = threading.Thread(target=self.user_input_loop)
        self.input_thread.daemon = True
        self.input_thread.start()

    def state_callback(self, msg):
        self.current_state = msg.data

    def user_input_loop(self):
        print("\n==================================", flush=True)
        print("   BASE TELEPORTER ONLINE", flush=True)
        print("==================================", flush=True)
        print("Boundary Limit: -4.5 to 4.5", flush=True)
        print("Type coordinates (e.g., '3 -2') or 'quit' to exit.", flush=True)
        
        while rclpy.ok():
            try:
                user_input = input("\nEnter new X Y: ")
                if user_input.lower() in ['q', 'quit', 'exit']:
                    print("Shutting down teleporter...", flush=True)
                    rclpy.shutdown()
                    break
                
                parts = user_input.split()
                if len(parts) != 2:
                    print("[ERROR] Please enter exactly two numbers separated by a space.", flush=True)
                    continue
                    
                x = float(parts[0])
                y = float(parts[1])
                
                self.attempt_teleport(x, y)
                
            except ValueError:
                print("[ERROR] Invalid input. Please enter numbers.", flush=True)
            except Exception as e:
                print(f"\n[CRITICAL ERROR] {e}", flush=True)
                traceback.print_exc()

    def attempt_teleport(self, x, y):
        if self.current_state == "UNKNOWN":
            print("[REJECTED] Simulation clock is paused! Hit 'Play' in Gazebo first.", flush=True)
            return

        if abs(x) > 4.5 or abs(y) > 4.5:
            print(f"[REJECTED] Coordinates ({x}, {y}) are outside the boundary area!", flush=True)
            return
            
        if self.current_state in ["LANDING", "DWELLING", "SEARCHING"]:
            print(f"[REJECTED] Drone is busy in {self.current_state} mode. You can only teleport during WANDERING!", flush=True)
            return
            
        print(f"[APPROVED] Relocating base to X: {x}, Y: {y}...", flush=True)
        
        req_payload = f'name: "landing_base", position: {{x: {x}, y: {y}, z: 0.005}}'
        
        gz_cmd = [
            'gz', 'service', '-s', '/world/landing_world/set_pose',
            '--reqtype', 'gz.msgs.Pose',
            '--reptype', 'gz.msgs.Boolean',
            '--timeout', '2000',
            '--req', req_payload
        ]
        
        try:
            result = subprocess.run(gz_cmd, capture_output=True, text=True)
            output = result.stdout.lower() + result.stderr.lower()
            
            if result.returncode != 0 or "timed out" in output or "error" in output or "failed" in output:
                print(f"[ERROR] Gazebo rejected the command.", flush=True)
                if output.strip():
                    print(f"        (Gazebo Reply: {output.strip()})", flush=True)
            else:
                print("[SUCCESS] Base successfully moved in Gazebo!", flush=True)
                
        except Exception as e:
            print(f"[CRITICAL ERROR] Could not contact Gazebo: {e}", flush=True)

def main(args=None):
    rclpy.init(args=args)
    node = BaseTeleporter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

if __name__ == '__main__':
    main()
