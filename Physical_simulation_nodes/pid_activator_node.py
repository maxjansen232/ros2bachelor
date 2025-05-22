import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
import time
import json

class pid_activator(Node):
    def __init__(self):
        super().__init__('pid_activator')
        self.cnc_status_subscriber = self.create_subscription(String, "cnc/status", self.cnc_status_callback, 10)
        self.pid_start_pub = self.create_publisher(Float32, "activate_pid", 10)


    def cnc_status_callback(self, msg : String):
        try:
            message = json.loads(msg.data)
            self.get_logger().info(f'\n CNC_status: {message["state"]}')
            isidle = message["state"] == "Idle"
            self.get_logger().error(f"{isidle}")
        except json.JSONDecodeError:
            self.last_response = "Error parsing status JSON"
            isidle == False
        
        if isidle == True: 
            activatemsg = Float32()
            activatemsg.data = 1.0
            self.pid_start_pub.publish(activatemsg)
            time.sleep(0.5)
        else: time.sleep(0.01)

def main(args = None):
    rclpy.init(args=args)
    node = pid_activator()
    rclpy.spin(node)
    rclpy.shutdown()
        
if __name__== "__main__":
    main()
