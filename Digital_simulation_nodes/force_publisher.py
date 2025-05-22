#!usr/bin/env python3

import rclpy
from rclpy.node import Node
from ros_gz_interfaces.msg import EntityWrench
from geometry_msgs.msg import Wrench, Point
   

class ForcePublisher(Node):

    def __init__(self):
        super().__init__("ForcePublisher")
        self.force_publisher = self.create_publisher(
                                EntityWrench, "/world/ROV_ENV_01/wrench", 10)
        self.timer = self.create_timer(0.01, self.send_force)

        self.pid_sub = self.create_subscription(
                                                Point, "/pid/gain", self.pid_sub, 10)
        
    
        self.x_force = 0
        self.y_force = 0
        self.z_force = 0
    
    def pid_sub(self, msg : Point):
            self.x_force = msg.x * 20
            self.y_force = msg.y * 20
            self.z_force = msg.z * 0.8
    
    def send_force(self):
        msg = EntityWrench() 
        msg.entity.name = "ROV"
        msg.entity.id = 8
        msg.wrench.force.x = float(self.x_force)
        #msg.wrench.force.x = float(0) #just for debugging the y direction!!!
        msg.wrench.force.y = float(self.y_force)
        msg.wrench.force.z = float(self.z_force)
        msg.wrench.torque.x = float(0)
        msg.wrench.torque.y = float(0)
        msg.wrench.torque.z = float(0)

        self.get_logger().info(f"Applying force \n Force_X: {round(self.x_force)}N \n Force_Y: {round(self.y_force)}N \n Force_Z: {round(self.z_force)}N")
        self.force_publisher.publish(msg)
        
        
def main(args = None):
    rclpy.init(args = args)
    node = ForcePublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
