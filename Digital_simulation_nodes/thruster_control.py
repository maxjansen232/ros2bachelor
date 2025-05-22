#!usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64
from geometry_msgs.msg import Point

class ThrusterController(Node):

    def __init__(self):
        super().__init__("ThrusterController")

        self.stern_thrust_pub = self.create_publisher(
                                                Float64, "/rov_cmd/stern_thrust", 10)
        self.bow_thrust_pub = self.create_publisher(
                                                Float64, "/rov_cmd/bow_thrust", 10)
        self.starboard_thrust_pub = self.create_publisher(
                                                Float64, "/rov_cmd/starboard_thrust", 10)
        self.port_thrust_pub = self.create_publisher(
                                                Float64, "/rov_cmd/port_thrust", 10)
        self.top_thrust_pub = self.create_publisher(
                                                Float64, "/rov_cmd/top_thrust", 10)
        self.bottom_thrust_pub = self.create_publisher(
                                                Float64, "/rov_cmd/bottom_thrust", 10)
        
        self.pid_sub = self.create_subscription(
                                                Point, "/pid/gain", self.pid_callback, 10)
        
    
    def pid_callback(self, msg : Point):
        x_gain = msg.x
        y_gain = msg.y
        z_gain = msg.z
        self.get_logger().info(f"msg z is {msg.z}")
        msg_thrust_x = Float64()
        msg_thrust_y = Float64()
        msg_thrust_z = Float64()
        msg_zero = Float64()
        msg_zero.data = float(0.0) 

        
        
        if x_gain > 0:
            msg_thrust_x.data = float(x_gain)
            
            self.stern_thrust_pub.publish(msg_thrust_x) #kraft fremover
            self.bow_thrust_pub.publish(msg_zero)
        else: 
            msg_thrust_x.data = float(-x_gain)
            self.bow_thrust_pub.publish(msg_thrust_x) #kraft bakover
            self.stern_thrust_pub.publish(msg_zero)
        
        if y_gain > 0:
            msg_thrust_y.data = float(y_gain)
            self.starboard_thrust_pub.publish(msg_thrust_y)
            self.port_thrust_pub.publish(msg_zero)
        else: 
            msg_thrust_y.data = float(-y_gain)
            self.port_thrust_pub.publish(msg_thrust_y)
            self.starboard_thrust_pub.publish(msg_zero)
        
        if z_gain > 0:
            msg_thrust_z.data = float(z_gain)
            self.bottom_thrust_pub.publish(msg_thrust_z)
            self.top_thrust_pub.publish(msg_zero)
        else:
            msg_thrust_z.data = float(-z_gain)
            self.top_thrust_pub.publish(msg_thrust_z)
            self.bottom_thrust_pub.publish(msg_zero)
        
        
        self.get_logger().info(f"\n X: {msg_thrust_x.data}N \n Y: {msg_thrust_y.data}N \n Z: {msg_thrust_z.data}N") 

def main(args = None):
    rclpy.init(args = args)
    node = ThrusterController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
        
