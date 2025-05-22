#!usr/bin/env python3 

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class VelocityLogger(Node):
    
    def __init__(self):
        super().__init__("VelocityLogger")
        self.previous_x = 0
        self.previous_y = 0 
        self.previous_z = 0
        self.previous_time = 0
                
        self.position_subscriber =  self.create_subscription(
            Odometry, "/rov/odometry", 
            self.velocity_logger, 10)

    def velocity_logger(self, msg : Odometry):
            current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 10**(-9)
            rov_pose = msg.pose.pose
            rov_x = float(1000* rov_pose.position.x)
            rov_y = float(1000* rov_pose.position.y)
            rov_z = float(1000* rov_pose.position.z)
            if self.previous_time != 0: 
                vel_x = (rov_x-self.previous_x)/(current_time-self.previous_time)
                vel_y = (rov_y-self.previous_y)/(current_time-self.previous_time)
                vel_z = (rov_z-self.previous_z)/(current_time-self.previous_time)
                absolute_velocity = (vel_x**2 + vel_y**2 + vel_z**2)**(0.5)
                self.get_logger().info(f"Velocity Logger: \n Absolute: {round(absolute_velocity, 2)} \n X: {round(vel_x,2)}mm/s \n Y: {round(vel_y,2)}mm/s \n Z: {round(vel_z,2)}mm/s")
            
            self.previous_time = current_time
            self.previous_x = rov_x
            self.previous_y = rov_y
            self.previous_z = rov_z

def main(args=None):
    rclpy.init(args=args)
    node = VelocityLogger()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()

                

