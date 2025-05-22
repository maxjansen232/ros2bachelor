#!usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, PoseArray
from nav_msgs.msg import Odometry
import pandas as pd
from custom_interfaces.msg import MagneticStamped

class MagNode(Node):

    def __init__(self):
        super().__init__("Mag_Node")
        self.position_subscriber =  self.create_subscription(
            Odometry, "/rov/odometry", 
            self.point_callback, 1)
        
        self.magnetic_publisher = self.create_publisher(
                        MagneticStamped, "rov/magnetic", 1)
    
    def point_callback(self, msg: Odometry):
        rov_pose = msg.pose.pose
        rov_x = float(1000*rov_pose.position.x)
        rov_y = float(1000*rov_pose.position.y)
        rov_z = float(1000*rov_pose.position.z)
        rov_time = msg.header.stamp.sec + msg.header.stamp.nanosec*10**(-9)
        self.get_logger().info(f"\n Simulation time: {rov_time} \n position_X: {round(rov_x, 2)} \n position_Y: {round(rov_y, 2)} \n position_Z: {round(rov_z, 2)}")
        self.send_magnetic_data(rov_x, rov_y, rov_z, rov_time)

    '''
    def point_callback(self, msg : PoseArray):
        ROV_pose = msg.poses[0]
        ROV_x = float(1000* ROV_pose.position.x)
        ROV_y = float(1000* ROV_pose.position.y)
        ROV_z = float(1000* ROV_pose.position.z)
        ROV_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 10**(-9)
        self.get_logger().info(f"\n Simulation time: {ROV_time} \n position_X: {round(ROV_x, 2)} \n position_Y: {round(ROV_y, 2)} \n position_Z: {round(ROV_z, 2)}")
        self.send_magnetic_data(ROV_x, ROV_y, ROV_z, ROV_time)
    '''

    def send_magnetic_data(self, drone_x, drone_y, drone_z, drone_time):
        
        mag_msg = MagneticStamped()
        mag_msg.sensor_a, mag_msg.sensor_b, mag_msg.sensor_c, mag_msg.sensor_z = self.data_finder(drone_x, drone_y, drone_z)
        mag_msg.time = float(drone_time)
        
        self.magnetic_publisher.publish(mag_msg)
        

    def data_finder(self, drone_x, drone_y, drone_z):

        old_data_set = '/home/lodrik/ros2_wss/src/my_robot_controller/my_robot_controller/magnetic_data_shifted.csv'
        df = pd.read_csv(old_data_set)

        new_columns = ["posx", "posy", "posz", "a", "b", "c", "z"]
        df.columns = new_columns

        drone_pos = {"x":drone_x, "y":drone_y, "z":drone_z}
       
        def rounded_z(z):
            return 10 * round(z/10)
        
        result = df[df['posz'] == rounded_z(drone_pos["z"])]
        result = result[(drone_pos['x']-10 < result['posx'])]
        result = result[(drone_pos['x']+10 > result['posx'])]
        result = result[(drone_pos['y']-10 < result['posy'])]
        result = result[(drone_pos['y']+10 > result['posy'])]
        result = result.reset_index(drop=True)
        
        closest_index = 0
        closest_point = {"x": float(result["posx"][closest_index]), "y": float(result["posy"][closest_index])}

        
        def closest(drone_x, drone_y):
            distance = 0
            closest_square = 999999999
            closest_index = 0
            for i in range(0,len(result)):
                distance = (float(result["posx"][i]) -drone_x)**2 + (float(result["posy"][i])- drone_y)**2
                if distance < closest_square:
                    closest_index = i
                    closest_square = distance
            return closest_index

        best_index = closest(drone_pos["x"], drone_pos["y"])  
    
        a_value = float(result.iloc[best_index]["a"])
        b_value = float(result.iloc[best_index]["b"])
        c_value = float(result.iloc[best_index]["c"])
        z_value = float(result.iloc[best_index]["z"])

        return a_value, b_value, c_value, z_value

    
        



def main(args=None):
    rclpy.init(args=args)
    node = MagNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
