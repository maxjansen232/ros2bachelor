#!usr/bin/env python3

import rclpy 
from rclpy.node import Node
from geometry_msgs.msg import Point
from custom_interfaces.msg import MagneticStamped
from numpy import arctan
class controller(Node):

    def __init__(self):
        super().__init__('controller')
        #subscriber and publisher initialization 
        self.magnetic_subscriber = self.create_subscription(
                                MagneticStamped, "rov/magnetic", self.magnetic_callback, 10)
        
        self.gain_publisher = self.create_publisher(
                                                Point, "pid/gain", 10)
        #loop check i 
        self.i = 0 

        #Time definitons
        self.current_time = 0 
        self.previous_time = 0
        
        #X-axis definitions
        self.x_out = 0.0
        
        self.x_in= 0.0
        self.x_in_1= 0.0 
        self.x_in_2 = 0.0
        
        self.x_Kp = 20
        self.x_Kd = 200
        self.x_Ki = 1

        self.x_d_filter_constant = 0.8
        
        self.x_p = 0 

        self.x_d = 0 
        self.x_d_1 = 0

        self.x_i = 0
        self.x_i_1= 0 
        
        #y axis definitions
        self.y_out = 0.0
        
        self.y_in= 0.0
        self.y_in_1= 0.0 
        self.y_in_2 = 0.0
        
        self.y_Kp = 70
        self.y_Kd = 100
        self.y_Ki = 0.5

        self.y_d_filter_constant = 0.8
        
        self.y_p = 0 

        self.y_d = 0 
        self.y_d_1 = 0

        self.y_i = 0
        self.y_i_1= 0 
        #z axis definitions
        self.z_out = 0.0 

        self.z_in= 0.0
        self.z_in_1= 0.0 
        self.z_in_2 = 0.0
        
        self.z_Kp = 20
        self.z_Kd = 50
        self.z_Ki = 0.001

        self.z_d_filter_constant = 0.8
        
        self.z_p = 0 

        self.z_d = 0 
        self.z_d_1 = 0

        self.z_i = 0
        self.z_i_1= 0 

        self.z_commit = 0
        self.z_magnetic_goal = 7000
        
    def magnetic_callback(self, msg : MagneticStamped):
        #output definition
        gain_msg = Point()
        #input mapping
        A = msg.sensor_a
        B = msg.sensor_b
        C = msg.sensor_c
        Z = msg.sensor_z

        #time calculations
        self.i += 1

        self.previous_time = self.current_time
        self.current_time = msg.time
        #dt = self.current_time - self.previous_time //KEEP ACTIVE FOR GAZEBO RUNNING
        dt = 0.2 #KEEP ACTIVE FOR PLANT/DISCRETE RUNNING
        
        
        #x-axis PID calculations
        self.x_in_1 = self.x_in
        self.x_i_1 = self.x_i 
        self.x_d_1 = self.x_d
        self.x_in = (0.5*A+0.5*B-C)/(0.5*A +0.5*B + C)
        self.x_in = 0.17*arctan(self.x_in)
      
        self.x_p = self.x_Kp*self.x_in
        self.x_d = 2 * self.x_Kd /(2*self.x_d_filter_constant + dt) *(self.x_in -self.x_in_1) + self.x_d_1*(2*self.x_d_filter_constant - dt)/ (2*self.x_d_filter_constant+dt)
        self.x_i = self.x_Ki *dt /2 *(self.x_in+ self.x_in_1) + self.x_i_1
        if self.x_i > 8:
            self.x_i = 8
        if self.x_i < -8:
            self.x_i = -8
        self.x_out = self.x_p + self.x_d + self.x_i

        #y-axis PID calculations
        self.y_in_1 = self.y_in
        self.y_i_1 = self.y_i 
        self.y_d_1 = self.y_d
        #self.y_in = (A - 1.325*B)/(A + 1.325*B) weighet inputs, no shift in dataset
        self.y_in = (A-B)/(A+B)
        
      
        self.y_p = self.y_Kp*self.y_in
        self.y_d = 2 * self.y_Kd /(2*self.y_d_filter_constant + dt) *(self.y_in -self.y_in_1) + self.y_d_1*(2*self.y_d_filter_constant - dt)/ (2*self.y_d_filter_constant+dt)
        self.y_i = self.y_Ki *dt /2 *(self.y_in+ self.y_in_1) + self.y_i_1

        self.y_out = self.y_p + self.y_d + self.y_i
        
        #z-axis PID calculations
        if self.z_commit == 0:
            self.z_in_1 = self.z_in
            self.z_i_1 = self.z_i 
            self.z_d_1 = self.z_d
            self.z_in = (Z-self.z_magnetic_goal)/self.z_magnetic_goal
            
        
            self.z_p = self.z_Kp*self.z_in
            self.z_d = 2 * self.z_Kd /(2*self.z_d_filter_constant + dt) *(self.z_in -self.z_in_1) + self.z_d_1*(2*self.z_d_filter_constant - dt)/ (2*self.z_d_filter_constant+dt)
            self.z_i = self.z_Ki *dt /2 *(self.z_in+ self.z_in_1) + self.z_i_1

            self.z_out = self.z_p + self.z_d + self.z_i
        else: self.z_out = -50.0
        #logging values to terminal
        self.get_logger().info(f"""
SENSOR DATA:
        Sensor_A: {A} 
        Sensor_B: {B} 
        Sensor_C: {C} 
        Sensor_Z: {Z} 
GAIN: 
        x-gain: {self.x_out} 
        y-gain: {self.y_out} 
        z-gain: {self.z_out}
Delta time:
        dt = {dt}
                                """)

        #publish to thruster
        gain_msg.x = self.x_out 
        gain_msg.y = 0.0 #self.y_out
        gain_msg.z = 0.0 #self.z_out
        self.gain_publisher.publish(gain_msg)

        #checking for commit mode
        if abs(self.y_in) < 0.05 and abs(self.x_in) < 0.05:
            self.i += 1 
            if self.i > 20:
                self.z_commit = 1 
                #self.z_magnetic_goal = 60000
        else: self.i = 0



def main(args=None):
    rclpy.init(args=args)
    node = controller()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()

            