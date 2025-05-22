#!usr/bin/env python3

import rclpy 
from rclpy.node import Node
from geometry_msgs.msg import Point
from custom_interfaces.msg import MagneticStamped


class PID(Node):
    
    def __init__(self):
        super().__init__("PID_node")
        self.gain_x = 0
        self.gain_y = 0 
        self.gain_z = 0
        
        self.magnetic_subscriber = self.create_subscription(
                                MagneticStamped, "rov/magnetic", self.magnetic_callback, 10)
        
        
        
        
        
        self.gain_publisher = self.create_publisher(
                                                Point, "pid/gain", 10)
        
        

        #pid time parameter
        self.current_time = 0.01
        #PID parameters for x direction
        self.Kp_x = 0.012 #ts calc x 1.1
        self.Kd_x = 0.2 #ts calc x 0.9
        #self.Kd_x = 0 
        self.Ki_x = 0.000
        self.previous_error_x = 0
        self.integral_x = 0 
        
        #PID parameters for y direction
        self.Kp_y = 0.1133
        self.Kd_y = 0.05
        self.Ki_y = 0.0
        self.previous_error_y = 0
        self.integral_y = 0 

        #PID parameters for z direction
        self.Kp_z= 0.05
        self.Kd_z = 0.0
        self.Ki_z = 0.0
        self.previous_error_z = 0
        self.integral_z = 0 
        
    
    def magnetic_callback(self, msg : MagneticStamped): #A: msg.x B: msg.y C: msg.z
        A = msg.sensor_a
        B = msg.sensor_b
        C = msg.sensor_c
        Z = msg.sensor_z

        previous_time = self.current_time
        self.current_time = msg.time
        
        delta_time = self.current_time -previous_time

         
        #gain calculation for x direction
        error_x = (0.5*A+0.5*B-C)/(0.5*A +0.5*B + C)

        Proportional_x = self.Kp_x * error_x
        
        self.integral_x += error_x * delta_time
        Integral_Out_x = self.Ki_x * self.integral_x
        if delta_time != 0:
            derivative_x = (error_x-self.previous_error_x)/delta_time
            Derivative_Out_x = self.Kd_x * derivative_x
        else: Derivative_Out_x = 0

        self.gain_x = (Proportional_x+ Integral_Out_x + Derivative_Out_x)
        self.previous_error_x= error_x
        
        #gain calculation for y direction
        error_y = (A - 1.325*B)/(A + 1.325*B)
        
        Proportional_y = self.Kp_y * error_y
        
        self.integral_y += error_y * delta_time
        Integral_Out_y = self.Ki_y * self.integral_y
        if delta_time != 0:
            derivative_y = (error_y-self.previous_error_y)/delta_time
            Derivative_Out_Y = self.Kd_y * derivative_y
        else: Derivative_Out_Y = 0

        self.gain_y = (Proportional_y + Integral_Out_y + Derivative_Out_Y)
        self.previous_error_y = error_y
        
        #gain calculation for z direction
        error_z = (Z-13400)/13400
        
        Proportional_z = self.Kp_z * error_z
        
        self.integral_z += error_z * delta_time
        Integral_Out_z = self.Ki_z * self.integral_z
        if delta_time != 0:
            derivative_z = (error_z-self.previous_error_z)/delta_time
            Derivative_Out_z = self.Kd_z * derivative_z
        else: Derivative_Out_z = 0

        self.gain_z = (Proportional_z + Integral_Out_z + Derivative_Out_z)
        self.previous_error_z = error_z
        
        self.get_logger().info(f"\n SENSOR_DATA:\n       Sensor_A: {A} \n       Sensor_B: {B} \n       Sensor_C: {C} \n       Sensor_Z: {Z} \n GAIN: \n       x-gain: {self.gain_x} \n       y_gain: {self.gain_y} \n       z_gain: {self.gain_z}")
        self.gain_pub()
    
    
    
    def gain_pub(self):
        msg = Point()
        msg.x = float(self.gain_x)
        msg.y = float(self.gain_y)
        msg.z = float(self.gain_z)
        self.get_logger().info(f"message z is {msg.z}")
        self.gain_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PID()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()

            
            
        

