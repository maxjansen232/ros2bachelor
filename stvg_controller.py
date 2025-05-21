#!usr/bin/env python3
import rclpy 
from rclpy.node import Node
from geometry_msgs.msg import Point
from numpy import arctan
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from scipy import signal

class controller(Node):
    def __init__(self):
        super().__init__('controller')
        #subscriber and publisher initialization 
        self.magnetic_subscriber = self.create_subscription(
                                Float32MultiArray, "magnetic_sensor_data", self.update_sensors_callback, 10) #optionally float32multiarray msg
        
        self.calculate_pid_gain_sub = self.create_subscription(Float32, "activate_pid", self.magnetic_callback, 10)

        self.gain_publisher = self.create_publisher(
                                                Point, "pid/gain", 10)
        
        self.sensor_a = 1000
        self.sensor_b = 1000
        self.sensor_c = 5000
        self.sensor_z = 30000
        #loop check i 
        self.i = 0 

        #Time definitons
        self.current_time = 1.0
        self.previous_time = 0
        
        #X-AXIS DEFINITIONS
        self.x_out = 0.0
        
        self.x_in= 0.0
        self.x_in_1= 0.0 
        self.x_in_2 = 0.0
        
        self.x_Kp = 15
        self.x_Kd = 250
        self.x_Ki = 0

        self.x_d_filter_constant = 0.8
        
        self.x_p = 0 

        self.x_d = 0 
        self.x_d_1 = 0

        self.x_i = 0
        self.x_i_1= 0 
        
        #Y-AXIS DEFINITIONS
        self.y_out = 0.0
        
        self.y_in= 0.0
        self.y_in_1= 0.0 
        self.y_in_2 = 0.0
        
        self.y_Kp = 20
        self.y_Kd = 200
        self.y_Ki = 0.0

        self.y_d_filter_constant = 0.8
        
        self.y_p = 0 

        self.y_d = 0 
        self.y_d_1 = 0

        self.y_i = 0
        self.y_i_1= 0 
        
        #Z-AXIS DEFINITIONS
        self.z_out = 0.0 

        self.z_in= 0.0
        self.z_in_1= 0.0 
        self.z_in_2 = 0.0
        
        self.z_Kp = 10
        self.z_Kd = 50
        self.z_Ki = 0

        self.z_d_filter_constant = 0.8
        
        self.z_p = 0 

        self.z_d = 0 
        self.z_d_1 = 0

        self.z_i = 0
        self.z_i_1= 0 

        self.z_commit = 0
        self.z_magnetic_goal = 1000

        #SENSOR-LOW-PASS-FILTER VARIABLES
        self.sensor_a_1 = 0
        self.sensor_b_1 = 0
        self.sensor_c_1 = 0
 
        self.sensor_z = 0
        
        self.unfiltered_a_1 = 0
        self.unfiltered_b_1 = 0
        self.unfiltered_c_1 = 0
        self.unfiltered_a = 0
        self.unfiltered_b = 0
        self.unfiltered_c = 0

        self.sensor_a = 0
        self.sensor_b = 0
        self.sensor_c = 0



    #RECIEVES MAGNETIC SENSOR DATA
    def update_sensors_callback(self, msg : Float32MultiArray):
        self.sensor_a_1 = self.sensor_a
        self.sensor_b_1 = self.sensor_b
        self.sensor_c_1 = self.sensor_c
 
        self.sensor_z = msg.data[3]
        
        self.unfiltered_a_1 = self.unfiltered_a
        self.unfiltered_b_1 = self.unfiltered_b
        self.unfiltered_c_1 = self.unfiltered_c
        self.unfiltered_a = msg.data[0]
        self.unfiltered_b = msg.data[2]
        self.unfiltered_c = msg.data[1]

        self.sensor_a = 0.846*self.sensor_a_1 + 0.079*(self.unfiltered_a+self.unfiltered_a_1)
        self.sensor_b = 0.846*self.sensor_b_1 + 0.079*(self.unfiltered_b+self.unfiltered_b_1)
        self.sensor_c = 0.846*self.sensor_c_1 + 0.079*(self.unfiltered_c+self.unfiltered_c_1)

        

    def magnetic_callback(self, msg : Float32):
        #output definition
        gain_msg = Point()
        #input mapping
        A = self.sensor_a
        B = self.sensor_b
        C = self.sensor_c
        Z = self.sensor_z

        
        self.i += 1
        #Delta time for calculations
        dt = 0.8 
        
        #X-AXIS PID CALCULATIONS
        self.x_in_1 = self.x_in
        self.x_i_1 = self.x_i
        self.x_d_1 = self.x_d
        self.x_in = (0.5*A+0.5*B-C)/(0.5*A +0.5*B + C)
        self.x_in = 0.17*arctan(self.x_in)
  
        #PROPORTIONAL GAIN
        self.x_p = self.x_Kp*self.x_in 
        #DERIVATIVE GAIN
        self.x_d = 2 * self.x_Kd /(2*self.x_d_filter_constant + dt) *(self.x_in -self.x_in_1) + self.x_d_1*(2*self.x_d_filter_constant - dt)/ (2*self.x_d_filter_constant+dt)
        #INTEGRAL GAIN
        self.x_i = self.x_Ki *dt /2 *(self.x_in+ self.x_in_1) + self.x_i_1

        #MAX Ki GAIN
        if self.x_i > 8:
            self.x_i = 8
        if self.x_i < -8:
            self.x_i = -8
        self.x_out = self.x_p + self.x_d + self.x_i

        #Y-AXIS PID CALCULATIONS
        self.y_in_1 = self.y_in
        self.y_i_1 = self.y_i 
        self.y_d_1 = self.y_d
        self.y_in = (A-B)/(A+B)
       
        self.y_p = self.y_Kp*self.y_in
        self.y_d = 2 * self.y_Kd /(2*self.y_d_filter_constant + dt) *(self.y_in -self.y_in_1) + self.y_d_1*(2*self.y_d_filter_constant - dt)/ (2*self.y_d_filter_constant+dt)
        self.y_i = self.y_Ki *dt /2 *(self.y_in+ self.y_in_1) + self.y_i_1

        self.y_out = self.y_p + self.y_d + self.y_i
        
        
        #z-axis PID calculations
        self.z_in_1 = self.z_in
        self.z_i_1 = self.z_i 
        self.z_d_1 = self.z_d
        self.z_in = (Z-self.z_magnetic_goal)/self.z_magnetic_goal
            
        self.z_p = self.z_Kp*self.z_in
        self.z_d = 2 * self.z_Kd /(2*self.z_d_filter_constant + dt) *(self.z_in -self.z_in_1) + self.z_d_1*(2*self.z_d_filter_constant - dt)/ (2*self.z_d_filter_constant+dt)
        self.z_i = self.z_Ki *dt /2 *(self.z_in+ self.z_in_1) + self.z_i_1

        self.z_out = self.z_p + self.z_d + self.z_i
        
      
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
        ERROR_X {self.x_in}
        ERROR_Y {self.y_in}
                                """)

        #PUBLISH GAINS TO PLANT
        gain_msg.x = float(self.x_out) 
        gain_msg.y = float(self.y_out) 
        gain_msg.z = float(self.z_out)
        self.gain_publisher.publish(gain_msg)

        #CHECKING FOR COMMIT MODE
        if abs(self.y_in) < 0.13 and abs(self.x_in) < 0.1:
            self.i += 1 
            if self.i > 10:
                self.z_magnetic_goal = 34000 
        else: self.i = 0  #resets counter if error is too high



def main(args=None):
    rclpy.init(args=args)
    node = controller()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
