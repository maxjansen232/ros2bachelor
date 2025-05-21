#!usr/bin/env python3
import rclpy
from rclpy.node import Node
from custom_interfaces.msg import AuvData
from geometry_msgs.msg import Point

class Plant(Node):
    def __init__(self):
        super().__init__("Plant_Node")
        self.get_logger().info("Plant is running smoothly!")
        
        self.pid_sub = self.create_subscription(
            Point, "pid/gain", self.callback, 1)
        
        self.position_pub = self.create_publisher(AuvData, "AuvData", 1)
        
        #DELTA TIME FOR DISCRETE CALCULATIONS
        self.delta_time = 0.8 

        #min/max values for operation of cnc machine. Unit: meters
        self.min_x = 0.0
        self.max_x = 1.0

        self.min_y = 0.0
        self.max_y = 1.0

        self.min_z = -0.200
        self.max_z = 0.0

        #X-AXIS-CONSTANTS
        self.x_mass = 2100
        self.x_damping = 20

        #X-AXIS-VARIABLES
        self.x_position_initial = 0.224 #Initial position, must correspond with CNC initial position

        self.x_position = self.x_position_initial
        self.x_position_1 = self.x_position_initial
        self.x_position_2 = self.x_position_initial

        self.x_velocity = 0 
        self.x_velocity_1 = 0 

        self.x_input = 0
        self.x_input_1 = 0 
        self.x_input_2 = 0

        #Y-AXIS-CONSTANTS
        self.y_mass = 3000 
        self.y_damping = 2500

        #Y-AXIS-VARIABLES
        self.y_position_initial = 0.018 #Initial position, must correspond with CNC initial position

        self.y_position = self.y_position_initial
        self.y_position_1 = self.y_position_initial
        self.y_position_2 = self.y_position_initial

        self.y_velocity = 0 
        self.y_velocity_1 = 0 

        self.y_input = 0
        self.y_input_1 = 0 
        self.y_input_2 = 0

        #Z-AXIS-CONSTANTS
        self.z_mass = 4700
        self.z_damping = 3000

        #Z-AXIS-VARIABLES
        self.z_position_initial = 0.0 #Initial position, must correspond with CNC initial position

        self.z_position = self.z_position_initial
        self.z_position_1 = self.z_position_initial
        self.z_position_2 = self.z_position_initial

        self.z_velocity = 0 
        self.z_velocity_1 = 0 

        self.z_input = 0
        self.z_input_1 = 0 
        self.z_input_2 = 0
       
    #Function that checks if calculated position is safe for CNC-operation
    def max_value_check(self):
        if self.x_position < self.min_x:
            self.get_logger().warning(f"\n X-AXIS:{self.x_position} is less than minimum safety X")
            self.x_position = self.min_x
        elif self.x_position > self.max_x: 
            self.get_logger().warning(f"\n X-AXIS:{self.x_position} is larger than maximum safety X")
            self.x_postion = self.max_x
            
        if self.y_position < self.min_y:
            self.get_logger().warning(f"\n Y-AXIS:{self.y_position} is less than minimum safety Y")
            self.y_position = self.min_y
        elif self.y_position > self.max_y: 
            self.get_logger().warning(f"\n Y-AXIS:{self.y_position} is larger than maximum safety Y")
            self.y_postion = self.max_y
        
        if self.z_position < self.min_z:
            self.get_logger().warning(f"\n Z-AXIS:{self.z_position} is less than minimum safety Z")
            self.z_position = self.min_z
        elif self.z_position > self.max_z: 
            self.get_logger().warning(f"\n Z-AXIS:{self.z_position} is less than minimum safety Z")
            self.z_postion = self.max_z
        
    
    def callback(self, msg_in : Point):
        x_gain = msg_in.x 
        y_gain = msg_in.y 
        z_gain = msg_in.z

        self.calculate_x(x_gain)
        self.calculate_y(y_gain)
        self.calculate_z(z_gain)

        self.max_value_check()
        
        msg_out = AuvData()
        msg_out.pos_x, msg_out.pos_y, msg_out.pos_z = self.x_position, self.y_position, self.z_position
        msg_out.vel_x, msg_out.vel_y, msg_out.vel_z = self.x_velocity, self.y_velocity, self.z_velocity
        
        self.position_pub.publish(msg_out)
        

    def calculate_x(self, pid_x):
        #Cycle variables backwards
        self.x_position_2, self.x_position_1 = self.x_position_1, self.x_position
        self.x_velocity_1 = self.x_velocity
        self.x_input_2, self.x_input_1, self.x_input = self.x_input_1, self.x_input, pid_x

        #Calculate new velocity and new position based on difference equations
        self.x_velocity = (self.x_input*self.delta_time + self.x_input_1*self.delta_time - 
                           self.x_velocity_1 * (self.x_damping*self.delta_time -2*self.x_mass))/(2*self.x_mass + self.x_damping *self.delta_time)

        self.x_position = (self.x_input * self.delta_time**2 + self.x_input_1*2*self.delta_time**2 + self.x_input_2 * self.delta_time**2 + self.x_position_1*8*self.x_mass - self.x_position_2*(4*self.x_mass - 2* self.delta_time*self.x_damping))/(4*self.x_mass + 2*self.delta_time *self.x_damping)

        return self.x_position, self.x_velocity
    

    def calculate_y(self, pid_y):
        #Cycle variables backwards
        self.y_position_2, self.y_position_1 = self.y_position_1, self.y_position
        self.y_velocity_1 = self.y_velocity
        self.y_input_2, self.y_input_1, self.y_input = self.y_input_1, self.y_input, pid_y

        #Calculate new velocity and new position based on difference equations
        self.y_velocity = (self.y_input*self.delta_time + self.y_input_1*self.delta_time - 
                           self.y_velocity_1 * (self.y_damping*self.delta_time -2*self.y_mass))/(2*self.y_mass + self.y_damping *self.delta_time)

        self.y_position = (self.y_input * self.delta_time**2 + self.y_input_1*2*self.delta_time**2 + self.y_input_2 * self.delta_time**2 + self.y_position_1*8*self.y_mass - self.y_position_2*(4*self.y_mass - 2* self.delta_time*self.y_damping))/(4*self.y_mass + 2*self.delta_time *self.y_damping)

        return self.y_position, self.y_velocity

    
    def calculate_z(self, pid_z):
            #Cycle variables backwards
            self.z_position_2, self.z_position_1 = self.z_position_1, self.z_position
            self.z_velocity_1 = self.z_velocity
            self.z_input_2, self.z_input_1, self.z_input = self.z_input_1, self.z_input, pid_z

            #Calculate new velocity and new position based on difference equations
            self.z_velocity = (self.z_input*self.delta_time + self.z_input_1*self.delta_time - 
                            self.z_velocity_1 * (self.z_damping*self.delta_time -2*self.z_mass))/(2*self.z_mass + self.z_damping *self.delta_time)

            self.z_position = (self.z_input * self.delta_time**2 + self.z_input_1*2*self.delta_time**2 + self.z_input_2 * self.delta_time**2 + self.z_position_1*8*self.z_mass - self.z_position_2*(4*self.z_mass - 2* self.delta_time*self.z_damping))/(4*self.z_mass + 2*self.delta_time *self.z_damping)
            
            return self.z_position, self.z_velocity


def main(args=None):
    rclpy.init(args=args)
    node = Plant()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
