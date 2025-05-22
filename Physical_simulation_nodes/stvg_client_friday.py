import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from custom_interfaces.msg import AuvData
from custom_interfaces.srv import CNCService
import numpy as np
import time

class stvg_client(Node):
    def __init__(self):
        super().__init__('stvg_client')
        
        self.g_code_list = []
        
        #subscriber to recieve position_data 
        self.plant_pos_sub = self.create_subscription(AuvData, "AuvData", self.g_code_callback, 10)
        
        #Client for sending g-code to CNC-machine
        self.test_client = self.create_client(CNCService, 'cnc/send_gcode')
                

    def g_code_callback(self, msg: AuvData):
        #RECIEVE POSITION AND VELOCITY FROM PLANT
        pos_x, pos_y, pos_z = msg.pos_x, msg.pos_y, msg.pos_z 
        vel_x, vel_y, vel_z = msg.vel_x, msg.vel_y, msg.vel_z 
        
        pos_x = pos_x*1000 #Convert from meters to millimeters
        pos_y = pos_y*1000
        pos_z = pos_z*1000

        #COMPUTE VELOCITY
        velocity = np.sqrt(vel_x**2 + vel_y**2 + vel_z**2) #Combine velocity on all axis to one velocity
        velocity = velocity*1000*60 #Convert velocity from m/s, to mm/min
        
        #Check velocity and height for safe CNC operation
        
        if velocity > 1650: #Maximum velocity to send to CNC
            velocity = 1650

        if pos_z < -200: #Check Z-axis, for safe minimum Z
            pos_z = -200
        
        #Format position and velocity as a g-code string
        
        decimals = 2    #Rounding data to two decimals
        
        #CREATE G-CODE STRING
        g_code = f"G01 X{np.round(pos_x, decimals)} Y{np.round(pos_y, decimals)} Z{np.round(pos_z, decimals)} F{np.round(velocity, 0)}" # G01 - CNC command for linear displacement
        
        #SEND G-CODE TO CNC
        request = CNCService.Request()
        request.gcode = g_code
        self.get_logger().info(f"Sending gcode: {request.gcode}")       
        response = self.test_client.call_async(request)
        
def main(args = None):
    rclpy.init(args=args)
    node = stvg_client()
    rclpy.spin(node)
    rclpy.shutdown()
        
if __name__== "__main__":
    main()
