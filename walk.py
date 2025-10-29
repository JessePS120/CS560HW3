import rclpy
from rclpy.node import Node
import time
import math
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from enum import Enum 
import numpy as np 
import random 

class Robot_State(Enum): 
    OBAV = 1 
    DRIVE = 2 
    SCAN = 3 

STATE =  Robot_State.SCAN
#Spin rate of the robot. 
robot_spin_velo = 0.5

class Walk(Node):
    def __init__(self):
        super().__init__('Track')
        self.startTime = time.time()
        #Variables used for the find_door function. 
        self.last_spin_complete = 0 
        self.time_spinning = 0 
        self.spinning = False 
        self.random_spin = 1.0 
        #End find_door function variables. 
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription( 
            LaserScan, 
            '/base_scan', 
            self.sensor_callback, 
            10
        )

    def find_door(self, laser_data, force_spin):
        #The number of seconds between each spin. 
        spin_rate = 5  
        #How long the robot needs to spin for. 
        spin_time = (2 * math.pi) / robot_spin_velo 
        edge = 0.2
        max_offset = 1.0
        min_door_width = 0.5
        max_door_width = 2.5 
        door_found = False 

        if force_spin or (time.time() - self.last_spin_complete > spin_rate): 
            if not self.spinning: 
                self.random_spin = random.choice([-1, 1]) 
                self.spinning = True 
                self.time_spinning = time.time() 
            
            #For Debugging 
            twist = Twist() 
            twist.linear.x = 0.0 
            twist.angular.z = self.random_spin * robot_spin_velo  
            self.publisher.publish(twist)
            #End Debugging 

            #Convert lidar data into a more usable form(np arrays) 
            laser_ranges = np.array(laser_data.ranges)
            angles = laser_data.angle_min + np.arange(len(laser_ranges)) * laser_data.angle_increment
            #Masking data so that we only use the lidar directly in front of the robot. 
            half_cone = np.deg2rad(30)
            mask = (angles >= -half_cone) & (angles <= half_cone)
            laser_ranges = laser_ranges[mask]
            angles = angles[mask]
            #Calculating the difference array. 
            range_difference = np.diff(laser_ranges) 
            #Detect any "edges"(where there is a large enough range difference for there to be a door) 
            edges = np.where(np.abs(range_difference) > edge)[0] 


            for i in range(len(edges) - 1): 
                left_index = edges[i] 
                right_index = edges[i + 1] 
                middle_index = (left_index + right_index) // 2 

                #Check if there is an open space between the two edges of the doorway. 
                if laser_ranges[middle_index] > max(laser_ranges[left_index], laser_ranges[right_index]) and (abs(laser_ranges[left_index] - laser_ranges[right_index]) <= max_offset): 
                    left_x = laser_ranges[left_index] * math.cos(angles[left_index])
                    left_y = laser_ranges[left_index] * math.sin(angles[left_index])
                    right_x = laser_ranges[right_index] * math.cos(angles[right_index]) 
                    right_y = laser_ranges[right_index] * math.sin(angles[right_index]) 
                    door_width = math.sqrt((left_x - right_x )**2 + (left_y - right_y)**2)

                    if min_door_width <= door_width <= max_door_width: 
                        print("Door Found At", laser_ranges[middle_index] * math.cos(angles[middle_index]), laser_ranges[middle_index] * math.sin(angles[middle_index]))
                        door_found = True 

        #Spin has been completed 
        if time.time() - self.time_spinning >= spin_time or door_found: 
            global STATE 
            STATE = Robot_State.DRIVE 
            self.last_spin_complete = time.time() 
            self.spinning = False 

            #For Debugging 
            twist = Twist() 
            twist.linear.x = 0.0  
            twist.angular.z = 0.0  
            self.publisher.publish(twist)
            #End Debugging 
    
    def sensor_callback(self, msg): 
        if STATE == Robot_State.SCAN: 
            self.find_door(msg, False) 

    
def main(args=None):
    rclpy.init(args=args)
    Walk_Node = Walk()
    rclpy.spin(Walk_Node)
    Walk_Node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()