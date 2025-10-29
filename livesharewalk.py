import rclpy
from rclpy.node import Node
import time
import math
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from enum import Enum 
import numpy as np 
import random 

#Spin rate of the robot. 
robot_spin_velo = 0.5
robot_forward_speed = 1 
sleep_time = 1 

class Walk(Node):
    
    def __init__(self):
        super().__init__('Track')
        self.startTime = time.time()
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription( 
            LaserScan, 
            '/base_scan', 
            self.sensor_callback, 
            10
        )
    
    def send(self, x=0, z=0):
        twist_msg = Twist()
        twist_msg.linear.x = float(x)  
        twist_msg.angular.z = float(z)   
        self.publisher.publish(twist_msg)

    def stop(self): 
        self.send(0,0)
        time.sleep(sleep_time) #stop for 1 second

    def rotate_CW(self, time=0):
        time_start = time.time() 
        while(time.time() < time_start + time): 
            self.send(0,-robot_spin_velo)
            print("Rotate CW")
        time.sleep(sleep_time) 

    def rotate_CCW(self, time=0):
        time_start = time.time()
        while(time.time() < time_start + time):
            self.send(0,robot_spin_velo)
            print("Rotate CCW")
        time.sleep(sleep_time)

    #Just in case we need it. 
    def move_backward(self): 

    #Returns a boolean array ([Left, Right, Front], right_dist) where 1 indicates a wall 
    #and 0 indicates nothing and dist is the distance from the wall on the right. 
    def find_wall(self):


    #Error = Smallest Robot distance from wall that we are tracking(right wall). 
    # Error is the distance from the wall. We want to get as close to the wall on the right side as possible.
    # Output angular velocity 
    def pid(self, error): 
        integral += error * dt
        derivative = (error - prev_error) / dt
        output = Kp * error + Ki * integral + Kd * derivative
        prev_error = error

    def sensor_callback(self, msg): 

def main(args=None):
    rclpy.init(args=args)
    Walk_Node = Walk()
    rclpy.spin(Walk_Node)
    Walk_Node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()





'''
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
'''  
