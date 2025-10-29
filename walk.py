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

        # PID Controller initialization
        # Tips for Tuning:
            # Increase Kp until there is a good response to the wall from the robot, there may be some oscillation
            # Reduce Kp by 50% for stability after that
            # Then, start with small values of Ki (<0.1) and increase until there is no steady-state error
            # If there is overshoot, reduce Ki by 50%
            # Then, start with small values for Kd (<0.5) and increase until the oscillations are dampened
            # If it seems sluggish, reduce Kp by 50%
        self.target_distance = 0.5  # Target distance from wall (meters)
        self.Kp = 2.0  # Proportional gain
        self.Ki = 0.1  # Integral gain  
        self.Kd = 0.5  # Derivative gain
        # PID state variables
        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_time = time.time()
        # Angular velocity limits
        self.max_angular_vel = 1.0  # Maximum angular velocity (rad/s)
        # Anti-windup integral limit:
        self.integral_limit = 2.0

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
    
    # PID Controller for wall following
    # Input: error - distance from target wall on the right side (meters)
    # Output: angular velocity (rad/s) to adjust robot position
    def pid(self, error):
        # Calculate time step
        current_time = time.time()
        dt = current_time - self.prev_time
        
        # Avoid division by zero on first call
        if dt == 0:
            dt = 0.01
            
        # Calculate error from target distance
        # Positive error means too far from wall, negative means too close
        distance_error = error - self.target_distance
        
        # Proportional term: immediate response to current error
        proportional = self.Kp * distance_error
        
        # Integral term: accumulates error over time to eliminate steady-state error
        self.integral += distance_error * dt
        
        # Prevent integral windup by limiting the integral term
        integral_limit = self.integral_limit
        if self.integral > integral_limit:
            self.integral = integral_limit
        elif self.integral < -integral_limit:
            self.integral = -integral_limit
            
        integral_term = self.Ki * self.integral
        
        # Derivative term: rate of change of error (damping)
        derivative = (distance_error - self.prev_error) / dt
        derivative_term = self.Kd * derivative
        
        # Calculate PID output (angular velocity)
        # We will need to tune this because the error does not map directly to the angular velocity. We may need to limit the max angular velocity or scale it up/down.
        angular_velocity = proportional + integral_term + derivative_term
        
        # Limit angular velocity to prevent excessive turning
        if angular_velocity > self.max_angular_vel:
            angular_velocity = self.max_angular_vel
        elif angular_velocity < -self.max_angular_vel:
            angular_velocity = -self.max_angular_vel
        
        # Update state variables for next iteration
        self.prev_error = distance_error
        self.prev_time = current_time
        
        # Debug output (optional - remove in production)
        print(f"PID Debug - Error: {distance_error:.3f}, P: {proportional:.3f}, I: {integral_term:.3f}, D: {derivative_term:.3f}, Output: {angular_velocity:.3f}")
        
        return angular_velocity
    
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