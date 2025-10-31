import rclpy
from rclpy.node import Node
import time
import math
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from enum import Enum 
import numpy as np 
import random 
from nav_msgs.msg import Odometry


class Walk(Node):
    
    def __init__(self):
        super().__init__('Track')
        self.isStart = False 
        self.startTime = time.time()
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription( 
            LaserScan, 
            '/base_scan', 
            self.sensor_callback, 
            10
        )
        self.subscription = self.create_subscription(
            Odometry,
            '/ground_truth',
            self.listener_callback,
        10)

        self.mode = "FULL"

    def listener_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        if self.isStart == False:
            self.startPos = [x, y]
            self.isStart = True

        dx = x - self.startPos[0]
        dy = y - self.startPos[1]
        distanceTraveled = math.sqrt(dx * dx + dy * dy)

        curTime = time.time()
        elapsed = curTime - self.startTime

        print("Elapsed Time: " + str(elapsed))
        print("Distance: " + str(distanceTraveled))
        
    # Scan the front 180 degrees of lidar range and calculate angular velocity needed to go in the direction of more openness
    def scan_area(self, laser_data):
        # Get the front 180 degrees of lidar range
        laser_ranges = np.array(laser_data.ranges)
        middle_index = len(laser_ranges) // 2
        start_index = middle_index - 90
        end_index = middle_index + 90
        lidar_front_180 = laser_ranges[start_index:end_index]
        
        # Check if we can go straight through a 20-degree cone
        cone_clear = self.check_front_cone_clear(laser_ranges, middle_index)
        
        if cone_clear:
            # Use 20-degree cone for fine adjustments
            self.mode = "CONE"
            angular_velocity, linear_velocity = self.calculate_cone_velocity(laser_ranges, middle_index)
        else:
            # Use 180-degree bias for obstacle avoidance
            self.mode = "FULL"
            angular_velocity, linear_velocity = self.calculate_180_degree_velocity(lidar_front_180)
        
        return angular_velocity, linear_velocity
        
    def check_front_cone_clear(self, laser_ranges, middle_index):
        # Check 20-degree cone in the center (10 degrees on each side)
        cone_start = middle_index - 35
        cone_end = middle_index + 35
        cone_data = laser_ranges[cone_start:cone_end]
        
        # Check if there's clear space (2 meters) in the front cone
        clear_distance = 2.0  # meters
        max_distance_in_cone = np.mean(cone_data)
        
        # Return True if there's at least one clear path (2+ meters) in the cone
        # This allows us to detect doors even if part of the cone faces a wall
        return max_distance_in_cone > clear_distance
        
    def calculate_cone_velocity(self, laser_ranges, middle_index):
        # Get 20-degree cone data (10 degrees on each side of center)
        cone_start = middle_index - 45
        cone_end = middle_index + 45
        cone_data = laser_ranges[cone_start:cone_end]
        
        # Split cone into left and right halves (10 measurements each)
        left_cone = cone_data[:45]    # Left 10 degrees of cone
        right_cone = cone_data[45:]   # Right 10 degrees of cone
        
        # Calculate average distance for each half
        left_openness = np.mean(left_cone)
        right_openness = np.mean(right_cone)
        
        # Calculate bias for fine adjustments
        bias = right_openness - left_openness
        
        # Angular velocity calculation (fine adjustments)
        max_angular_vel = 0.4  # Much smaller than 180-degree version
        scale_factor = 0.1     # Much more gentle for fine adjustments
        if laser_ranges[cone_start] > 0.5 and laser_ranges[cone_end] > 0.5: 
            angular_velocity = bias * random.betavariate(0.3, 0.3) 
        else:  
            angular_velocity = bias * scale_factor 
        
        # Limit the angular velocity for fine adjustments
        if angular_velocity > max_angular_vel:
            angular_velocity = max_angular_vel
        elif angular_velocity < -max_angular_vel:
            angular_velocity = -max_angular_vel
        
        # Linear velocity calculation based on cone openness
        # Use the minimum distance in cone to determine how fast to go
        min_distance_in_cone = np.min(cone_data)
        
        # Base linear velocity
        base_linear_vel = 0.5  # Base speed when cone is clear
        
        # Slow down as we get closer to obstacles
        if min_distance_in_cone < 1.0:
            linear_velocity = 0.3  # Very slow when close to obstacles
        elif min_distance_in_cone < 2.0:
            linear_velocity = 0.5  # Moderate speed when approaching obstacles
        else:
            linear_velocity = base_linear_vel  # Full speed when clear
            
        return angular_velocity, linear_velocity
        
    def calculate_180_degree_velocity(self, front_180_data):
        # Split the 180 degrees into left and right halves
        left_half = front_180_data[:90]    # First 90 measurements (left side)
        right_half = front_180_data[90:]   # Last 90 measurements (right side)
        
        # Calculate average distance for each half (more distance = more openness)
        left_openness = np.mean(left_half)
        right_openness = np.mean(right_half)
        
        # Calculate bias: positive means turn right, negative means turn left
        bias = right_openness - left_openness
        
        # Angular velocity calculation
        max_angular_vel = 1.0  # Maximum angular velocity
        scale_factor = 0.5      # How sensitive the turning is
        angular_velocity = bias * scale_factor
        
        # Limit the angular velocity to prevent excessive turning
        if angular_velocity > max_angular_vel:
            angular_velocity = max_angular_vel
        elif angular_velocity < -max_angular_vel:
            angular_velocity = -max_angular_vel
        
        # Linear velocity calculation based on overall openness
        min_distance = np.min(front_180_data)
        
        # Base linear velocity
        base_linear_vel = 0.8  # Base speed for 180-degree mode
        
        # Adjust speed based on obstacles
        if min_distance < 0.4:
            linear_velocity = 0.0  # Stop if too close to obstacles
            angular_velocity = 1.0
        elif min_distance < 1.0:
            linear_velocity = 0.2  # Very slow when close
        elif min_distance < 2.0:
            linear_velocity = 0.5  # Moderate speed
        else:
            linear_velocity = base_linear_vel  # Full speed when clear
            
        return angular_velocity, linear_velocity

    def sensor_callback(self, msg):
        # Get lidar data and calculate velocities using our scan_area function
        angular_velocity, linear_velocity = self.scan_area(msg)
        
        # Create and send twist command
        twist_msg = Twist()
        twist_msg.linear.x = linear_velocity
        twist_msg.angular.z = angular_velocity
        
        # Publish the twist command
        self.publisher.publish(twist_msg)
        
        # Optional: Print debug information
        print(f"Mode: {self.mode} \t | \t Linear: {linear_velocity:.2f} \t | \t Angular: {angular_velocity:.2f} \t | \t Time: {time.time() - self.startTime:.2f}")

def main(args=None):
    rclpy.init(args=args)
    Walk_Node = Walk()
    rclpy.spin(Walk_Node)
    Walk_Node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
