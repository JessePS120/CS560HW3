import rclpy
from rclpy.node import Node
import time
import math
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from enum import Enum 
import numpy as np 
import random 

robot_velocity = 0.5

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
        self.target_distance = 0.15
        self.left_right_threshold = 1.5
        self.lzr_cone_size = 55
        self.front_cone_size = 30

        # PID Controller initialization
        # Tips for Tuning:
            # Increase Kp until there is a good response to the wall from the robot, there may be some oscillation
            # Reduce Kp by 50% for stability after that
            # Then, start with small values of Ki (<0.1) and increase until there is no steady-state error
            # If there is overshoot, reduce Ki by 50%
            # Then, start with small values for Kd (<0.5) and increase until the oscillations are dampened
            # If it seems sluggish, reduce Kp by 50%
        self.target_distance = 0.3  # Target distance from wall (meters)
        self.Kp = 2.0  # Proportional gain
        self.Ki = 0.1  # Integral gain  
        self.Kd = 0.5  # Derivative gain
        # PID state variables
        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_time = time.time()
        # Angular velocity limits
        self.max_angular_vel = 0.3 # Maximum angular velocity (rad/s)
        # Anti-windup integral limit:
        self.integral_limit = 2.0

    def move(self, x, z): 
        twist = Twist() 
        twist.linear.x = float(x) 
        twist.angular.z = float(z) 
        self.publisher.publish(twist) 

    def turn_cw(self, time_len): 
        # start_time = time.time()
        # while time.time()-start_time < time_len:
        #     self.move(0, -(math.pi/2 / time_len))
        self.move(.05, -100)
    
    def turn_ccw(self, time_len): 
        # start_time = time.time()
        # while time.time()-start_time < time_len:
        #     self.move(0, (math.pi/2 / time_len))
        self.move(.05, 100)
    
    #Returns a boolean array ([Left, Right, Front], right_dist) where 1 indicates a wall 
    #and 0 indicates nothing and dist is the distance from the wall on the right. 
    def find_walls(self, lzr_msg):
        cone_dif = self.lzr_cone_size // 2
        front_cone_dif = self.front_cone_size // 2
        lzr_dist = np.array(lzr_msg.ranges)
        mid = len(lzr_dist) // 2
        lzr_front = lzr_dist[mid-front_cone_dif:mid+front_cone_dif]
        lzr_left = lzr_dist[0:cone_dif]
        lzr_right = lzr_dist[len(lzr_dist)-cone_dif-1:len(lzr_dist)-1]

        print(min(lzr_left), min(lzr_right), min(lzr_front))

        walls = [min(lzr_left)<self.left_right_threshold, min(lzr_right)<self.left_right_threshold, min(lzr_front)<self.target_distance]
        right_dist = 5
        if walls[1]:
            right_dist = min(lzr_right)

        print(walls, right_dist)

        return walls, right_dist

    # PID Controller for wall following
    # Input: error - distance from target wall on the right side (meters)
    # Output: angular velocity (rad/s) to adjust robot position
    def pid(self, dist):
        # Calculate time step
        current_time = time.time()
        dt = current_time - self.prev_time
        
        # Avoid division by zero on first call
        if dt == 0:
            dt = 0.01
            
        # Calculate error from target distance
        # Positive error means too far from wall, negative means too close
        distance_error = dist - self.target_distance
        
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
        
        return angular_velocity * -1.0
    
    def sensor_callback(self, msg): 
        #TODO: Consider adding a timeout if not right wall has been found that causes the robot to move randomly. 
        #TODO: We can escape islands by recording the movement pattern of the robot. if this pattern happens over and over again 
        #we can use the left hand rule instead. 
        walls, dist = self.find_walls(msg)
        #No wall on the right and no wall in front. 
        if walls[1] == 0 and walls[2] == 0: 
            self.turn_cw(math.pi / 2 / robot_velocity)
        #Wall in front of the robot
        elif walls[2] == 1: 
            self.turn_ccw(math.pi / 2 / robot_velocity)
        #Right wall is present but no front wall. 
        else: 
            self.move(robot_velocity, self.pid(dist))


def main(args=None):
    rclpy.init(args=args)
    Walk_Node = Walk()
    rclpy.spin(Walk_Node)
    Walk_Node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()



''''''
#                                                    ___
#                                               ,----'   `-,
#                      ___,-'~~~`---'~~~~`-----' ,-'        \,
#              ___,---'          '        ,-~~~~~            :
#         _,--'                 ,        ; ;       ) "   __   \,
#    _,--'     ,                 ,'      :: "  ;  ` ,'  (\o\)  |
#   / _,       `,                     ,  `; " ;    (     `~~ `'\
#  ; /         ,`               ,     `    ~~~`. " ;   _     ,  `.
# ,'/          ` ,              `     ` ,  ,    \_/ ?   ;    )   `.
# :;:            `                      `  ` ,     uu`--(___(    ~:
# :::          , ,  ,            ,   ;     , `  ,-'      \~(  ~   :
# ||:          ` `  `         ,  ` ,'    ( ` _,-          \ \_   ~:
# :|`.        , ,  ,          `_   ;       ) );            \__>   :
# |:  : ;     ` `  ` ;  __,--~~ `-(         ( |              `.  ~|
# :|  :         ` __,--'           :  ()    : :               |~  ;
# |;  |  `     ,-'    ;             :  )(   | `.         /(   `. ~|
# ::  :   ~  _/;     ;               |   )  :  :        ; /    ;~ ;
# {}  ;     /  :   ~ :               :      ; ,;        : `._,-  ,
# {} /~    /   ;    ;                 : ,  |  `;         `.___,-'
#   ;~    ;    ;  ~ `.                | `  )   ;
#   :`    \    `;~   \                ;~   `-, `-.     Targon
#   `.__OOO;    `;_OOO;               )_____OO;_(O)
#    ~~~~~~       ~~~~                ~~~~~~~~ ~~~~ ''''''

#        ,,,,,,,,,,,,,,,
#     ,(((((((((())))))))),
#   ,((((((((((()))))))))))),
#  ,(((((((((\\\|///))))))))),
# ,((((((((((///|\\\)))))))))),
# ((((((((//////^\\\\\\))))))))
# ((((((' .-""-   -""-. '))))))
# (((((  `\.-.     .-./`  )))))
# ((((( -=(0) )   (0) )=- )))))
# '((((   /'-'     '-'\   ))))'
#  ((((\   _,   A  ,_    /))))
#  '((((\    \     /    /))))'
#    '((('.   `-o-'   .')))'
# jgs      '-.,___,.-'
