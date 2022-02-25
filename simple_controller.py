#! /usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry 
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2

#starting coordinates
x = 0.0
y = 0.0
theta = 0.0

def newOdom(msg):
    global x 
    global y 
    global theta 
    
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    
    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion ([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
    

rospy.init_node ("speed_controller") 

sub = rospy.Subscriber("/odom", Odometry, newOdom)
pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

speed = Twist()

r = rospy.Rate(2)

print("Input number of goals:")
n = int(input())

while n > 0: 
    goal = Point ()
    print("Input x coordinate:")
    goal.x = float(input())
    print("Input y coordinate:")
    goal.y = float(input())

    while goal.x - x > 0.02 and goal.y - y > 0.02:
        inc_x = goal.x - x 
        inc_y = goal.y - y 
    
        angle_to_goal = atan2(inc_y, inc_x)
    
        if abs(angle_to_goal - theta) > 0.1:
            if (angle_to_goal - theta) < 0.0:	
                speed.linear.x = 0.0
                speed.angular.z = 0.3
            
            elif (angle_to_goal - theta) > 0.0:
                speed.linear.x = 0.0
                speed.angular.z = -0.3
        else: 
            speed.linear.x = 0.5
            speed.angular.z = 0.0
    
        pub.publish(speed)
        r.sleep()
    
    speed.linear.x = 0.0
    speed.angular.z = 0.0
    pub.publish(speed)
    n -= 1