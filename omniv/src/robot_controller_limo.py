#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty, EmptyResponse
import math

class RobotControllerLIMO:
    def __init__(self):
        rospy.init_node('robot_controller_limo')
        
        # Robot state from odometry
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        
        # Publisher for velocity commands
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        # Subscribe to odometry
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        
        rospy.loginfo("LIMO Robot controller started")
        
    def odom_callback(self, msg):
        # Update robot position from odometry
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        
        # Extract yaw from quaternion
        orientation = msg.pose.pose.orientation
        siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)
    
    def move(self, linear_x=0.0, linear_y=0.0, angular_z=0.0):
        """Send velocity command to LIMO"""
        cmd = Twist()
        cmd.linear.x = linear_x
        cmd.linear.y = linear_y
        cmd.angular.z = angular_z
        self.cmd_pub.publish(cmd)
    
    def stop(self):
        """Stop the robot"""
        self.move(0.0, 0.0, 0.0)

if __name__ == '__main__':
    try:
        controller = RobotControllerLIMO()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
