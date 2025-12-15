#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np
import random

class AutonomousNav:
    def __init__(self):
        rospy.init_node('autonomous_nav')
        
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        
        self.linear_speed = 0.3
        self.angular_speed = 0.5
        self.safe_distance = 1.5
        self.laser_data = None
        self.state = 'forward'
        self.turn_direction = 1
        
        rospy.loginfo("Autonomous navigation started")

    def laser_callback(self, msg):
        self.laser_data = msg

    def get_min_distance_in_sector(self, ranges, start_angle, end_angle):
        if not ranges:
            return float('inf')
        
        start_idx = int((start_angle - self.laser_data.angle_min) / self.laser_data.angle_increment)
        
        end_idx = int((end_angle - self.laser_data.angle_min) / self.laser_data.angle_increment)
        
        start_idx = max(0, min(start_idx, len(ranges)-1))
        end_idx = max(0, min(end_idx, len(ranges)-1))
        
        sector = ranges[start_idx:end_idx]
        valid = [r for r in sector if self.laser_data.range_min < r < self.laser_data.range_max]
        
        return min(valid) if valid else float('inf')

    def explore(self):
        twist = Twist()
        
        if self.laser_data is None:
            return
        
        front = self.get_min_distance_in_sector(self.laser_data.ranges, -0.3, 0.3)
        left = self.get_min_distance_in_sector(self.laser_data.ranges, 0.3, 1.5)
        right = self.get_min_distance_in_sector(self.laser_data.ranges, -1.5, -0.3)
        
        if front < self.safe_distance:
            self.state = 'turn'
            self.turn_direction = 1 if left > right else -1
        
        if self.state == 'forward':
            twist.linear.x = -1 * self.linear_speed
            twist.angular.z = 0
            
            if random.random() < 0.01:
                self.state = 'turn'
                self.turn_direction = random.choice([-1, 1])
        
        elif self.state == 'turn':
            twist.linear.x = 0
            twist.angular.z = self.angular_speed * self.turn_direction
            
            if front > self.safe_distance * 1.5:
                self.state = 'forward'
        
        self.pub.publish(twist)

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.explore()
            rate.sleep()

if __name__ == '__main__':
    try:
        nav = AutonomousNav()
        nav.run()
    except rospy.ROSInterruptException:
        pass
