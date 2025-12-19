#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class AutoWander:
    def __init__(self):
        rospy.init_node('wander_node')
        
        # --- CONFIGURATION ---
        # If your robot doesn't move, check these topic names!
        self.cmd_topic = '/cmd_vel'  # Might need to be '/limo/cmd_vel'
        self.scan_topic = '/limo/scan'    # Might need to be '/limo/scan'
        
        self.min_dist_to_wall = 1  # Stop if wall is closer than 0.6 meters
        self.linear_speed = 1
        self.angular_speed = 0.5
        # ---------------------

        self.pub = rospy.Publisher(self.cmd_topic, Twist, queue_size=10)
        self.sub = rospy.Subscriber(self.scan_topic, LaserScan, self.scan_callback)
        self.move_cmd = Twist()
        self.obstacle_detected = False

    def scan_callback(self, data):
        # The laser scanner gives us an array of distances.
        # We check the middle slice (front of robot) for obstacles.
        
        # Assume the scan array goes from Right -> Front -> Left
        # We take a slice from the middle of the array
        mid_index = len(data.ranges) // 2
        window_size = 40  # Check 20 rays on left and 20 on right of center
        
        # Get the minimum distance in the front cone
        # We filter out '0.0' or 'inf' which can mean "no return" or "too close"
        valid_ranges = [r for r in data.ranges[mid_index-window_size : mid_index+window_size] if r > 0.05]
        
        if len(valid_ranges) > 0:
            min_front_dist = min(valid_ranges)
        else:
            min_front_dist = 10.0 # Clear path if no valid data

        if min_front_dist < self.min_dist_to_wall:
            self.obstacle_detected = True
        else:
            self.obstacle_detected = False

    def run(self):
        rate = rospy.Rate(10) # 10 Hz
        while not rospy.is_shutdown():
            if self.obstacle_detected:
                # Wall ahead! Turn left.
                self.move_cmd.linear.x = 0.0
                self.move_cmd.angular.z = self.angular_speed
                rospy.loginfo("Obstacle! Turning...")
            else:
                # Path clear! Go forward.
                self.move_cmd.linear.x = self.linear_speed
                self.move_cmd.angular.z = 0.0
                
            self.pub.publish(self.move_cmd)
            rate.sleep()

if __name__ == '__main__':
    try:
        wanderer = AutoWander()
        wanderer.run()
    except rospy.ROSInterruptException:
        pass