#!/usr/bin/env python3
import rospy
import random
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class AutoWander:
    def __init__(self):
        rospy.init_node('wander_node')
        
        # --- CONFIGURATION ---
        # If your robot doesn't move, check these topic names!
        self.cmd_topic = '/cmd_vel'  # Might need to be '/limo/cmd_vel'
        self.scan_topic = '/scan'    # Might need to be '/scan'
        
        # TODO: account for the real size of the world on deployment
        self.min_dist_to_wall = 1  # Stop if wall is closer than 0.6 meters
        self.linear_speed = 1
        self.angular_speed = 1
        # ---------------------

        self.pub = rospy.Publisher(self.cmd_topic, Twist, queue_size=10)
        self.sub = rospy.Subscriber(self.scan_topic, LaserScan, self.scan_callback)
        self.move_cmd = Twist()
        self.obstacle_detected = False
        self.turn_direction = 1

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
        
        # State management for random exploration
        # States: "FORWARD", "RANDOM_TURN"
        state = "FORWARD"
        # Initialize next change time. Wait for valid time if using sim time.
        while rospy.Time.now().is_zero() and not rospy.is_shutdown():
            rate.sleep()
        next_state_change = rospy.Time.now() + rospy.Duration(random.uniform(3.0, 6.0))

        while not rospy.is_shutdown():
            if self.obstacle_detected:
                # Wall ahead!
                # Decelerate smoothly to avoid pitching
                self.move_cmd.linear.x = max(0.0, self.move_cmd.linear.x - 0.2)
                
                # If we were going straight, pick a new random turn direction
                if self.move_cmd.angular.z == 0.0:
                    self.turn_direction = 1 if random.choice([True, False]) else -1
                self.move_cmd.angular.z = self.angular_speed * self.turn_direction
                
                # Reset random wander timer so we don't interrupt recovery
                state = "FORWARD"
                next_state_change = rospy.Time.now() + rospy.Duration(random.uniform(3.0, 6.0))
            else:
                # Path clear!
                if state == "FORWARD":
                    self.move_cmd.linear.x = min(self.linear_speed, self.move_cmd.linear.x + 0.3)
                    self.move_cmd.angular.z = 0.0
                    
                    if rospy.Time.now() > next_state_change:
                        state = "RANDOM_TURN"
                        next_state_change = rospy.Time.now() + rospy.Duration(random.uniform(0.5, 1.5))
                        self.turn_direction = 1 if random.choice([True, False]) else -1
                
                elif state == "RANDOM_TURN":
                    self.move_cmd.linear.x = max(0.0, self.move_cmd.linear.x - 0.3)
                    self.move_cmd.angular.z = self.angular_speed * self.turn_direction
                    
                    if rospy.Time.now() > next_state_change:
                        state = "FORWARD"
                        next_state_change = rospy.Time.now() + rospy.Duration(random.uniform(8.0, 15.0))
                
            self.pub.publish(self.move_cmd)
            rate.sleep()

if __name__ == '__main__':
    try:
        wanderer = AutoWander()
        wanderer.run()
    except rospy.ROSInterruptException:
        pass