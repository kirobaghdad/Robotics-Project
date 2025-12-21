#!/usr/bin/env python3
import rospy
import random
import sys
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

class AutoWander:
    def __init__(self, robot_name):
        rospy.init_node(f'{robot_name}_wander_node', anonymous=False)
        
        self.robot_name = robot_name
        self.cmd_topic = f'/{robot_name}/cmd_vel'
        self.scan_topic = f'/{robot_name}/scan'
        
        self.min_dist_to_wall = 1
        self.linear_speed = 0.5
        self.angular_speed = 0.6
        self.mapping_complete = False

        self.pub = rospy.Publisher(self.cmd_topic, Twist, queue_size=10)
        self.sub = rospy.Subscriber(self.scan_topic, LaserScan, self.scan_callback)
        rospy.Subscriber('/mapping_complete', Bool, self.complete_callback)
        
        self.move_cmd = Twist()
        self.obstacle_detected = False
        self.turn_direction = 1

    def complete_callback(self, msg):
        if msg.data:
            self.mapping_complete = True
            rospy.loginfo(f"{self.robot_name}: Received mapping complete signal, stopping...")

    def scan_callback(self, data):
        mid_index = len(data.ranges) // 2
        window_size = 40
        valid_ranges = [r for r in data.ranges[mid_index-window_size : mid_index+window_size] if r > 0.05]
        
        if len(valid_ranges) > 0:
            min_front_dist = min(valid_ranges)
        else:
            min_front_dist = 10.0

        self.obstacle_detected = min_front_dist < self.min_dist_to_wall

    def run(self):
        rate = rospy.Rate(10)
        state = "FORWARD"
        
        while rospy.Time.now().is_zero() and not rospy.is_shutdown():
            rate.sleep()
        next_state_change = rospy.Time.now() + rospy.Duration(random.uniform(3.0, 6.0))

        while not rospy.is_shutdown():
            if self.mapping_complete:
                self.move_cmd.linear.x = 0
                self.move_cmd.angular.z = 0
                self.pub.publish(self.move_cmd)
                rospy.loginfo(f"{self.robot_name}: Stopped due to mapping completion")
                break
                
            if self.obstacle_detected:
                self.move_cmd.linear.x = max(0.0, self.move_cmd.linear.x - 0.1)
                if self.move_cmd.angular.z == 0.0:
                    self.turn_direction = 1 if random.choice([True, False]) else -1
                self.move_cmd.angular.z = self.angular_speed * self.turn_direction
                state = "FORWARD"
                next_state_change = rospy.Time.now() + rospy.Duration(random.uniform(3.0, 6.0))
            else:
                if state == "FORWARD":
                    self.move_cmd.linear.x = min(self.linear_speed, self.move_cmd.linear.x + 0.1)
                    self.move_cmd.angular.z = 0.0
                    if rospy.Time.now() > next_state_change:
                        state = "RANDOM_TURN"
                        next_state_change = rospy.Time.now() + rospy.Duration(random.uniform(0.5, 1.5))
                        self.turn_direction = 1 if random.choice([True, False]) else -1
                elif state == "RANDOM_TURN":
                    self.move_cmd.linear.x = max(0.2, self.move_cmd.linear.x - 0.1)
                    self.move_cmd.angular.z = self.angular_speed * self.turn_direction
                    if rospy.Time.now() > next_state_change:
                        state = "FORWARD"
                        next_state_change = rospy.Time.now() + rospy.Duration(random.uniform(8.0, 15.0))
                
            self.pub.publish(self.move_cmd)
            rate.sleep()

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: wander.py <robot_name>")
        print("Example: wander.py robot_0")
        sys.exit(1)
    
    robot_name = sys.argv[1]
    try:
        wanderer = AutoWander(robot_name)
        wanderer.run()
    except rospy.ROSInterruptException:
        pass
