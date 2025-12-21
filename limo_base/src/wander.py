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
        random.seed(hash(robot_name) + int(rospy.Time.now().to_sec() * 1000))
        
        self.robot_name = robot_name
        self.cmd_topic = f'/{robot_name}/cmd_vel'
        self.scan_topic = f'/{robot_name}/scan'
        
        self.linear_speed = random.uniform(0.6, 1.0)
        self.angular_speed = random.uniform(0.8, 1.2)
        self.mapping_complete = False
        self.scan_data = None
        self.stuck_counter = 0
        self.exploration_bias = random.uniform(-0.3, 0.3)
        self.last_position_check = rospy.Time.now()

        self.pub = rospy.Publisher(self.cmd_topic, Twist, queue_size=10)
        self.sub = rospy.Subscriber(self.scan_topic, LaserScan, self.scan_callback)
        rospy.Subscriber('/mapping_complete', Bool, self.complete_callback)
        
        self.move_cmd = Twist()

    def complete_callback(self, msg):
        if msg.data:
            self.mapping_complete = True
            rospy.loginfo(f"{self.robot_name}: Received mapping complete signal, stopping...")

    def scan_callback(self, data):
        self.scan_data = data

    def find_best_direction(self):
        if not self.scan_data:
            return 0.0
        
        ranges = self.scan_data.ranges
        n = len(ranges)
        sectors = 12
        sector_size = n // sectors
        sector_scores = []
        
        for i in range(sectors):
            start = i * sector_size
            end = start + sector_size
            sector_ranges = [r for r in ranges[start:end] if 0.1 < r < 10.0]
            
            if sector_ranges:
                avg_dist = sum(sector_ranges) / len(sector_ranges)
                min_dist = min(sector_ranges)
                
                if avg_dist > 3.0:
                    score = avg_dist * 0.2 + min_dist * 0.3
                else:
                    score = avg_dist * 0.8 + min_dist * 0.5
                
                score += random.uniform(-0.3, 0.3)
            else:
                score = 0.0
            
            sector_scores.append(score)
        
        top_sectors = sorted(range(len(sector_scores)), key=lambda i: sector_scores[i], reverse=True)[:3]
        best_sector = random.choice(top_sectors)
        
        angle_per_sector = 2 * 3.14159 / sectors
        target_angle = (best_sector * angle_per_sector) - 3.14159 + random.uniform(-0.3, 0.3)
        
        return target_angle

    def run(self):
        rate = rospy.Rate(10)
        
        while not rospy.is_shutdown():
            if self.mapping_complete:
                self.move_cmd.linear.x = 0
                self.move_cmd.angular.z = 0
                self.pub.publish(self.move_cmd)
                break
            
            if not self.scan_data:
                rate.sleep()
                continue
            
            ranges = self.scan_data.ranges
            mid = len(ranges) // 2
            front_ranges = [r for r in ranges[mid-30:mid+30] if r > 0.1]
            left_ranges = [r for r in ranges[mid+30:mid+90] if r > 0.1]
            right_ranges = [r for r in ranges[mid-90:mid-30] if r > 0.1]
            
            min_front = min(front_ranges) if front_ranges else 10.0
            min_left = min(left_ranges) if left_ranges else 10.0
            min_right = min(right_ranges) if right_ranges else 10.0
            
            if min_front < 0.5 or (min_left < 0.4 and min_right < 0.4):
                self.stuck_counter += 1
            else:
                self.stuck_counter = max(0, self.stuck_counter - 1)
            
            if self.stuck_counter > 15:
                self.move_cmd.linear.x = -0.3
                self.move_cmd.angular.z = random.choice([-1.2, 1.2])
                self.stuck_counter = 0
            elif min_front < 0.7:
                self.move_cmd.linear.x = 0.0
                target_angle = self.find_best_direction()
                self.move_cmd.angular.z = max(-self.angular_speed, min(self.angular_speed, target_angle * 1.2))
            else:
                target_angle = self.find_best_direction()
                self.move_cmd.linear.x = self.linear_speed
                self.move_cmd.angular.z = max(-0.6, min(0.6, target_angle * 0.5 + self.exploration_bias))
            
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
