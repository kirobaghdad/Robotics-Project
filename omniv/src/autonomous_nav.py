#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, Pose
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from gazebo_msgs.srv import SpawnModel, DeleteModel, GetWorldProperties
from std_srvs.srv import Empty, EmptyResponse
import numpy as np
import random
import subprocess
import os
from concurrent.futures import ThreadPoolExecutor

class AutonomousNav:
    def __init__(self):
        rospy.init_node('autonomous_nav')

        
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.path_pub = rospy.Publisher('/path_trace', Marker, queue_size=10)
        # self.clear_service = rospy.Service('/clear_path', Empty, self.clear_path_callback)
        
        
        self.clear_path()
        
        self.linear_speed = 3  # Realistic speed for small robot
        self.angular_speed = 1.0  # Realistic turning speed
        self.safe_distance = 0.5  # Appropriate safe distance
        
        self.laser_data = None
        self.state = 'forward'
        self.turn_direction = 1
        self.path_points = []
        self.last_pos = None
        self.marker_count = 0
        
        self.min_distance = 0.2
        
        # Stuck detection
        self.position_history = []
        self.stuck_threshold = 0.5  # If moved less than this in 2 seconds
        self.stuck_time_window = 2.0  # seconds
        
        # # Delete existing markers on startup
        # self.delete_existing_markers()
        
        rospy.loginfo("Autonomous navigation started")

    def delete_existing_markers(self):
        """Delete all existing path markers from Gazebo"""
        try:
            get_world = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)
            world_props = get_world()
            delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
            
            markers = [m for m in world_props.model_names if m.startswith('path_marker_')]
            
            def delete(name):
                try:
                    delete_model(name)
                except:
                    pass
            
            with ThreadPoolExecutor(max_workers=10) as executor:
                executor.map(delete, markers)
        except:
            pass
    

    def laser_callback(self, msg):
        self.laser_data = msg
    
    def odom_callback(self, msg):
        pos = msg.pose.pose.position
        current_time = rospy.Time.now().to_sec()
        
        # Track position for stuck detection
        self.position_history.append((current_time, pos.x, pos.y))
        self.position_history = [(t, x, y) for t, x, y in self.position_history 
                                  if current_time - t < self.stuck_time_window]
        
        if self.last_pos is None:
            self.last_pos = pos
            # self.spawn_marker(pos)
            return
        
        dist = np.sqrt((pos.x - self.last_pos.x)**2 + (pos.y - self.last_pos.y)**2)
        
        if dist >= self.min_distance:
            # self.spawn_marker(pos)
            self.last_pos = pos
        
        point = Point()
        point.x = pos.x
        point.y = pos.y
        point.z = pos.z
        self.path_points.append(point)
        self.publish_path()
    
    def spawn_marker(self, position):
        try:
            spawn_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
            model_xml = f'''<?xml version='1.0'?>
            <sdf version='1.6'>
              <model name='path_marker_{self.marker_count}'>
                <static>true</static>
                <link name='link'>
                  <visual name='visual'>
                    <geometry>
                      <sphere><radius>0.05</radius></sphere>
                    </geometry>
                    <material>
                      <ambient>1 0 0 1</ambient>
                      <diffuse>1 0 0 1</diffuse>
                    </material>
                  </visual>
                </link>
              </model>
            </sdf>'''
            pose = Pose()
            pose.position.x = position.x
            pose.position.y = position.y
            pose.position.z = 0.15
            spawn_model(f'path_marker_{self.marker_count}', model_xml, '', pose, 'world')
            self.marker_count += 1
        except:
            pass
    
    def clear_path(self):
        """Service callback to clear the path visualization"""
        self.path_points = []
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = rospy.Time.now()
        marker.action = Marker.DELETEALL
        self.path_pub.publish(marker)
        rospy.loginfo("Path cleared")
        return EmptyResponse()
    
    def publish_path(self):
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = rospy.Time.now()
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.id = 0
        marker.scale.x = 0.1
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.points = self.path_points
        self.path_pub.publish(marker)

    def is_stuck(self):
        if len(self.position_history) < 2:
            return False
        
        oldest = self.position_history[0]
        newest = self.position_history[-1]
        
        distance_moved = np.sqrt((newest[1] - oldest[1])**2 + (newest[2] - oldest[2])**2)
        
        return distance_moved < self.stuck_threshold
    
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
        
        # Check if stuck in corner
        if self.is_stuck():
            # Reverse and turn sharply
            twist.linear.x = -self.linear_speed * 0.5
            twist.angular.z = self.angular_speed * random.choice([-1, 1])
            self.pub.publish(twist)
            rospy.sleep(1.0)
            self.position_history.clear()
            return
        
        front = self.get_min_distance_in_sector(self.laser_data.ranges, -0.3, 0.3)
        left = self.get_min_distance_in_sector(self.laser_data.ranges, 0.3, 1.5)
        right = self.get_min_distance_in_sector(self.laser_data.ranges, -1.5, -0.3)
        
        if front < self.safe_distance:
            self.state = 'turn'
            self.turn_direction = 1 if left > right else -1
        
        if self.state == 'forward':
            twist.linear.x = self.linear_speed
            twist.angular.z = 0
            
            if random.random() < 0.1:  # Less random turning = faster exploration
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
