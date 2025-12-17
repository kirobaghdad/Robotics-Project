#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose, Point, Quaternion
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs

class MappingNode:
    def __init__(self):
        rospy.init_node('mapping_node')
        
        # Map parameters
        self.map_resolution = 0.5  # meters per pixel
        self.map_width = 200  # pixels
        self.map_height = 200  # pixels
        
        # Initialize occupancy grid
        self.occupancy_grid = np.full((self.map_height, self.map_width), -1, dtype=np.int8)
        
        # Map origin (center of grid)
        self.map_origin_x = -self.map_width * self.map_resolution / 2
        self.map_origin_y = -self.map_height * self.map_resolution / 2
        
        # TF setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer)
        
        # Publishers and subscribers
        self.map_pub = rospy.Publisher('/map', OccupancyGrid, queue_size=1)
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        
        rospy.loginfo("Mapping node initialized")

    def world_to_grid(self, x, y):
        """Convert world coordinates to grid coordinates"""
        grid_x = int((x - self.map_origin_x) / self.map_resolution)
        grid_y = int((y - self.map_origin_y) / self.map_resolution)
        return grid_x, grid_y

    def laser_callback(self, laser_msg):
        try:
            # Get robot pose in odom frame
            transform = self.tf_buffer.lookup_transform('odom', 'base_footprint', rospy.Time())
            robot_x = transform.transform.translation.x
            robot_y = transform.transform.translation.y
            
            # Get robot orientation
            quat = transform.transform.rotation
            yaw = np.arctan2(2.0 * (quat.w * quat.z + quat.x * quat.y),
                           1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z))
            
            # Process laser scan
            angle = laser_msg.angle_min
            for i, range_val in enumerate(laser_msg.ranges):
                if laser_msg.range_min <= range_val <= laser_msg.range_max:
                    # Calculate obstacle position
                    obstacle_x = robot_x + range_val * np.cos(yaw + angle)
                    obstacle_y = robot_y + range_val * np.sin(yaw + angle)
                    
                    # Mark obstacle in grid
                    grid_x, grid_y = self.world_to_grid(obstacle_x, obstacle_y)
                    if 0 <= grid_x < self.map_width and 0 <= grid_y < self.map_height:
                        self.occupancy_grid[grid_y, grid_x] = 50  # Occupied
                    
                    # Mark free space along ray
                    self.mark_free_space(robot_x, robot_y, obstacle_x, obstacle_y)
                
                angle += laser_msg.angle_increment
            
            self.publish_map()
            
        except Exception as e:
            rospy.logwarn(f"Failed to process laser scan: {e}")

    def mark_free_space(self, start_x, start_y, end_x, end_y):
        """Mark free space along a ray using Bresenham's line algorithm"""
        start_gx, start_gy = self.world_to_grid(start_x, start_y)
        end_gx, end_gy = self.world_to_grid(end_x, end_y)
        
        # Bresenham's line algorithm
        dx = abs(end_gx - start_gx)
        dy = abs(end_gy - start_gy)
        sx = 1 if start_gx < end_gx else -1
        sy = 1 if start_gy < end_gy else -1
        err = dx - dy
        
        x, y = start_gx, start_gy
        
        while True:
            if 0 <= x < self.map_width and 0 <= y < self.map_height:
                if self.occupancy_grid[y, x] == -1:  # Only mark unknown cells as free
                    self.occupancy_grid[y, x] = 0  # Free space
            
            if x == end_gx and y == end_gy:
                break
                
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy

    def publish_map(self):
        """Publish the occupancy grid map"""
        map_msg = OccupancyGrid()
        map_msg.header.stamp = rospy.Time.now()
        map_msg.header.frame_id = "odom"
        
        # Map metadata
        map_msg.info.resolution = self.map_resolution
        map_msg.info.width = self.map_width
        map_msg.info.height = self.map_height
        map_msg.info.origin.position.x = self.map_origin_x
        map_msg.info.origin.position.y = self.map_origin_y
        map_msg.info.origin.orientation.w = 1.0
        
        # Flatten and publish map data
        map_msg.data = self.occupancy_grid.flatten().tolist()
        self.map_pub.publish(map_msg)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        mapper = MappingNode()
        mapper.run()
    except rospy.ROSInterruptException:
        pass