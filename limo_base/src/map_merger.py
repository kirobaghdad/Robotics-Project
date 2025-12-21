#!/usr/bin/env python3
import rospy
import sys
import numpy as np
import cv2
import os
import tf2_ros
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Bool
from geometry_msgs.msg import TransformStamped

class MapMerger:
    def __init__(self, num_robots):
        rospy.init_node('map_merger_node')
        self.num_robots = num_robots
        self.robot_maps = {}
        self.prev_unknown_count = None
        self.stable_count = 0
        self.mapping_complete = False
        self.tf_broadcaster = tf2_ros.StaticTransformBroadcaster()
        
        for i in range(num_robots):
            robot_name = f'robot_{i}'
            self.robot_maps[robot_name] = None
            rospy.Subscriber(f'/{robot_name}/map', OccupancyGrid, self.map_callback, robot_name)
        
        self.merged_pub = rospy.Publisher('/map', OccupancyGrid, queue_size=10)
        self.complete_pub = rospy.Publisher('/mapping_complete', Bool, queue_size=1)
        
        # Publish static transforms from map to robot_X/map
        self.publish_static_transforms()
        
        rospy.loginfo(f"Map merger initialized for {num_robots} robots")

    def publish_static_transforms(self):
        transforms = []
        for i in range(self.num_robots):
            t = TransformStamped()
            t.header.stamp = rospy.Time.now()
            t.header.frame_id = "map"
            t.child_frame_id = f"robot_{i}/map"
            t.transform.translation.x = 0.0
            t.transform.translation.y = 0.0
            t.transform.translation.z = 0.0
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = 0.0
            t.transform.rotation.w = 1.0
            transforms.append(t)
        
        self.tf_broadcaster.sendTransform(transforms)

    def map_callback(self, msg, robot_name):
        self.robot_maps[robot_name] = msg

    def merge_maps(self):
        available_maps = [m for m in self.robot_maps.values() if m is not None]
        if not available_maps:
            return None
        
        merged = OccupancyGrid()
        merged.header.stamp = rospy.Time.now()
        merged.header.frame_id = "map"
        merged.info = available_maps[0].info
        
        data_arrays = [np.array(m.data, dtype=np.int8) for m in available_maps]
        merged_data = np.maximum.reduce(data_arrays)
        merged.data = merged_data.tolist()
        
        return merged

    def check_mapping_complete(self, merged_map):
        if merged_map is None:
            return False
        
        data = np.array(merged_map.data, dtype=np.int8)
        unknown_count = np.sum(data == -1)
        total_cells = len(data)
        unknown_ratio = unknown_count / total_cells
        
        if self.prev_unknown_count is not None:
            change = abs(unknown_count - self.prev_unknown_count)
            if change < 50 and unknown_ratio < 0.15:
                self.stable_count += 1
            else:
                self.stable_count = 0
        
        self.prev_unknown_count = unknown_count
        
        if self.stable_count >= 10:
            return True
        return False

    def save_map_image(self, merged_map):
        data = np.array(merged_map.data, dtype=np.int8)
        width = merged_map.info.width
        height = merged_map.info.height
        
        map_array = data.reshape((height, width))
        img = np.zeros((height, width), dtype=np.uint8)
        img[map_array == -1] = 127  # Unknown = gray
        img[map_array == 0] = 255   # Free = white
        img[map_array == 100] = 0   # Occupied = black
        
        img = cv2.flip(img, 0)
        
        output_dir = os.path.expanduser('~/catkin_ws/src/Robotics_Project')
        output_path = os.path.join(output_dir, 'final_map.png')
        cv2.imwrite(output_path, img)
        rospy.loginfo(f"Final map saved to: {output_path}")
        return output_path

    def run(self):
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            merged_map = self.merge_maps()
            if merged_map:
                self.merged_pub.publish(merged_map)
                
                if not self.mapping_complete and self.check_mapping_complete(merged_map):
                    self.mapping_complete = True
                    rospy.loginfo("\n" + "="*50)
                    rospy.loginfo("MAPPING COMPLETE!")
                    rospy.loginfo("="*50)
                    
                    map_path = self.save_map_image(merged_map)
                    rospy.loginfo(f"Map image saved: {map_path}")
                    
                    self.complete_pub.publish(Bool(data=True))
                    rospy.signal_shutdown("Mapping complete")
                    break
            
            rate.sleep()

if __name__ == '__main__':
    if len(sys.argv) < 2:
        rospy.logerr("Usage: map_merger.py <num_robots>")
        sys.exit(1)
    
    num_robots = int(sys.argv[1])
    try:
        merger = MapMerger(num_robots)
        merger.run()
    except rospy.ROSInterruptException:
        pass
