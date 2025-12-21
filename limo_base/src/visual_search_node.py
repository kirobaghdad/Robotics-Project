#!/usr/bin/env python3

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import os
import sys

# Add current directory to path to import visual_search
sys.path.insert(0, os.path.dirname(__file__))
from visual_search import VisualSearch

class VisualSearchNode:
    def __init__(self, robot_name):
        rospy.init_node(f'{robot_name}_visual_search_node', anonymous=True)
        
        self.robot_name = robot_name
        self.bridge = CvBridge()
        self.object_found = False
        self.found_image = None
        
        # Load target image
        target_path = os.path.join(os.path.dirname(__file__), '..', 'images')
        
        if not os.path.exists(target_path):
            rospy.logerr(f"Target image not found: {target_path}")
            rospy.logerr("Please run image_picker first to capture target object")
            return
            
        self.visual_search = VisualSearch(target_path)
        
        # Subscribe to robot-specific camera topic
        camera_topic = f'/{robot_name}/camera_ir/color/image_raw'
        self.image_sub = rospy.Subscriber(camera_topic, Image, self.search_callback)
        
        rospy.loginfo(f"Visual Search Node started for {robot_name} on topic {camera_topic}")
        
    def search_callback(self, data):
        if self.object_found:
            return
            
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            
            # Perform visual search
            found, similarity_img = self.visual_search.search_object(cv_image)
            
            if found:
                rospy.loginfo(f"{self.robot_name}: TARGET OBJECT FOUND! Search concluded.")
                self.object_found = True
                self.found_image = similarity_img if similarity_img is not None else cv_image
                self.image_sub.unregister()
                cv2.destroyAllWindows()
                self.show_result()
            else:
                rospy.loginfo(f"{self.robot_name}: Searching...")

            # if similarity_img is not None:
            #     cv2.imshow("Visual Search Result", similarity_img)
            
            cv2.waitKey(1)
                
        except Exception as e:
            rospy.logerr(f"Error: {e}")
    
    def show_result(self):
        """Display the found object image and wait for user to close it"""
        cv2.imshow("OBJECT FOUND - Press any key to exit", self.found_image)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        rospy.loginfo("User closed result window. Shutting down node...")
        rospy.signal_shutdown("Search completed")

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: visual_search_node.py <robot_name>")
        print("Example: visual_search_node.py robot_0")
        sys.exit(1)
    
    robot_name = sys.argv[1]
    try:
        node = VisualSearchNode(robot_name)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass