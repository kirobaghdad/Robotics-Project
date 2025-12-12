#!/usr/bin/env python3

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import os
import sys

# Add parent directory to path to import visual_search
sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..'))
from omniv.src.visual_search import VisualSearch

class VisualSearchNode:
    def __init__(self):
        rospy.init_node('visual_search_node', anonymous=True)
        
        self.bridge = CvBridge()
        
        # Load target image
        target_path = os.path.join(os.path.dirname(__file__), '..', 'images')
        
        if not os.path.exists(target_path):
            rospy.logerr(f"Target image not found: {target_path}")
            rospy.logerr("Please run image_picker first to capture target object")
            return
            
        self.visual_search = VisualSearch(target_path)
        
        # Subscribe to sensor camera
        self.image_sub = rospy.Subscriber('/omniv/sensor_camera/image_raw', Image, self.search_callback)
        
        rospy.loginfo("Visual Search Node started")
        
    def search_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            
            # Perform visual search
            found, similarity_img = self.visual_search.search_object(cv_image)
            
            if found:
                rospy.loginfo("TARGET OBJECT FOUND!")
            else:
                rospy.loginfo("Searching...")
                
            # Display camera frame
            cv2.imshow("Camera Feed", cv_image)
            
            # Display similarity visualization
            if similarity_img is not None:
                cv2.imshow("Visual Search Result", similarity_img)
            
            cv2.waitKey(1)
                
        except Exception as e:
            rospy.logerr(f"Error: {e}")

if __name__ == '__main__':
    try:
        node = VisualSearchNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass