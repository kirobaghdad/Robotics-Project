#!/usr/bin/env python3

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import os

class ImagePicker:
    def __init__(self):
        rospy.init_node('image_picker', anonymous=True)
        
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/omniv/sensor_camera/image_raw', Image, self.image_callback)
        
        self.save_path = os.path.join(os.path.dirname(__file__), '..', 'images')
        os.makedirs(self.save_path, exist_ok=True)
        
        rospy.loginfo("Image Picker started. Press 's' to save target image, 'q' to quit")
        
    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            
            cv2.imshow("Target Object Picker - Press 's' to save, 'q' to quit", cv_image)
            
            key = cv2.waitKey(1) & 0xFF
            if key == ord('s'):
                filename = os.path.join(self.save_path, 'target_object.jpg')
                cv2.imwrite(filename, cv_image)
                rospy.loginfo(f"Target image saved: {filename}")
                
            elif key == ord('q'):
                rospy.signal_shutdown("User quit")
                
        except Exception as e:
            rospy.logerr(f"Error: {e}")

if __name__ == '__main__':
    try:
        picker = ImagePicker()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()