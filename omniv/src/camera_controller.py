#!/usr/bin/env python3

import rospy
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose, Point, Quaternion
import tf.transformations
import math
import time

class CameraController:
    def __init__(self):
        rospy.init_node('camera_controller', anonymous=True)
        
        # Wait for Gazebo service
        rospy.wait_for_service('/gazebo/set_model_state')
        self.set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        
        # Camera movement parameters
        self.target_pos = [5, 5, 0.5]  # Target object position
        self.radius = 3.0  # Distance from target
        self.height = 0.5  # Camera height
        self.angle = 0.0   # Current angle around target
        
        rospy.loginfo("Camera Controller started")
        
    def move_camera_around_target(self):
        """Move camera in a circle around the target object"""
        rate = rospy.Rate(0.5)  # 0.5 Hz = move every 2 seconds
        
        while not rospy.is_shutdown():
            # Calculate camera position
            x = self.target_pos[0] + self.radius * math.cos(self.angle)
            y = self.target_pos[1] + self.radius * math.sin(self.angle)
            z = self.height
            
            # Calculate orientation to look at target
            yaw = math.atan2(self.target_pos[1] - y, self.target_pos[0] - x)
            pitch = math.atan2(self.target_pos[2] - z, 
                             math.sqrt((self.target_pos[0] - x)**2 + (self.target_pos[1] - y)**2))
            
            # Create quaternion from euler angles
            quaternion = tf.transformations.quaternion_from_euler(0, pitch, yaw)
            
            # Create model state
            model_state = ModelState()
            model_state.model_name = 'sensor_camera'
            model_state.pose = Pose(
                position=Point(x, y, z),
                orientation=Quaternion(*quaternion)
            )
            
            # Set camera position
            try:
                self.set_model_state(model_state)
                rospy.loginfo(f"Camera moved to angle: {math.degrees(self.angle):.1f}°")
            except rospy.ServiceException as e:
                rospy.logerr(f"Service call failed: {e}")
            
            # Update angle for next position
            self.angle += math.pi / 6  # 30 degrees increment
            if self.angle >= 2 * math.pi:
                self.angle = 0.0
                
            rate.sleep()

if __name__ == '__main__':
    try:
        controller = CameraController()
        controller.move_camera_around_target()
    except rospy.ROSInterruptException:
        pass