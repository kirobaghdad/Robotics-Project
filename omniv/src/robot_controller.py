#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose, Point, Quaternion
import tf.transformations
import math

class RobotController:
    def __init__(self):
        rospy.init_node('robot_controller')
        
        # Wait for Gazebo service
        rospy.wait_for_service('/gazebo/set_model_state')
        self.set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        
        # Robot state
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        
        # Subscribe to cmd_vel
        self.cmd_sub = rospy.Subscriber('/cmd_vel', Twist, self.cmd_callback)
        
        rospy.loginfo("Robot controller started")
        
    def cmd_callback(self, msg):
        # Get velocities
        linear_vel_x = msg.linear.x  # forward/backward
        linear_vel_y = msg.linear.y  # left/right
        angular_vel = msg.angular.z  # rotation
        
        # Update robot pose (simple integration)
        dt = 0.1  # 10Hz update rate
        # Transform velocities from robot frame to world frame
        world_vel_x = linear_vel_x * math.cos(self.yaw) - linear_vel_y * math.sin(self.yaw)
        world_vel_y = linear_vel_x * math.sin(self.yaw) + linear_vel_y * math.cos(self.yaw)
        
        self.x += world_vel_x * dt
        self.y += world_vel_y * dt
        self.yaw += angular_vel * dt
        
        # Create quaternion from yaw
        quaternion = tf.transformations.quaternion_from_euler(0, 0, self.yaw)
        
        # Create model state
        model_state = ModelState()
        model_state.model_name = 'fancy_robot'
        model_state.pose = Pose(
            position=Point(self.x, self.y, 0),
            orientation=Quaternion(*quaternion)
        )
        
        # Set robot position
        try:
            self.set_model_state(model_state)
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

if __name__ == '__main__':
    try:
        controller = RobotController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass