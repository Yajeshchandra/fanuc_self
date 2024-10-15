#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
import geometry_msgs.msg
from sensor_msgs.msg import Joy

class JoystickControlledArm:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('joystick_controlled_arm', anonymous=True)
        
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group_name = "manipulator"
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name)
        
        self.move_group.set_goal_position_tolerance(0.05)
        self.move_group.set_goal_orientation_tolerance(0.05)
        self.move_group.set_goal_joint_tolerance(0.01)
        
        self.joy_sub = rospy.Subscriber("joy", Joy, self.joy_callback)
        self.rate = rospy.Rate(10)  # 10 Hz
        
        self.velocity_scale = 0.1  # Adjust this to change the speed of movement

    def joy_callback(self, data):
        # Assuming right stick X-axis is at index 3
        x_velocity = data.axes[3] * self.velocity_scale
        
        # Get current pose
        current_pose = self.move_group.get_current_pose().pose
        
        # Calculate new position
        new_x = current_pose.position.x + x_velocity
        
        # Set the new position
        self.set_end_effector_position(new_x, current_pose.position.y, current_pose.position.z)

    def set_end_effector_position(self, x, y, z):
        waypoints = []
        
        # Get the current pose and modify only the X position
        current_pose = self.move_group.get_current_pose().pose
        target_pose = current_pose
        target_pose.position.x = x
        
        waypoints.append(target_pose)
        
        (plan, fraction) = self.move_group.compute_cartesian_path(
            waypoints,   # waypoints to follow
            0.01,        # eef_step
            0.0)         # jump_threshold
        
        self.move_group.execute(plan, wait=True)

    def run(self):
        while not rospy.is_shutdown():
            self.rate.sleep()

if __name__ == "__main__":
    try:
        controller = JoystickControlledArm()
        controller.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        moveit_commander.roscpp_shutdown()