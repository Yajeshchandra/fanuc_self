#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
from geometry_msgs.msg import Twist,Pose
from sensor_msgs.msg import Joy

# Global variables to store joystick input and the MoveIt! commander object
move_group = None
joystick_input = None

def initialize_moveit():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_joystick_control', anonymous=True)
    
    # Initialize MoveIt! Commander
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_name = "manipulator"
    global move_group
    move_group = moveit_commander.MoveGroupCommander(group_name)

    # Set tolerances for motion
    move_group.set_goal_position_tolerance(0.05)  # 1 cm tolerance
    move_group.set_goal_orientation_tolerance(0.05)  # ~2.8 degrees tolerance
    move_group.set_goal_joint_tolerance(0.01)  # ~0.57 degrees tolerance
    rospy.loginfo("MoveIt! initialized.")
    
def joystick_callback(joy_msg):
    """
    Callback to update joystick input. We use this to modify the desired end-effector velocity.
    """
    global joystick_input
    joystick_input = joy_msg

def set_end_effector_velocity():
    """
    Moves the end-effector based on joystick input.
    """
    global joystick_input

    if joystick_input is None:
        return  # No joystick input yet, return

    # Create a Twist message to control the end-effector velocity
    twist = Twist()
    
    # Map the joystick axes to velocity (modify as per your joystick configuration)
    twist.linear.x = joystick_input.axes[0] * 0.1  # Joystick input scaled for x-axis velocity
    twist.linear.y = joystick_input.axes[1] * 0.1  # y-axis velocity
    twist.linear.z = 0.0  # Assuming no vertical movement for now
    
    twist.angular.x = 0.0  # No rotation along x
    twist.angular.y = 0.0  # No rotation along y
    twist.angular.z = joystick_input.axes[2] * 0.1  # Rotation around z-axis
    
    # Ensure minimal movement threshold to avoid jitter
    if abs(twist.linear.x) < 0.01 and abs(twist.linear.y) < 0.01 and abs(twist.angular.z) < 0.01:
        return  # No significant movement, so we don't update the robot's state

    # Get the current end-effector pose and add the velocity
    current_pose = move_group.get_current_pose().pose
    rospy.loginfo(f"Current pose: {current_pose}")

    # Update target position based on joystick input (integrating velocity)
    target_pose = Pose()
    target_pose.orientation = current_pose.orientation  # Keep current orientation
    target_pose.position.x = current_pose.position.x + twist.linear.x
    target_pose.position.y = current_pose.position.y + twist.linear.y
    target_pose.position.z = current_pose.position.z + twist.linear.z

    # Set the new target pose for the end-effector
    move_group.set_pose_reference_frame("base_link")
    move_group.set_end_effector_link("link_6")
    move_group.set_num_planning_attempts(10)
    move_group.set_planning_time(5)
    move_group.set_pose_target(target_pose, "link_6")

    # Plan and execute the motion
    plan = move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()

    rospy.loginfo("Motion executed.")

if __name__ == "__main__":
    try:
        # Initialize MoveIt! and the joystick subscriber
        initialize_moveit()

        # Subscribe to the joystick topic
        rospy.Subscriber("/joy", Joy, joystick_callback)
        rospy.loginfo("Joystick subscriber initialized.")

        # Main control loop
        rate = rospy.Rate(10)  # 10 Hz loop rate
        while not rospy.is_shutdown():
            # Continuously check joystick input and move the end-effector accordingly
            set_end_effector_velocity()
            rate.sleep()

        moveit_commander.roscpp_shutdown()

    except rospy.ROSInterruptException:
        pass
