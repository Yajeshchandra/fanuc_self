#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
import geometry_msgs.msg

def initialize_moveit():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface', anonymous=True)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_name = "manipulator"
    move_group = moveit_commander.MoveGroupCommander(group_name)
    move_group.set_goal_position_tolerance(0.05)  # 1 cm tolerance
    move_group.set_goal_orientation_tolerance(0.05)  # 0.05 radians tolerance (~2.8 degrees)
    move_group.set_goal_joint_tolerance(0.01)  # 0.01 radians tolerance (~0.57 degrees)
    return move_group

def set_end_effector_position(move_group, x, y, z):
    # try:
        # Set pose target for a specific link (link_6)
        current_pose = move_group.get_current_pose().pose
        rospy.loginfo(f"Current pose: {current_pose}")
        target_pose = geometry_msgs.msg.Pose()
        target_pose.orientation.w = 1.0  # Default orientation, modify as needed
        target_pose.position.x = x
        target_pose.position.y = y
        target_pose.position.z = z

        move_group.set_pose_reference_frame("base_link")
        move_group.set_end_effector_link("link_6")
        move_group.set_num_planning_attempts(10)
        move_group.set_planning_time(5)
        move_group.set_pose_target(target_pose, "link_6")

        # Plan and execute
        plan = move_group.go(wait=True)
        move_group.stop()
        move_group.clear_pose_targets()

        rospy.loginfo("Planning successful! Executing the plan...")
        # move_group.execute(plan, wait=True)
        # move_group.stop()
        # move_group.clear_pose_targets()

    # except Exception as e:
    #     rospy.logerr(f"An error occurred: {e}")

if __name__ == "__main__":
    move_group = initialize_moveit()
    set_end_effector_position(move_group, 1.0, 1.2, 1.2)  # Example target position
    moveit_commander.roscpp_shutdown()
