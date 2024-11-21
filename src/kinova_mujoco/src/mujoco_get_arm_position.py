#!/usr/bin/env python

import rospy
import moveit_commander
import geometry_msgs.msg
import sys
def get_current_arm_pose():
    # 初始化 ROS 节点
    rospy.init_node('get_current_pose', anonymous=True)

    # 初始化 MoveIt 接口
    moveit_commander.roscpp_initialize(sys.argv)
    
    # 创建 MoveGroupCommander 对象，用于控制机械臂（假设组名为 "arm"）
    left_arm_group = moveit_commander.MoveGroupCommander("left_arm")


    # 获取当前的姿态
    left_current_pose = left_arm_group.get_current_pose().pose

    # 打印当前位置
    rospy.loginfo("Current Position:")
    rospy.loginfo("x: {}".format(left_current_pose.position.x))
    rospy.loginfo("y: {}".format(left_current_pose.position.y))
    rospy.loginfo("z: {}".format(left_current_pose.position.z))

    # 打印当前方向
    rospy.loginfo("Current Orientation:")
    rospy.loginfo("x: {}".format(left_current_pose.orientation.x))
    rospy.loginfo("y: {}".format(left_current_pose.orientation.y))
    rospy.loginfo("z: {}".format(left_current_pose.orientation.z))
    rospy.loginfo("w: {}".format(left_current_pose.orientation.w))

    right_arm_group = moveit_commander.MoveGroupCommander("right_arm")


    # 获取当前的姿态
    right_current_pose = right_arm_group.get_current_pose().pose

    # 打印当前位置
    rospy.loginfo("Current Position:")
    rospy.loginfo("x: {}".format(right_current_pose.position.x))
    rospy.loginfo("y: {}".format(right_current_pose.position.y))
    rospy.loginfo("z: {}".format(right_current_pose.position.z))

    # 打印当前方向
    rospy.loginfo("Current Orientation:")
    rospy.loginfo("x: {}".format(right_current_pose.orientation.x))
    rospy.loginfo("y: {}".format(right_current_pose.orientation.y))
    rospy.loginfo("z: {}".format(right_current_pose.orientation.z))
    rospy.loginfo("w: {}".format(right_current_pose.orientation.w))
    rospy.signal_shutdown("Shutting down the node.")

if __name__ == "__main__":
    get_current_arm_pose()
