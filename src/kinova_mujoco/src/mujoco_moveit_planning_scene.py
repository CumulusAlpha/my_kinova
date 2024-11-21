#!/usr/bin/env python
import rospy
import moveit_commander
from moveit_commander import PlanningSceneInterface
from geometry_msgs.msg import PoseStamped
import sys


def add_boxes():
    # 初始化ros节点
    rospy.init_node('add_boxes_to_scene', anonymous=True)
    
    # 初始化MoveIt接口
    moveit_commander.roscpp_initialize(sys.argv)
    
    # 初始化规划场景
    scene = PlanningSceneInterface()
    
    # 给MoveIt一点时间来初始化场景
    rospy.sleep(2)

    # 设置盒子的尺寸
    box_size1 = [1, 2, 1]  

    # 设置第一个盒子的位姿
    box1_pose = PoseStamped()
    box1_pose.header.frame_id = "world"  # 设置参考坐标系
    box1_pose.pose.position.x = 0.80
    box1_pose.pose.position.y = 0
    box1_pose.pose.position.z = 0.5
    box1_pose.pose.orientation.w = 1.0

    # 添加第一个盒子到规划场景中
    box1_name = "desk"
    scene.add_box(box1_name, box1_pose, size=box_size1)

    # 设置第二个盒子的位姿
    box2_pose = PoseStamped()
    box2_pose.header.frame_id = "world"  # 设置参考坐标系
    box2_pose.pose.position.x = 0 
    box2_pose.pose.position.y = 0
    box2_pose.pose.position.z = 1.7/2
    box2_pose.pose.orientation.w = 1.0

    box_size2 = [0.5, 0.5, 1.7]
    # 添加第二个盒子到规划场景中
    box2_name = "body"
    # scene.add_box(box2_name, box2_pose, size=box_size2)

    # 等待盒子加入场景
    rospy.sleep(2)

    # # 检查盒子是否成功加入场景
    # if box1_name in scene.get_known_object_names() and box2_name in scene.get_known_object_names():
    #     rospy.loginfo("Both boxes added to the planning scene!")
    # else:
    #     rospy.logwarn("Failed to add one or both boxes to the planning scene.")

    # 可选：删除盒子
    # scene.remove_world_object(box1_name)
    # scene.remove_world_object(box2_name)

if __name__ == "__main__":
    try:
        add_boxes()
    except rospy.ROSInterruptException:
        pass
