import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from moveit_commander import PlanningSceneInterface
from tf.transformations import quaternion_from_euler
import shape_msgs.msg

def main():
    # 初始化ROS节点
    rospy.init_node("moveit_multi_gripper_example", anonymous=True)
    moveit_commander.roscpp_initialize("")

    # 定义机械臂和左夹爪的MoveGroup接口
    move_group = moveit_commander.MoveGroupCommander("left_arm")
    left_gripper_group = moveit_commander.MoveGroupCommander("left_hand")
    planning_scene_interface = PlanningSceneInterface()

    # 创建碰撞物体并添加到场景
    collision_object = moveit_msgs.msg.CollisionObject()
    collision_object.header.frame_id = move_group.get_planning_frame()
    collision_object.id = "target_object"

    # 设置物体的形状（盒子）和位置
    primitive = shape_msgs.msg.SolidPrimitive()
    primitive.type = primitive.CYLINDER
    primitive.dimensions = [0.04, 0.04]
    # primitive.dimensions[0] = 0.04  # 长度
    # primitive.dimensions[1] = 0.04  # 宽度
    

    # 设置物体的位置
    cylinder_pose = geometry_msgs.msg.Pose()
    cylinder_pose.orientation.w = 1.0
    cylinder_pose.position.x = 0.5
    cylinder_pose.position.y = 0.0
    cylinder_pose.position.z = 1

    # 将物体添加到碰撞对象中
    collision_object.primitives = [primitive]
    collision_object.primitive_poses = [cylinder_pose]
    collision_object.operation = collision_object.ADD

    # 应用碰撞物体到规划场景
    planning_scene_interface.add_object(collision_object)

    # 定义抓取动作
    grasp = moveit_msgs.msg.Grasp()
    grasp.grasp_pose.header.frame_id = "world"

    # 设置抓取的姿态（方向）
    q = quaternion_from_euler(0, 0, 0)
    grasp.grasp_pose.pose.orientation.x = q[0]
    grasp.grasp_pose.pose.orientation.y = q[1]
    grasp.grasp_pose.pose.orientation.z = q[2]
    grasp.grasp_pose.pose.orientation.w = q[3]
    grasp.grasp_pose.pose.position.x = 0.5
    grasp.grasp_pose.pose.position.y = 0.0
    grasp.grasp_pose.pose.position.z = 1.25

    # 使用“open”姿势作为预抓取姿势
    # left_gripper_group.set_named_target("Open")  # "open"为Setup Assistant中定义的张开姿势
    # success,pre_grasp_posture = left_gripper_group.plan()  # 获取预抓取的轨迹
    # if success: 
    #     grasp.pre_grasp_posture = pre_grasp_posture.trajectory  # 将预抓取轨迹赋值给 grasp
    # else :
    #     print("Planning failed.")
    # 使用“close”姿势作为抓取姿势
    # left_gripper_group.set_named_target("Close")  # "close"为Setup Assistant中定义的闭合姿势
    # grasp_posture = left_gripper_group.plan()  # 获取抓取的轨迹
    # print(grasp_posture)
    # if success:   
    #     grasp.grasp_posture = grasp_posture.trajectory  # 将抓取轨迹赋值给 grasp
    # else :
    #     print("Planning failed.")
    # 设置前抓取和后抓取的路径
    grasp.pre_grasp_approach.direction.header.frame_id = "world"
    grasp.pre_grasp_approach.direction.vector.x = 1.0
    grasp.pre_grasp_approach.min_distance = 0.0095
    grasp.pre_grasp_approach.desired_distance = 0.00115

    grasp.post_grasp_retreat.direction.header.frame_id = "world"
    grasp.post_grasp_retreat.direction.vector.z = 1.0
    grasp.post_grasp_retreat.min_distance = 0.010
    grasp.post_grasp_retreat.desired_distance = 0.0025

    # 执行抓取
    grasps = [grasp]

    # 尝试抓取目标物体
    try:
        move_group.pick("target_object", grasps)
        rospy.loginfo("抓取成功！")
    except Exception as e:
        rospy.logerr("抓取失败: %s", e)

if __name__ == "__main__":
    main()
