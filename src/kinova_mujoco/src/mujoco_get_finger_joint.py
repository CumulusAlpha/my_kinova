import rospy
from moveit_commander import MoveGroupCommander, roscpp_initialize, roscpp_shutdown

def get_finger_joint_angles():
    # 初始化各个手指的MoveGroupCommander
    left_F1  = MoveGroupCommander("left_F1")
    left_F2  = MoveGroupCommander("left_F2")
    left_F3  = MoveGroupCommander("left_F3")
    right_F1  = MoveGroupCommander("right_F1")
    right_F2  = MoveGroupCommander("right_F2")
    right_F3  = MoveGroupCommander("right_F3")

    # 获取当前关节角度
    left_F1_angles = left_F1 .get_current_joint_values()
    left_F2_angles = left_F2 .get_current_joint_values()
    left_F3_angles = left_F3 .get_current_joint_values()
    right_F1_angles = right_F1 .get_current_joint_values()
    right_F2_angles = right_F2 .get_current_joint_values()
    right_F3_angles = right_F3 .get_current_joint_values()

    # 打印每个手指的关节角度
    rospy.loginfo("Left F1 joint angles: %s", left_F1_angles)
    rospy.loginfo("Left F2 joint angles: %s", left_F2_angles)
    rospy.loginfo("Left F3 joint angles: %s", left_F3_angles)
    rospy.loginfo("Right F1 joint angles: %s", right_F1_angles)
    rospy.loginfo("Right F2 joint angles: %s", right_F2_angles)
    rospy.loginfo("Right F3 joint angles: %s", right_F3_angles)

if __name__ == "__main__":
    roscpp_initialize([])
    rospy.init_node("get_finger_joint_angles", anonymous=True)

    # 调用获取关节角度的函数
    get_finger_joint_angles()

    roscpp_shutdown()
