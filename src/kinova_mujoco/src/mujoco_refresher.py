import numpy as np
import mujoco
from mujoco import viewer
import time
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

class JointControllerNode:
    
    def __init__(self,m,d):
        self.joint_positions = [0]*38
        rospy.init_node('mujoco_fresher_node', anonymous=True)
        self.model=m
        self.data=d
        # 订阅 /jnt_goal 话题
        self.subscriber = rospy.Subscriber("/joint_states", JointState, self.joint_goal_callback)

    def joint_goal_callback(self, msg):
        # 从消息中提取关节坐标
        self.joint_positions = list(msg.position)
        # 将关节坐标传递给 mujoco 控制器
#   <node name="mujoco_moveit_scene_create" pkg="kinova_mujoco" type="mujoco_moveit_planning_scene.py" />

    def run(self):
        rospy.spin()

def main():

    model = mujoco.MjModel.from_xml_path('src/kinova_mujoco/meshes/dual_1.xml')
    data = mujoco.MjData(model)  # 导入模型与数据kinova_mujoco
    node = JointControllerNode(model,data)

    # 控制函数
    def Joint_controller(model, data):
            data.ctrl[:] =  node.joint_positions

    mujoco.set_mjcb_control(Joint_controller)

    # 启动仿真器
    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            mujoco.mj_step(model, data)
            viewer.sync()
            time.sleep(0.01)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass