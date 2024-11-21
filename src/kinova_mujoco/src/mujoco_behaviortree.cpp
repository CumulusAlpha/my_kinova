#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <ros/ros.h>
#include <behaviortree_cpp/bt_factory.h>

// 节点 1: 张开夹爪
class OpenGripper : public BT::SyncActionNode {
public:
  OpenGripper(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts() {
    return {};
  }

  BT::NodeStatus tick() override {
    ROS_INFO("Opening gripper...");
    // 假设使用 MoveIt 控制夹爪
    moveit::planning_interface::MoveGroupInterface gripper_group("left_hand");
    gripper_group.setNamedTarget("Open");  // 假设有一个 "open" 的预定义状态
    bool success = (gripper_group.move() == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (success) {
      ROS_INFO("Gripper opened");
      return BT::NodeStatus::SUCCESS;
    } else {
      ROS_ERROR("Failed to open gripper");
      return BT::NodeStatus::FAILURE;
    }
  }
};


// class MoveToHome : public BT::SyncActionNode {
// public:
//   MoveToPosition(const std::string& name, const BT::NodeConfiguration& config)
//     : BT::SyncActionNode(name, config) {}

//   static BT::PortsList providedPorts() {
//     return {};
//   }

//   BT::NodeStatus tick() override {
//     ROS_INFO("Moving to Home");
//     moveit::planning_interface::MoveGroupInterface arm_group("left_arm");
//     arm_group.setNamedTarget("Home");  // 使用预定义的位置
//     bool success = (arm_group.move() == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//     if (success) {
//       ROS_INFO("Reached Home");
//       return BT::NodeStatus::SUCCESS;
//     } else {
//       ROS_ERROR("Failed to reach Home");
//       return BT::NodeStatus::FAILURE;
//     }
//   }
// };
class MoveToHome : public BT::SyncActionNode {
public:
  // 构造函数
  MoveToHome(const std::string& name, const BT::NodeConfiguration& config)
      : BT::SyncActionNode(name, config) {}

  // 提供的端口（此处无输入端口）
  static BT::PortsList providedPorts() {
    return {};
  }

  // 行为树执行的主要逻辑
  BT::NodeStatus tick() override {
    ROS_INFO("Moving to Home");
    moveit::planning_interface::MoveGroupInterface arm_group("left_arm");

    // 设置预定义位置为 "Home"
    arm_group.setNamedTarget("Home");

    // 执行移动
    bool success = (arm_group.move() == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (success) {
      ROS_INFO("Reached Home");
      return BT::NodeStatus::SUCCESS;
    } else {
      ROS_ERROR("Failed to reach Home");
      return BT::NodeStatus::FAILURE;
    }
  }
};

class MoveToPosition : public BT::SyncActionNode {
public:
  MoveToPosition(const std::string& name, const BT::NodeConfiguration& config)
      : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<double>("x"),
      BT::InputPort<double>("y"),
      BT::InputPort<double>("z"),
      BT::InputPort<double>("qx"),
      BT::InputPort<double>("qy"),
      BT::InputPort<double>("qz"),
      BT::InputPort<double>("qw")
    };
  }

  BT::NodeStatus tick() override {
    double x, y, z, qx, qy, qz, qw;
    if (!getInput("x", x) || 
        !getInput("y", y) || 
        !getInput("z", z) || 
        !getInput("qx", qx) || 
        !getInput("qy", qy) || 
        !getInput("qz", qz) || 
        !getInput("qw", qw)) {
      ROS_ERROR("Failed to get target position or orientation inputs");
      return BT::NodeStatus::FAILURE;
    }

    // 输出目标位姿
    ROS_INFO("Moving to position: [x: %f, y: %f, z: %f, qx: %f, qy: %f, qz: %f, qw: %f]", 
             x, y, z, qx, qy, qz, qw);

    moveit::planning_interface::MoveGroupInterface arm_group("left_arm");

    // 设置目标位姿
    geometry_msgs::Pose target_pose;
    target_pose.position.x = x;
    target_pose.position.y = y;
    target_pose.position.z = z;
    target_pose.orientation.x = qx;
    target_pose.orientation.y = qy;
    target_pose.orientation.z = qz;
    target_pose.orientation.w = qw;
    arm_group.setPoseTarget(target_pose);

    // 执行移动
    bool success = (arm_group.move() == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (success) {
      ROS_INFO("Reached target position successfully");
      return BT::NodeStatus::SUCCESS;
    } else {
      ROS_ERROR("Failed to reach target position");
      return BT::NodeStatus::FAILURE;
    }
  }
};


// 节点 3: 关闭夹爪
class CloseGripper : public BT::SyncActionNode {
public:
  CloseGripper(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts() {
    return {};
  }

  BT::NodeStatus tick() override {
    ROS_INFO("Closing gripper...");
    moveit::planning_interface::MoveGroupInterface gripper_group("left_hand");
    gripper_group.setNamedTarget("Close");  // 假设有一个 "close" 的预定义状态
    bool success = (gripper_group.move() == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (success) {
      ROS_INFO("Gripper closed");
      return BT::NodeStatus::SUCCESS;
    } else {
      ROS_ERROR("Failed to close gripper");
      return BT::NodeStatus::FAILURE;
    }
  }
};


int main(int argc, char** argv) {
  ros::init(argc, argv, "robot_arm_behavior_tree_with_moveit");
  ros::NodeHandle nh;

  ros::AsyncSpinner spinner(1);
  spinner.start();

  BT::BehaviorTreeFactory factory;

  // 注册自定义节点
  factory.registerNodeType<OpenGripper>("OpenGripper");
  factory.registerNodeType<MoveToPosition>("MoveToPosition");
  factory.registerNodeType<CloseGripper>("CloseGripper");
  factory.registerNodeType<MoveToHome>("MoveToHome");
  // 定义 XML 配置
  std::string xml_text = R"(
  <root main_tree_to_execute="MainTree">
    <BehaviorTree ID="MainTree">
      <Sequence>
        <MoveToPosition x="0.5424289260290689" y="-0.20" z="1.1555851873193097"
                qx="-0.06784457883373415" qy="-0.7994632205873456" 
                qz="-0.5947445876543256" qw="0.05034428971461809"/>
        <CloseGripper/>
        <MoveToHome/>
        <OpenGripper/>
      </Sequence>
    </BehaviorTree>
  </root>
  )";

  // 创建行为树
  auto tree = factory.createTreeFromText(xml_text);

  BT::NodeStatus status = BT::NodeStatus::IDLE;

  while (ros::ok() && (status == BT::NodeStatus::IDLE || status == BT::NodeStatus::RUNNING)) {
    ros::spinOnce();
    status = tree.rootNode()->executeTick();   
    std::cout << "Behavior Tree Status: " << toStr(status) << std::endl;
    ros::Duration(0.1).sleep();
  }

  return 0;
}
