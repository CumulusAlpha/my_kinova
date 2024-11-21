#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_scene/planning_scene.h>
#include <math.h>
#include <tf2/LinearMath/Quaternion.h>

class MultiGroupSynchronizedMotion {

private:
    moveit::planning_interface::MoveGroupInterface left_arm_group_;
    moveit::planning_interface::MoveGroupInterface left_F1_group_;
    moveit::planning_interface::MoveGroupInterface left_F2_group_;
    moveit::planning_interface::MoveGroupInterface left_F3_group_;
    moveit::planning_interface::MoveGroupInterface right_arm_group_;
    moveit::planning_interface::MoveGroupInterface right_F1_group_;
    moveit::planning_interface::MoveGroupInterface right_F2_group_;
    moveit::planning_interface::MoveGroupInterface right_F3_group_;

public:
    MultiGroupSynchronizedMotion(
        const std::string& left_arm_group_name, const std::string& left_F1_group_name,
        const std::string& left_F2_group_name, const std::string& left_F3_group_name,
        const std::string& right_arm_group_name, const std::string& right_F1_group_name,
        const std::string& right_F2_group_name, const std::string& right_F3_group_name
    ) :
        left_arm_group_(left_arm_group_name), 
        left_F1_group_(left_F1_group_name),
        left_F2_group_(left_F2_group_name),
        left_F3_group_(left_F3_group_name),
        right_arm_group_(right_arm_group_name),
        right_F1_group_(right_F1_group_name),
        right_F2_group_(right_F2_group_name),
        right_F3_group_(right_F3_group_name)
    {}



    bool Armsplan(geometry_msgs::Pose left_arm_target, geometry_msgs::Pose right_arm_target) {
        double position_tolerance = 0.01;  // 位置误差容限，例如 1 cm
        double orientation_tolerance = 0.05;  // 姿态误差容限，例如 0.05 弧度

        left_arm_group_.setGoalPositionTolerance(position_tolerance);
        left_arm_group_.setGoalOrientationTolerance(orientation_tolerance);
        right_arm_group_.setGoalPositionTolerance(position_tolerance);
        right_arm_group_.setGoalOrientationTolerance(orientation_tolerance);

        // Set targets for each group
        left_arm_group_.setPoseTarget(left_arm_target);

        right_arm_group_.setPoseTarget(right_arm_target);

        // Plan for each group separately
        moveit::planning_interface::MoveGroupInterface::Plan left_arm_plan;
        moveit::planning_interface::MoveGroupInterface::Plan right_arm_plan;

        bool success_left_arm = (left_arm_group_.plan(left_arm_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        bool success_right_arm = (right_arm_group_.plan(right_arm_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

        if (!success_left_arm || !success_right_arm) {
            ROS_ERROR("Failed to plan for one or both arm groups");
            return false;
        }

        
        bool success1 = (left_arm_group_.asyncExecute(left_arm_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        bool success2 = (right_arm_group_.asyncExecute(right_arm_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        ros::Duration(10).sleep();

        if (!success1 || !success2) {
            ROS_ERROR("Failed to execute combined trajectory");
            return false;
        }

        ROS_INFO("Synchronized motion executed successfully!");
        return true;
    }

bool controlHand(const std::string& hand_name, int open_close) {
    // 根据手名称创建对应的 MoveGroupInterface 实例
    moveit::planning_interface::MoveGroupInterface hand_group(hand_name);

    // 设置目标姿态：0 表示 "close"，1 表示 "open"
    std::string target_pose = (open_close == 1) ? "Open" : "Close";
    hand_group.setNamedTarget(target_pose);

    // 执行开合动作
    bool success = (hand_group.move() == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (success) {
        ROS_INFO_STREAM(hand_name << (open_close == 1 ? "opened" : "closed") << " successfully.");
    } else {
        ROS_ERROR_STREAM("Failed to " << (open_close == 1 ? "open" : "close") << " " << hand_name );
    }

    return success;
}

};

int main(int argc, char** argv) {
    ros::init(argc, argv, "multi_group_synchronized_motion");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    MultiGroupSynchronizedMotion multi_group_motion(
        "left_arm", "left_F1", "left_F2", "left_F3",
        "right_arm", "right_F1", "right_F2", "right_F3"
    );

    // Define target positions (adjust as needed)

    // geometry_msgs::Pose left_arm_target;
    // left_arm_target.position.x = 0.655852;
    // left_arm_target.position.y = -0.0948;
    // left_arm_target.position.z =  1.1394756;
    // left_arm_target.orientation.x = -0.5;
    // left_arm_target.orientation.y =  -0.5;
    // left_arm_target.orientation.z =  0.5;
    // left_arm_target.orientation.w = -0.5;

    // geometry_msgs::Pose right_arm_target;
    // right_arm_target.position.x = 0;
    // right_arm_target.position.y = 1.5;
    // right_arm_target.position.z = 1.4;
    // right_arm_target.orientation.x = -0.5;
    // right_arm_target.orientation.y = 0.5;
    // right_arm_target.orientation.z = 0.5;
    // right_arm_target.orientation.w =0.5;

    // if (!multi_group_motion.Armplan(left_arm_target, right_arm_target)) {
    //     ROS_ERROR("Failed to execute synchronized motion");
    //     return 1;
    // }

    // std::vector<double> left_F1_target = {0, 0, 0, 0}; // 例如：左手食指的目标角度
    // std::vector<double> right_F1_target = {0, 0, 0, 0}; // 例如：右手食指的目标角度

    // std::vector<double> left_F2_target = {0, 0, 0, 0}; // 例如：左手中指的目标角度
    // std::vector<double> right_F2_target = {0, 0, 0, 0}; // 例如：右手中指的目标角度

    // std::vector<double> left_F3_target = {0, 0, 0, 0}; // 例如：左手无名指的目标角度
    // std::vector<double> right_F3_target = {0, 0, 0, 0}; // 例如：右手无名指的目标角度


    // std::vector<double> left_F1_target = {0, 0, 70, 60}; // 例如：左手食指的目标角度
    // std::vector<double> right_F1_target = {0, 0, 70, 60}; // 例如：右手食指的目标角度

    // std::vector<double> left_F2_target = {0, 0, 70, 60}; // 例如：左手中指的目标角度
    // std::vector<double> right_F2_target = {0, 0, 70, 60}; // 例如：右手中指的目标角度

    // std::vector<double> left_F3_target = {0, 0, 70, 60}; // 例如：左手无名指的目标角度
    // std::vector<double> right_F3_target = {0, 0, 70, 60}; // 例如：右手无名指的目标角度

    // −0.5,−0.5,0.5,−0.5 left
    //0.5 -0.5 -0.5 -0.5 
    double roll = 0;
    double pitch = 0;
    double yaw = 0;

    multi_group_motion.controlHand("left_hand", 0);

    ros::Duration(5).sleep();
    multi_group_motion.controlHand("left_hand", 1);
  // multi_group_motion.Handplan(left_F1_target,right_F1_target,left_F2_target,right_F2_target,left_F3_target,right_F3_target);
    // multi_group_motion.left_Handplan(left_F1_target,left_F2_target,left_F3_target);
   // multi_group_motion.right_Handplan(right_F1_target,right_F2_target,right_F3_target);
    ros::Duration(10).sleep();

    ros::shutdown();
    return 0;
}


// #include <ros/ros.h>
// #include <moveit/move_group_interface/move_group_interface.h>
// #include <moveit/planning_interface/planning_interface.h>
// #include <geometry_msgs/Pose.h>
// #include <thread>

// class MultiGroupSynchronizedMotion {
// private:
//     moveit::planning_interface::MoveGroupInterface left_arm_group_;
//     moveit::planning_interface::MoveGroupInterface right_arm_group_;

// public:
//     MultiGroupSynchronizedMotion(const std::string& left_arm_group_name, const std::string& right_arm_group_name)
//         : left_arm_group_(left_arm_group_name), right_arm_group_(right_arm_group_name) {}

//     bool planAndExecuteLeftArm(geometry_msgs::Pose left_arm_target) {
//         left_arm_group_.setPoseTarget(left_arm_target);
//         moveit::planning_interface::MoveGroupInterface::Plan left_arm_plan;
//         bool success_left_arm = (left_arm_group_.plan(left_arm_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        
//         if (success_left_arm) {
//             return (left_arm_group_.execute(left_arm_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//         }
//         ROS_ERROR("Failed to plan for left arm");
//         return false;
//     }

//     bool planAndExecuteRightArm(geometry_msgs::Pose right_arm_target) {
//         right_arm_group_.setPoseTarget(right_arm_target);
//         moveit::planning_interface::MoveGroupInterface::Plan right_arm_plan;
//         bool success_right_arm = (right_arm_group_.plan(right_arm_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

//         if (success_right_arm) {
//             return (right_arm_group_.execute(right_arm_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//         }
//         ROS_ERROR("Failed to plan for right arm");
//         return false;
//     }

//     // 使用多线程规划和执行左右臂的动作
//     bool planAndExecute(geometry_msgs::Pose left_arm_target, geometry_msgs::Pose right_arm_target) {
//         std::thread left_arm_thread(&MultiGroupSynchronizedMotion::planAndExecuteLeftArm, this, left_arm_target);
//         std::thread right_arm_thread(&MultiGroupSynchronizedMotion::planAndExecuteRightArm, this, right_arm_target);

//         left_arm_thread.join();  // 等待左臂线程完成
//         right_arm_thread.join(); // 等待右臂线程完成

//         // 返回两个臂的执行结果
//         return true;
//     }
// };

// int main(int argc, char** argv) {
//     ros::init(argc, argv, "multi_group_synchronized_motion");
//     ros::NodeHandle nh;
//     ros::AsyncSpinner spinner(1);
//     spinner.start();

//     MultiGroupSynchronizedMotion multi_group_motion("left_arm", "right_arm");

//     // 设置目标位置
//     geometry_msgs::Pose left_arm_target;
//     left_arm_target.position.x =0.4203862581129514;
//     left_arm_target.position.y = -1.2929717284915967;
//     left_arm_target.position.z =  1.2942027339563427;
//     left_arm_target.orientation.x = 0.000562;
//     left_arm_target.orientation.y = 0.707385;
//     left_arm_target.orientation.z = -0.706827;
//     left_arm_target.orientation.w = 0.0006;

//     geometry_msgs::Pose right_arm_target;
//     right_arm_target.position.x = 0.13462738361704085;
//     right_arm_target.position.y = 1.1556872315376894;
//     right_arm_target.position.z = 1.0087470266649854;
//     right_arm_target.orientation.x = -0.706886724759504;
//     right_arm_target.orientation.y = -1.9381658565884843e-05;
//     right_arm_target.orientation.z = 1.047248167927157e-06;
//     right_arm_target.orientation.w =0.7073267688855249;

//     if (!multi_group_motion.planAndExecute(left_arm_target, right_arm_target)) {
//         ROS_ERROR("Failed to execute synchronized motion");
//         return 1;
//     }

//     ros::shutdown();
//     return 0;
// }


/////////////////////////////////////////////////

// Combine the trajectories
        // 创建 combined_trajectory 来合并轨迹
        // robot_trajectory::RobotTrajectory combined_trajectory(left_arm_group_.getRobotModel(), "left_arm");
        // moveit::core::RobotStatePtr current_state = left_arm_group_.getCurrentState();
        // combined_trajectory.setRobotTrajectoryMsg(*current_state, left_arm_plan.trajectory_);

        // // 创建一个新的 RobotTrajectory 对象来存储 right_arm_plan.trajectory_ 并将其转换
        // robot_trajectory::RobotTrajectory right_arm_trajectory(left_arm_group_.getRobotModel(), "right_arm");
        // right_arm_trajectory.setRobotTrajectoryMsg(*current_state, right_arm_plan.trajectory_);

        // // 将 right_arm_trajectory 追加到 combined_trajectory
        // combined_trajectory.append(right_arm_trajectory, 0.0);


        // // Time parameterization
        // trajectory_processing::IterativeParabolicTimeParameterization time_param;
        // if (!time_param.computeTimeStamps(combined_trajectory)) {
        //     ROS_ERROR("Failed to compute time stamps for the combined trajectory");
        //     return false;
        // }

        // // Store combined trajectory in a new plan
        // moveit::planning_interface::MoveGroupInterface::Plan combined_plan;
        // combined_trajectory.getRobotTrajectoryMsg(combined_plan.trajectory_);
        // Execute synchronized trajectory



    //     bool left_Handplan( std::vector<double> left_F1_target,  std::vector<double> right_F1_target,
    //              std::vector<double> left_F2_target,  std::vector<double> right_F2_target,
    //              std::vector<double> left_F3_target,  std::vector<double> right_F3_target
    // ) {
    //     // double position_tolerance = 0.01;  // 位置误差容限，例如 1 cm
    //     // double orientation_tolerance = 0.05;  // 姿态误差容限，例如 0.05 弧度

    //     // left_F1_group_.setGoalPositionTolerance(position_tolerance);
    //     // left_F1_group_.setGoalOrientationTolerance(orientation_tolerance);
    //     // left_F2_group_.setGoalPositionTolerance(position_tolerance);
    //     // left_F2_group_.setGoalOrientationTolerance(orientation_tolerance);        
    //     // left_F3_group_.setGoalPositionTolerance(position_tolerance);
    //     // left_F3_group_.setGoalOrientationTolerance(orientation_tolerance);


    //     // right_F1_group_.setGoalPositionTolerance(position_tolerance);
    //     // right_F1_group_.setGoalOrientationTolerance(orientation_tolerance);
    //     // right_F2_group_.setGoalPositionTolerance(position_tolerance);
    //     // right_F2_group_.setGoalOrientationTolerance(orientation_tolerance);        
    //     // right_F3_group_.setGoalPositionTolerance(position_tolerance);
    //     // right_F3_group_.setGoalOrientationTolerance(orientation_tolerance);

    //     // Set targets for each group
    //     // left_arm_group_.setPoseTarget(left_arm_target);

    //     // right_arm_group_.setPoseTarget(right_arm_target);

    //     // 将角度转换为弧度
    //     auto deg_to_rad = M_PI / 180.0;
        
    //     // 转换所有目标关节角度（从度转换为弧度）
    //     for (auto& angle : left_F1_target) angle *= deg_to_rad;
    //     for (auto& angle : right_F1_target) angle *= deg_to_rad;
    //     for (auto& angle : left_F2_target) angle *= deg_to_rad;
    //     for (auto& angle : right_F2_target) angle *= deg_to_rad;
    //     for (auto& angle : left_F3_target) angle *= deg_to_rad;
    //     for (auto& angle : right_F3_target) angle *= deg_to_rad;

    //     left_F1_group_.setJointValueTarget(left_F1_target);
    //     left_F2_group_.setJointValueTarget(left_F2_target);        
    //     left_F3_group_.setJointValueTarget(left_F3_target);

    //     right_F1_group_.setJointValueTarget(right_F1_target);
    //     right_F2_group_.setJointValueTarget(right_F2_target);        
    //     right_F3_group_.setJointValueTarget(right_F3_target);

    //     // Plan for each group separately
    //     moveit::planning_interface::MoveGroupInterface::Plan left_F1_plan;
    //     moveit::planning_interface::MoveGroupInterface::Plan left_F2_plan;
    //     moveit::planning_interface::MoveGroupInterface::Plan left_F3_plan;
    //     moveit::planning_interface::MoveGroupInterface::Plan right_F1_plan;
    //     moveit::planning_interface::MoveGroupInterface::Plan right_F2_plan;
    //     moveit::planning_interface::MoveGroupInterface::Plan right_F3_plan;


    //     bool success_left_F1 = (left_F1_group_.plan(left_F1_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    //     bool success_left_F2 = (left_F2_group_.plan(left_F2_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    //     bool success_left_F3 = (left_F3_group_.plan(left_F3_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);


    //     bool success_right_F1 = (right_F1_group_.plan(right_F1_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    //     bool success_right_F2 = (right_F2_group_.plan(right_F2_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    //     bool success_right_F3 = (right_F3_group_.plan(right_F3_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    //     if (!success_left_F1 || !success_left_F2 || !success_left_F3|| !success_right_F1|| !success_right_F2 || !success_right_F3) {
    //         ROS_ERROR("Failed to plan for one or both arm groups");
    //         return false;
    //     }
    //     bool success1 = (left_F1_group_.asyncExecute(left_F1_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    //     bool success2 = (left_F2_group_.asyncExecute(left_F2_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    //     bool success3 = (left_F3_group_.asyncExecute(left_F3_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    //     bool success4 = (right_F1_group_.asyncExecute(right_F1_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    //     bool success5 = (right_F2_group_.asyncExecute(right_F2_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    //     bool success6 = (right_F3_group_.asyncExecute(right_F3_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    //     ros::Duration(10).sleep();

    //     if (!success1 || !success2|| !success3 ||!success4 ||!success5 ||!success6) {
    //         ROS_ERROR("Failed to execute combined trajectory");
    //         return false;
    //     }

    //     ROS_INFO("Synchronized motion executed successfully!");
    //     return true;
    // }


//     bool Handplan(std::vector<double> left_F1_target, std::vector<double> right_F1_target,
//               std::vector<double> left_F2_target, std::vector<double> right_F2_target,
//               std::vector<double> left_F3_target, std::vector<double> right_F3_target) {

//     // 将角度转换为弧度
//     auto deg_to_rad = M_PI / 180.0;
//     for (auto& angle : left_F1_target) angle *= deg_to_rad;
//     for (auto& angle : right_F1_target) angle *= deg_to_rad;
//     for (auto& angle : left_F2_target) angle *= deg_to_rad;
//     for (auto& angle : right_F2_target) angle *= deg_to_rad;
//     for (auto& angle : left_F3_target) angle *= deg_to_rad;
//     for (auto& angle : right_F3_target) angle *= deg_to_rad;

//     left_F1_group_.setJointValueTarget(left_F1_target);
//     left_F2_group_.setJointValueTarget(left_F2_target);
//     left_F3_group_.setJointValueTarget(left_F3_target);

//     right_F1_group_.setJointValueTarget(right_F1_target);
//     right_F2_group_.setJointValueTarget(right_F2_target);
//     right_F3_group_.setJointValueTarget(right_F3_target);

//     // 定义每个关节组的执行函数
//     auto executeGroup = [](moveit::planning_interface::MoveGroupInterface& group) {
//         moveit::planning_interface::MoveGroupInterface::Plan plan;
//         if (group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
//             group.asyncExecute(plan);
//         } else {
//             ROS_ERROR("Failed to plan for one of the finger groups");
//         }
//     };

//     // 并行执行所有关节组
//     std::thread t1(executeGroup, std::ref(left_F1_group_));
//     std::thread t2(executeGroup, std::ref(left_F2_group_));
//     std::thread t3(executeGroup, std::ref(left_F3_group_));
//     std::thread t4(executeGroup, std::ref(right_F1_group_));
//     std::thread t5(executeGroup, std::ref(right_F2_group_));
//     std::thread t6(executeGroup, std::ref(right_F3_group_));

//     // 等待所有线程完成
//     t1.join();
//     ros::Duration(0.1).sleep();
//     t2.join();
//     ros::Duration(0.1).sleep();
//     t3.join();
//     ros::Duration(0.1).sleep();
//     t4.join();
//     ros::Duration(0.1).sleep();
//     t5.join();
//     ros::Duration(0.1).sleep();
//     t6.join();
//     ROS_INFO("All finger groups executed in parallel");
//     return true;
// }


// bool left_Handplan( std::vector<double> left_F1_target,  std::vector<double> left_F2_target,  std::vector<double> left_F3_target
//     ) {
//         // double position_tolerance = 0.01;  // 位置误差容限，例如 1 cm
//         // double orientation_tolerance = 0.05;  // 姿态误差容限，例如 0.05 弧度

//         // left_F1_group_.setGoalPositionTolerance(position_tolerance);
//         // left_F1_group_.setGoalOrientationTolerance(orientation_tolerance);
//         // left_F2_group_.setGoalPositionTolerance(position_tolerance);
//         // left_F2_group_.setGoalOrientationTolerance(orientation_tolerance);        
//         // left_F3_group_.setGoalPositionTolerance(position_tolerance);
//         // left_F3_group_.setGoalOrientationTolerance(orientation_tolerance);


//         // right_F1_group_.setGoalPositionTolerance(position_tolerance);
//         // right_F1_group_.setGoalOrientationTolerance(orientation_tolerance);
//         // right_F2_group_.setGoalPositionTolerance(position_tolerance);
//         // right_F2_group_.setGoalOrientationTolerance(orientation_tolerance);        
//         // right_F3_group_.setGoalPositionTolerance(position_tolerance);
//         // right_F3_group_.setGoalOrientationTolerance(orientation_tolerance);

//         // Set targets for each group
//         // left_arm_group_.setPoseTarget(left_arm_target);

//         // right_arm_group_.setPoseTarget(right_arm_target);

//         // 将角度转换为弧度
//         auto deg_to_rad = M_PI / 180.0;
        
//         // 转换所有目标关节角度（从度转换为弧度）
//         for (auto& angle : left_F1_target) angle *= deg_to_rad;
        
//         for (auto& angle : left_F2_target) angle *= deg_to_rad;
        
//         for (auto& angle : left_F3_target) angle *= deg_to_rad;
        
//         left_F1_group_.setJointValueTarget(left_F1_target);
//         left_F2_group_.setJointValueTarget(left_F2_target);        
//         left_F3_group_.setJointValueTarget(left_F3_target);

//         // Plan for each group separately
//         moveit::planning_interface::MoveGroupInterface::Plan left_F1_plan;
//         moveit::planning_interface::MoveGroupInterface::Plan left_F2_plan;
//         moveit::planning_interface::MoveGroupInterface::Plan left_F3_plan;


//         bool success_left_F1 = (left_F1_group_.plan(left_F1_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//         bool success_left_F2 = (left_F2_group_.plan(left_F2_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//         bool success_left_F3 = (left_F3_group_.plan(left_F3_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

//         if (!success_left_F1 || !success_left_F2 || !success_left_F3) {
//             ROS_ERROR("Failed to plan for one or both arm groups");
//             return false;
//         }
//         bool success1 = (left_F1_group_.asyncExecute(left_F1_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//         ros::Duration(0.1).sleep();
//         bool success2 = (left_F2_group_.asyncExecute(left_F2_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//         ros::Duration(0.1).sleep();
//         bool success3 = (left_F3_group_.asyncExecute(left_F3_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        

//         if (!success1 || !success2|| !success3 ) {
//             ROS_ERROR("Failed to execute combined trajectory");
//             return false;
//         }

//         ROS_INFO("Synchronized motion executed successfully!");
//         return true;
//     }
// bool right_Handplan(std::vector<double> right_F1_target,  std::vector<double> right_F2_target,std::vector<double> right_F3_target) {
        // double position_tolerance = 0.01;  // 位置误差容限，例如 1 cm
        // double orientation_tolerance = 0.05;  // 姿态误差容限，例如 0.05 弧度

        // left_F1_group_.setGoalPositionTolerance(position_tolerance);
        // left_F1_group_.setGoalOrientationTolerance(orientation_tolerance);
        // left_F2_group_.setGoalPositionTolerance(position_tolerance);
        // left_F2_group_.setGoalOrientationTolerance(orientation_tolerance);        
        // left_F3_group_.setGoalPositionTolerance(position_tolerance);
        // left_F3_group_.setGoalOrientationTolerance(orientation_tolerance);


        // right_F1_group_.setGoalPositionTolerance(position_tolerance);
        // right_F1_group_.setGoalOrientationTolerance(orientation_tolerance);
        // right_F2_group_.setGoalPositionTolerance(position_tolerance);
        // right_F2_group_.setGoalOrientationTolerance(orientation_tolerance);        
        // right_F3_group_.setGoalPositionTolerance(position_tolerance);
        // right_F3_group_.setGoalOrientationTolerance(orientation_tolerance);

        // Set targets for each group
        // left_arm_group_.setPoseTarget(left_arm_target);

        // right_arm_group_.setPoseTarget(right_arm_target);

        // 将角度转换为弧度
    //     auto deg_to_rad = M_PI / 180.0;
        
    //     // 转换所有目标关节角度（从度转换为弧度）
    //     for (auto& angle : right_F1_target) angle *= deg_to_rad;
       
    //     for (auto& angle : right_F2_target) angle *= deg_to_rad;
        
    //     for (auto& angle : right_F3_target) angle *= deg_to_rad;

    //     right_F1_group_.setJointValueTarget(right_F1_target);
    //     right_F2_group_.setJointValueTarget(right_F2_target);        
    //     right_F3_group_.setJointValueTarget(right_F3_target);

    //     // Plan for each group separately
        
    //     moveit::planning_interface::MoveGroupInterface::Plan right_F1_plan;
    //     moveit::planning_interface::MoveGroupInterface::Plan right_F2_plan;
    //     moveit::planning_interface::MoveGroupInterface::Plan right_F3_plan;

    //     bool success_right_F1 = (right_F1_group_.plan(right_F1_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    //     bool success_right_F2 = (right_F2_group_.plan(right_F2_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    //     bool success_right_F3 = (right_F3_group_.plan(right_F3_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    //     if (!success_right_F1|| !success_right_F2 || !success_right_F3) {
    //         ROS_ERROR("Failed to plan for one or both arm groups");
    //         return false;
    //     }
    //     bool success4 = (right_F1_group_.asyncExecute(right_F1_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    //     bool success5 = (right_F2_group_.asyncExecute(right_F2_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    //     bool success6 = (right_F3_group_.asyncExecute(right_F3_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    //     ros::Duration(2.4).sleep();

    //     if (!success4 ||!success5 ||!success6) {
    //         ROS_ERROR("Failed to execute combined trajectory");
    //         return false;
    //     }

    //     ROS_INFO("Synchronized motion executed successfully!");
    //     return true;
    // }