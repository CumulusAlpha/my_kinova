#include <ros/ros.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

const double tau = 2 * M_PI;
const double cor = 2 * M_PI / 180 ;

void openGripper(trajectory_msgs::JointTrajectory& posture)
{
    posture.joint_names.resize(12);
    posture.joint_names[0] = "left_F1M1";
    posture.joint_names[1] = "left_F1M2";
    posture.joint_names[2] = "left_F1M3";
    posture.joint_names[3] = "left_F1M4";
    posture.joint_names[4] = "left_F2M1";
    posture.joint_names[5] = "left_F2M2";
    posture.joint_names[6] = "left_F2M3";
    posture.joint_names[7] = "left_F2M4";
    posture.joint_names[8] = "left_F3M1";
    posture.joint_names[9] = "left_F3M2";
    posture.joint_names[10] = "left_F3M3";
    posture.joint_names[11] = "left_F3M4";

    posture.points.resize(1);
    posture.points[0].positions.resize(12);
    // 定义每个关节的目标角度（假设单位是弧度）
    posture.points[0].positions[0] = 0;  // 左手指第一节
    posture.points[0].positions[1] = 0;  // 左手指第二节
    posture.points[0].positions[2] = 0;  // 左手指第三节
    posture.points[0].positions[3] = 0;  // 左手指第四节
    posture.points[0].positions[4] = 0;  // 中手指第一节
    posture.points[0].positions[5] = 0;  // 中手指第二节
    posture.points[0].positions[6] = 0;  // 中手指第三节
    posture.points[0].positions[7] = 0;  // 中手指第四节
    posture.points[0].positions[8] = 0;  // 右手指第一节
    posture.points[0].positions[9] = 0;  // 右手指第二节
    posture.points[0].positions[10] = 0; // 右手指第三节
    posture.points[0].positions[11] = 0; // 右手指第四节

    posture.points[0].time_from_start = ros::Duration(0.5);
}

void closedGripper(trajectory_msgs::JointTrajectory& posture)
{
  posture.joint_names.resize(12);
    posture.joint_names[0] = "left_F1M1";
    posture.joint_names[1] = "left_F1M2";
    posture.joint_names[2] = "left_F1M3";
    posture.joint_names[3] = "left_F1M4";
    posture.joint_names[4] = "left_F2M1";
    posture.joint_names[5] = "left_F2M2";
    posture.joint_names[6] = "left_F2M3";
    posture.joint_names[7] = "left_F2M4";
    posture.joint_names[8] = "left_F3M1";
    posture.joint_names[9] = "left_F3M2";
    posture.joint_names[10] = "left_F3M3";
    posture.joint_names[11] = "left_F3M4";
    
    posture.points.resize(1);
    posture.points[0].positions.resize(12);

    // 定义每个关节的目标角度（假设单位是弧度）
    posture.points[0].positions[0] = 0;  // 左手指第一节
    posture.points[0].positions[1] = 0;  // 左手指第二节
    posture.points[0].positions[2] = 70 * cor;  // 左手指第三节
    posture.points[0].positions[3] = 60 * cor ;  // 左手指第四节
    posture.points[0].positions[4] = 0;  // 中手指第一节
    posture.points[0].positions[5] = 0;  // 中手指第二节
    posture.points[0].positions[6] = 70 * cor;  // 中手指第三节
    posture.points[0].positions[7] = 60 * cor;  // 中手指第四节
    posture.points[0].positions[8] = 0;  // 右手指第一节
    posture.points[0].positions[9] = 0;  // 右手指第二节
    posture.points[0].positions[10] = 70 * cor; // 右手指第三节
    posture.points[0].positions[11] = 60 * cor; // 右手指第四节

    posture.points[0].time_from_start = ros::Duration(0.5);
}

void pick(moveit::planning_interface::MoveGroupInterface& move_group)
{
  std::vector<moveit_msgs::Grasp> grasps(1);
  grasps[0].grasp_pose.header.frame_id = "world";
  tf2::Quaternion orientation;
//   orientation.setRPY(-tau / 4, -tau / 8, -tau / 4);
  orientation.setRPY(0, 0, 0);

  grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);
  grasps[0].grasp_pose.pose.position.x = 0.5;
  grasps[0].grasp_pose.pose.position.y = 0;
  grasps[0].grasp_pose.pose.position.z = 1.25;

  grasps[0].pre_grasp_approach.direction.header.frame_id = "world";
  grasps[0].pre_grasp_approach.direction.vector.x = 1.0;
  grasps[0].pre_grasp_approach.min_distance = 0.005;
  grasps[0].pre_grasp_approach.desired_distance = 0.010;

  grasps[0].post_grasp_retreat.direction.header.frame_id = "world";
  grasps[0].post_grasp_retreat.direction.vector.z = 1.0;
  grasps[0].post_grasp_retreat.min_distance = 0.001;
  grasps[0].post_grasp_retreat.desired_distance = 0.25;

  openGripper(grasps[0].pre_grasp_posture);
  closedGripper(grasps[0].grasp_posture);

  move_group.setSupportSurfaceName("table1");
  move_group.pick("object", grasps);
}

void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface)
{
  std::vector<moveit_msgs::CollisionObject> collision_objects(1);

  collision_objects[0].id = "table1";
  collision_objects[0].header.frame_id = "world";
  collision_objects[0].primitives.resize(1);
  collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
  collision_objects[0].primitives[0].dimensions = {1, 2, 1};
  collision_objects[0].primitive_poses.resize(1);
  collision_objects[0].primitive_poses[0].position.x = 0.8;
  collision_objects[0].primitive_poses[0].position.z = 0.5;
  collision_objects[0].primitive_poses[0].orientation.w = 1.0;
  collision_objects[0].operation = collision_objects[0].ADD;

//   collision_objects[1].id = "table2";
//   collision_objects[1].header.frame_id = "panda_link0";
//   collision_objects[1].primitives.resize(1);
//   collision_objects[1].primitives[0].type = collision_objects[1].primitives[0].BOX;
//   collision_objects[1].primitives[0].dimensions = {0.4, 0.2, 0.4};
//   collision_objects[1].primitive_poses.resize(1);
//   collision_objects[1].primitive_poses[0].position.y = 0.5;
//   collision_objects[1].primitive_poses[0].position.z = 0.2;
//   collision_objects[1].primitive_poses[0].orientation.w = 1.0;
//   collision_objects[1].operation = collision_objects[1].ADD;

    collision_objects[0].header.frame_id = "world";
    collision_objects[0].id = "object";
    collision_objects[0].primitives.resize(1);
    collision_objects[0].primitives[0].type = shape_msgs::SolidPrimitive::CYLINDER;
    // dimensions[0] 高度，dimensions[1] 是半径
    collision_objects[0].primitives[0].dimensions = {0.04, 0.04};

    collision_objects[0].primitive_poses.resize(1);
    collision_objects[0].primitive_poses[0].position.x = 0.5;
    collision_objects[0].primitive_poses[0].position.z = 1.25;
    collision_objects[0].primitive_poses[0].orientation.w = 1.0;
    collision_objects[0].operation = collision_objects[2].ADD;

    planning_scene_interface.applyCollisionObjects(collision_objects);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pick_place");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::WallDuration(1.0).sleep();
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit::planning_interface::MoveGroupInterface group("left_arm");
  group.setPlanningTime(45.0);

  addCollisionObjects(planning_scene_interface);

  ros::WallDuration(1.0).sleep();
  pick(group);

  ros::waitForShutdown();
  return 0;
}
