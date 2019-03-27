#include <moveit/move_group_interface/move_group_interface.h>S
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <tf/LinearMath/Quaternion.h>
#include "std_msgs/Float64MultiArray.h"
#include <actionlib/server/simple_action_server.h>
#include "std_msgs/Float32.h"
#include <aubo_demo/PublishPositionAction.h>

typedef actionlib::SimpleActionServer<aubo_demo::PublishPositionAction> Server;

  static float position_x;
  static float position_y;
  static float position_z;

void execute(const aubo_demo::PublishPositionGoalConstPtr& goal, Server* as)
{
    aubo_demo::PublishPositionFeedback feedback;
    int i = 1;
    feedback.move_time = i;
    
    //return the process of task
    as->publishFeedback(feedback);

    position_x = goal->object_position[0];
    position_y = goal->object_position[1];
    position_z = goal->object_position[2];
    ROS_INFO("The Round %d, Robot needs to move to position(%f, %f, %f).",feedback.move_time, position_x, position_y, position_z);
    as->setSucceeded();
    i++;
}

// std::vector<double> receive_point;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_position_server");
  ros::NodeHandle node_handle;
  Server server(node_handle, "move_position", boost::bind(&execute, _1, &server), false);
  server.start();
  
  // ros::AsyncSpinner spinner(1);
  // spinner.start();
  

  // Define the planning group name
  static const std::string PLANNING_GROUP = "manipulator_i5";
  // Create a planning group interface object and set up a planning group
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
  // Create a planning scene interface object
  // moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Create a robot model information object
  const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  // Get the coordinate system of the basic information
  ROS_INFO_NAMED("tutorial", "Planning frame: %s", move_group.getPlanningFrame().c_str());
  // Get the end of the basic information
  ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());

  // Home Position
  std::vector<double> home_position;
  home_position.push_back(-0.001255);
  home_position.push_back(-0.148822);
  home_position.push_back(-1.406503);
  home_position.push_back(0.311441);
  home_position.push_back(-1.571295);
  home_position.push_back(-0.002450);

  move_group.setJointValueTarget(home_position);
  move_group.move();

  // Set the target pose , RPY mode (rotation around the reference axis X, Y, Z)
  tf::Quaternion q;
  q.setRPY(3.14,0,-1.57);       //radian

  geometry_msgs::Pose target_pose;

  target_pose.position.x = position_x;
  target_pose.position.y = position_y;
  target_pose.position.z = position_z;

  target_pose.orientation.x = q.x();
  target_pose.orientation.y = q.y();
  target_pose.orientation.z = q.z();
  target_pose.orientation.w = q.w();

  move_group.setPoseTarget(target_pose);

  // // Call the planner for planning calculations Note: This is just planning
  // moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  // bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  // ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "Success" : "FAILED");

  // // Perform planning actions
  // move_group.execute(my_plan);

  // // Move to the home point position
  // move_group.setJointValueTarget(home_position);
  // move_group.move();
  ros::spin();
  return 0;
}
