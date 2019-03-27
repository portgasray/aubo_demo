/*the robot arm moves from point A to point B according to the set path constraint.*/

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>


#include <tf/LinearMath/Quaternion.h>
int main(int argc, char** argv)
{
    ros::init(argc, argv, "move_group_interface_tutorial");
    ros::NodeHandle node_handle;

    // Start a thread
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // Define the planning group name
    static const std::string PLANNING_GROUP = "manipulator_i5";
    // Create a planning group interface object and set up a planning group
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    // Create a planning scene interface object
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    // Create a robot model information object
    const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);


    tf::Quaternion q;
    q.setRPY(3.14,0,-1.57);

    // Define the path constraint
    moveit_msgs::OrientationConstraint ocm;
    ocm.link_name = "wrist3_Link";
    ocm.header.frame_id = "base_link";

    // Set the pose to be constrained at the end (consistent with base_link according to the settings)
    ocm.orientation.w = q.w();
    ocm.orientation.x = q.x();
    ocm.orientation.y = q.y();
    ocm.orientation.z = q.z();
    ocm.absolute_x_axis_tolerance = 0.2;   //(Specify the tolerance of the axis)
    ocm.absolute_y_axis_tolerance = 0.2;
    ocm.absolute_z_axis_tolerance = 0.2;
    ocm.weight = 1.0;


    // Add path constraints
    moveit_msgs::Constraints test_constraints;
    test_constraints.orientation_constraints.push_back(ocm);
    move_group.setPathConstraints(test_constraints);

    // Set initial position
    robot_state::RobotState start_state(*move_group.getCurrentState());
    geometry_msgs::Pose     start_pose;
    start_pose.position.x = -0.4;
    start_pose.position.y = 0.05;
    start_pose.position.z = 0.54;
    start_pose.orientation.x = q.x();
    start_pose.orientation.y = q.y();
    start_pose.orientation.z = q.z();
    start_pose.orientation.w = q.w();

    //1. the robot must first move to the starting position
    move_group.setPoseTarget(start_pose);

    // Reset the joint_model_group of the starting position for visual track display
    start_state.setFromIK(joint_model_group, start_pose);
    move_group.setStartState(start_state);


    // Set the target pose
    geometry_msgs::Pose target_pose3_1;
    target_pose3_1.position.x = -0.4;
    target_pose3_1.position.y = -0.19;
    target_pose3_1.position.z = 0.41;
    target_pose3_1.orientation.x = q.x();
    target_pose3_1.orientation.y = q.y();
    target_pose3_1.orientation.z = q.z();
    target_pose3_1.orientation.w = q.w();

    //move to the target 
    move_group.setPoseTarget(target_pose3_1);
    move_group.move();

    // The default time for the kinematics solver calculation plan is 5s. Increasing the time can increase the success rate.
    // move_group.setPlanningTime(20.0);

    // success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);


    // // Visualize the plan in RViz
    // visual_tools.deleteAllMarkers();
    // visual_tools.publishAxisLabeled(start_pose, "start");
    // visual_tools.publishAxisLabeled(target_pose3_1, "goal");
    // visual_tools.publishText(text_pose, "AUBO Constrained Goal Example3", rvt::RED, rvt::XLARGE);
    // visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
    // visual_tools.trigger();


    // Perform planning actions
    // move_group.execute(my_plan);

    // Move to the home point position
    move_group.setPoseTarget(start_pose);
    move_group.move();

    // Clear path constraint
    move_group.clearPathConstraints();

}
