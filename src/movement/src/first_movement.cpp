#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit/robot_model_loader/robot_model_loader.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>


int main(int argc, char** argv)
{
    ros::init(argc, argv, "first_movement");
    ros::NodeHandle node_handle;

    // ROS spinning must be running for the MoveGroupInterface to get information
    // about the robot's state. One way to do this is to start an AsyncSpinner
    // beforehand.
    ros::AsyncSpinner spinner(1);
    spinner.start();

    static const std::string PLANNING_GROUP = "yaskawa_arm";
    moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);
    const moveit::core::JointModelGroup* joint_model_group = move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    // Visualization
    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
    visual_tools.deleteAllMarkers();
    visual_tools.loadRemoteControl();

    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 1.0;
    visual_tools.publishText(text_pose, "Movement for Demo Day", rvt::WHITE, rvt::XLARGE);

    visual_tools.trigger();
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

    // Info
    ROS_INFO_NAMED("tutorial", "Planning frame: %s", move_group_interface.getPlanningFrame().c_str());

    // We can also print the name of the end-effector link for this group.
    ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group_interface.getEndEffectorLink().c_str());

    // We can get a list of all the groups in the robot:
    ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
    std::copy(move_group_interface.getJointModelGroupNames().begin(),
    move_group_interface.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));


    // Planning
    // moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    moveit::core::RobotStatePtr current_state = move_group_interface.getCurrentState();
    
    std::vector<geometry_msgs::Pose> waypoints;
    moveit::core::RobotState start_state(*move_group_interface.getCurrentState());
    move_group_interface.setStartState(start_state);
    geometry_msgs::Pose start_pose2 = move_group_interface.getCurrentPose().pose;
    std::cout << start_pose2;

    //Position 1 
    // start_pose2.orientation.x = -0.0222971;
    // start_pose2.orientation.y = 0.698193;
    // start_pose2.orientation.z = 0.0122177;
    // start_pose2.orientation.w = 0.715458;
    start_pose2.orientation.x = 0;
    start_pose2.orientation.y = 0.7071068;
    start_pose2.orientation.z = 0;
    start_pose2.orientation.w = 0.7071068;
    start_pose2.position.x = 0;
    start_pose2.position.y = -0.87;
    start_pose2.position.z = 0.461 + 0.322;

    waypoints.push_back(start_pose2);

    //Position 2
    geometry_msgs::Pose target_pose3 = start_pose2;

    target_pose3.orientation.x = 0;
    target_pose3.orientation.y = 0.7071068;
    target_pose3.orientation.z = 0;
    target_pose3.orientation.w = 0.7071068;
    target_pose3.position.x = -0.481;
    target_pose3.position.y = -0.87;
    target_pose3.position.z = 0.271 + 0.322;
    waypoints.push_back(target_pose3);  // down

    //Position 3
    geometry_msgs::Pose target_pose4 = target_pose3;
    //CHANGE THIS ORIENTATION
    target_pose4.orientation.x = 0;
    target_pose4.orientation.y = 0.7071068;
    target_pose4.orientation.z = 0;
    target_pose4.orientation.w = 0.7071068;
    target_pose4.position.x = -0.481;
    target_pose4.position.y = -0.87;
    target_pose4.position.z = 0.449;
    waypoints.push_back(target_pose4);  // down

    // //Position 4
    // geometry_msgs::Pose target_pose5 = target_pose4; 
    // target_pose5.position.z -= 0.05;
    // target_pose5.position.x -= 0.05;
    // waypoints.push_back(target_pose5);  // down


    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 5.0;
    const double eef_step = 0.01;
    double fraction = move_group_interface.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (Cartesian path) (%.2f%% achieved)", fraction * 100.0);

    visual_tools.deleteAllMarkers();
    visual_tools.publishText(text_pose, "Cartesian Path", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishTrajectoryLine(trajectory, joint_model_group);
    visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
    for (std::size_t i = 0; i < waypoints.size(); ++i)
    visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
    move_group_interface.execute(trajectory);

    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
    
    std::vector<geometry_msgs::Pose> waypoints2;
    moveit::core::RobotState start_state2(*move_group_interface.getCurrentState());
    move_group_interface.setStartState(start_state2);
    //geometry_msgs::Pose start_pose3 = move_group_interface.getCurrentPose().pose;

    //Position 1 
    geometry_msgs::Pose target_pose5 = target_pose4;

    target_pose5.position.x = -0.481;
    target_pose5.position.y = -0.87;
    target_pose5.position.z = 0.271 + 0.322;
    waypoints2.push_back(target_pose5);  // prepick

    waypoints2.push_back(target_pose5);

    //Position 2
    geometry_msgs::Pose target_pose6 = target_pose5;

    target_pose6.position.x = 0;
    target_pose6.position.y = -0.87;
    target_pose6.position.z = 0.461 + 0.322;
    waypoints2.push_back(target_pose6);  // home

    //Position 3
    geometry_msgs::Pose target_pose7 = target_pose6;
    target_pose7.position.x = 1.14532;
    target_pose7.position.y = -0.789876;
    target_pose7.position.z = 0.47;
    waypoints2.push_back(target_pose7);  // place

    moveit_msgs::RobotTrajectory trajectory2;
    double fraction2 = move_group_interface.computeCartesianPath(waypoints2, eef_step, jump_threshold, trajectory2);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (Cartesian path) (%.2f%% achieved)", fraction2 * 100.0);

    visual_tools.deleteAllMarkers();
    visual_tools.publishText(text_pose, "Cartesian Path", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishTrajectoryLine(trajectory2, joint_model_group);
    visual_tools.publishPath(waypoints2, rvt::LIME_GREEN, rvt::SMALL);
    for (std::size_t i = 0; i < waypoints2.size(); ++i)
    visual_tools.publishAxisLabeled(waypoints2[i], "pt" + std::to_string(i), rvt::SMALL);
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
    move_group_interface.execute(trajectory2);


    // current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    //     // Little test rotation
    // joint_group_positions[0] = -1.61796;  // -1/6 turn in radians
    // move_group_interface.setJointValueTarget(joint_group_positions);

    // move_group_interface.setMaxVelocityScalingFactor(0.05);
    // move_group_interface.setMaxAccelerationScalingFactor(0.05);
    // bool success = (move_group_interface.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    // ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");

    // visual_tools.deleteAllMarkers();
    // visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
    // visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
    // visual_tools.trigger();
    // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the movement");

    // move_group_interface.execute(my_plan);


    // Move 1 
    // moveit::planning_interface::MoveGroupInterface::Plan my_plan1;
    // moveit::core::RobotStatePtr current_state = move_group_interface.getCurrentState();
    // current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
    // joint_group_positions[0] = -1.61796;
    // joint_group_positions[1] = -0.29098;
    // joint_group_positions[2] = -0.4777;
    // joint_group_positions[3] = -0.00527;
    // joint_group_positions[4] = -1.35359;
    // joint_group_positions[5] =  1.57767;
    // move_group_interface.setJointValueTarget(joint_group_positions);
    
    // bool success = (move_group_interface.plan(my_plan1) == moveit::core::MoveItErrorCode::SUCCESS);
    // ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");

    // visual_tools.prompt("Press 'next'");
    // move_group_interface.execute(my_plan1);

    // // Move 2
    // moveit::planning_interface::MoveGroupInterface::Plan my_plan2;
    // current_state2 = move_group_interface.getCurrentState();
    // moveit::planning_interface::MoveGroupInterface::Plan my_plan1;
    // moveit::core::RobotStatePtr current_state = move_group_interface.getCurrentState();
    // current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
    // joint_group_positions[0] = -1.61796;
    // joint_group_positions[1] = -0.29098;
    // joint_group_positions[2] = -0.4777;
    // joint_group_positions[3] = -0.00527;
    // joint_group_positions[4] = -1.35359;
    // joint_group_positions[5] =  1.57767;
    // move_group_interface.setJointValueTarget(joint_group_positions);
    
    // bool success = (move_group_interface.plan(my_plan1) == moveit::core::MoveItErrorCode::SUCCESS);
    // ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");

    // visual_tools.prompt("Press 'next'");
    // move_group_interface.execute(my_plan1);

    // // Move 2
    // moveit::planning_interface::MoveGroupInterface::Plan my_plan2;
    // current_state2 = move_group_interface.getCurrentState();
    
    // current_state2->copyJointGroupPositions(joint_model_group, joint_group_positions);
    // joint_group_positions[0] = -2.138; 
    // joint_group_positions[1] = 0.62289;
    // joint_group_positions[2] = -0.525377;
    // joint_group_positions[3] = -0.064922;
    // joint_group_positions[4] = -0.3937427;
    // joint_group_positions[5] = 2.1907088;
    // move_group_interface.setJointValueTarget(joint_group_positions);
    // success = (move_group_interface.plan(my_plan2) == moveit::core::MoveItErrorCode::SUCCESS);
    // ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");

    // visual_tools.prompt("Press 'next'");    visual_tools.publishText(text_pose, "Cartesian Path", rvt::WHITE, rvt::XLARGE);
    // visual_tools.publishTrajectoryLine(trajectory, joint_model_group);
    // visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
    // for (std::size_t i = 0; i < waypoints.size(); ++i)
    // visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);
    // visual_tools.trigger();
    // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

    // move_group_interface.execute(my_plan2);

    // // Move 3
    // moveit::planning_interface::MoveGroupInterface::Plan my_plan3;
    // current_state = move_group_interface.getCurrentState();
    // current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
    // joint_group_positions[0] = -1.61796;
    // joint_group_positions[1] = -0.29098;
    // joint_group_positions[2] = -0.4777;
    // joint_group_positions[3] = -0.00527;
    // joint_group_positions[4] = -1.35359;
    // joint_group_positions[5] =  1.57767;
    // move_group_interface.setJointValueTarget(joint_group_positions);
    // success = (move_group_interface.plan(my_plan3) == moveit::core::MoveItErrorCode::SUCCESS);
    // ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");

    // visual_tools.prompt("Press 'next'");

    // move_group_interface.execute(my_plan3);

    // // Move 4
    // moveit::planning_interface::MoveGroupInterface::Plan my_plan4;
    // current_state = move_group_interface.getCurrentState();
    // current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
    // joint_group_positions[0] = -0.24243;
    // joint_group_positions[1] = 1.07208;
    // joint_group_positions[2] = 0.31348;
    // joint_group_positions[3] = 0.050131;
    // joint_group_positions[4] = -0.79863;
    // joint_group_positions[5] = 0.200687;
    // move_group_interface.setJointValueTarget(joint_group_positions);
    // success = (move_group_interface.plan(my_plan4) == moveit::core::MoveItErrorCode::SUCCESS);
    // ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");

    // visual_tools.prompt("Press 'next'");

    // move_group_interface.execute(my_plan4);
    // current_state2->copyJointGroupPositions(joint_model_group, joint_group_positions);
    // joint_group_positions[0] = -2.138; 
    // joint_group_positions[1] = 0.62289;
    // joint_group_positions[2] = -0.525377;
    // joint_group_positions[3] = -0.064922;
    // joint_group_positions[4] = -0.3937427;
    // joint_group_positions[5] = 2.1907088;
    // move_group_interface.setJointValueTarget(joint_group_positions);
    // success = (move_group_interface.plan(my_plan2) == moveit::core::MoveItErrorCode::SUCCESS);
    // ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");

    // visual_tools.prompt("Press 'next'");

    // move_group_interface.execute(my_plan2);

    // // Move 3
    // moveit::planning_interface::MoveGroupInterface::Plan my_plan3;
    // current_state = move_group_interface.getCurrentState();
    // current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
    // joint_group_positions[0] = -1.61796;
    // joint_group_positions[1] = -0.29098;
    // joint_group_positions[2] = -0.4777;
    // joint_group_positions[3] = -0.00527;
    // joint_group_positions[4] = -1.35359;
    // joint_group_positions[5] =  1.57767;
    // move_group_interface.setJointValueTarget(joint_group_positions);
    // success = (move_group_interface.plan(my_plan3) == moveit::core::MoveItErrorCode::SUCCESS);
    // ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");

    // visual_tools.prompt("Press 'next'");

    // move_group_interface.execute(my_plan3);

    // // Move 4
    // moveit::planning_interface::MoveGroupInterface::Plan my_plan4;
    // current_state = move_group_interface.getCurrentState();
    // current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
    // joint_group_positions[0] = -0.24243;
    // joint_group_positions[1] = 1.07208;
    // joint_group_positions[2] = 0.31348;
    // joint_group_positions[3] = 0.050131;
    // joint_group_positions[4] = -0.79863;
    // joint_group_positions[5] = 0.200687;
    // move_group_interface.setJointValueTarget(joint_group_positions);
    // success = (move_group_interface.plan(my_plan4) == moveit::core::MoveItErrorCode::SUCCESS);
    // ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");

    // visual_tools.prompt("Press 'next'");

    // move_group_interface.execute(my_plan4);

}