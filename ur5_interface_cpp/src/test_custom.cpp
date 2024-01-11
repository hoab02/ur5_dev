#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Moveit_Interface_Node");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    static const std::string MANIPULATOR = "manipulator";

    moveit::planning_interface::MoveGroupInterface move_group_interface(MANIPULATOR);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
    

    const moveit::core::JointModelGroup *joint_model_group =
        move_group_interface.getCurrentState()->getJointModelGroup(MANIPULATOR);

    ROS_INFO_NAMED("tutorial", "Planning frame: %s", move_group_interface.getPlanningFrame().c_str());
    ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
    std::copy(move_group_interface.getJointModelGroupNames().begin(),
              move_group_interface.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", /n"));

    // vvvvv
    // Start
    // ^^^^^

    // Pose 0:
    move_group_interface.setNamedTarget("up");
    bool success = (move_group_interface.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan (pose 0) %s", success ? "SUCCESS" : "FAILED");
    move_group_interface.move();

    geometry_msgs::PoseStamped current_pose;
    // Pose 1:
    current_pose = move_group_interface.getCurrentPose(move_group_interface.getEndEffectorLink());
    geometry_msgs::Pose target_pose_1 = current_pose.pose;
    target_pose_1.position.x += 0.25;
    target_pose_1.position.y += 0.05;
    target_pose_1.position.z -= 0.2;
    move_group_interface.setPoseTarget(target_pose_1);
    success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "SUCCESS" : "FAILED");
    visual_tools.publishAxisLabeled(target_pose_1, "pose 1");
    // visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group); //!: Unable to get end effector tips from jmg
    visual_tools.trigger();
    move_group_interface.move();

    // Pose 2:
    current_pose = move_group_interface.getCurrentPose(move_group_interface.getEndEffectorLink());
    geometry_msgs::Pose target_pose_2 = current_pose.pose;
    target_pose_2.position.x += 0.3;
    target_pose_2.position.y += 0.2;
    target_pose_2.position.z -= 0.3;
    move_group_interface.setPoseTarget(target_pose_2);
    success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (pose goal) %s", success ? "SUCCESS" : "FAILED");
    visual_tools.publishAxisLabeled(target_pose_2, "pose 2");
    // visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group); //!: Unable to get end effector tips from jmg
    visual_tools.trigger();
    move_group_interface.move();

    auto plan_1_2 = my_plan;

    // Pose 3:
    current_pose = move_group_interface.getCurrentPose(move_group_interface.getEndEffectorLink());
    geometry_msgs::Pose target_pose_3 = current_pose.pose;
    target_pose_3.position.x -= 0.6;
    target_pose_3.position.y += 0.25;
    move_group_interface.setPoseTarget(target_pose_3);
    success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 3 (pose goal) %s", success ? "SUCCESS" : "FAILED");
    visual_tools.publishAxisLabeled(target_pose_3, "pose 3");
    // visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group); //!: Unable to get end effector tips from jmg
    visual_tools.trigger();
    move_group_interface.move();

    auto plan_2_3 = my_plan;

    // // Back to Pose 1:
    current_pose = move_group_interface.getCurrentPose(move_group_interface.getEndEffectorLink());
    move_group_interface.setPoseTarget(target_pose_1);
    success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 back (pose goal) %s", success ? "SUCCESS" : "FAILED");
    move_group_interface.move();

    auto plan_3_1 = my_plan;

    // Run saved plan
    move_group_interface.execute(plan_1_2);
    move_group_interface.execute(plan_2_3);
    move_group_interface.execute(plan_3_1);

    // Pose END:
    move_group_interface.setNamedTarget("home");
    success = (move_group_interface.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan (pose END) %s", success ? "SUCCESS" : "FAILED");
    move_group_interface.move();

    ros::waitForShutdown();
}