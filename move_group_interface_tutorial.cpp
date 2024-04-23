#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_demo");
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node = rclcpp::Node::make_shared("move_group_interface_tutorial", node_options);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread([&executor]() { executor.spin(); }).detach();

  // MoveIt operates on sets of joints called "planning groups" and stores them in an object called
  // the ``JointModelGroup``. Throughout MoveIt, the terms "planning group" and "joint model group"
  // are used interchangeably.
  static const std::string PLANNING_GROUP = "panda_arm";
  moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  const moveit::core::JointModelGroup* joint_model_group =
  move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools(move_group_node, "panda_link0", "move_group_tutorial",
                                                      move_group.getRobotModel());
  visual_tools.deleteAllMarkers();
  visual_tools.loadRemoteControl();
  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.0;

  // We can print the name of the reference frame for this robot.
  RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group.getPlanningFrame().c_str());

  // We can also print the name of the end-effector link for this group.
  RCLCPP_INFO(LOGGER, "End effector link: %s", move_group.getEndEffectorLink().c_str());

  // We can get a list of all the groups in the robot:
  RCLCPP_INFO(LOGGER, "Available Planning Groups:");
  std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),
            std::ostream_iterator<std::string>(std::cout, ", "));

  //  we create an pointer that references the current robot's state.
  moveit::core::RobotStatePtr current_state = move_group.getCurrentState(10);
  std::vector<double> joint_group_positions;
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  double pi = 3.14;
  bool success;

  /////////////////////////////////////Joint-space positioning////////////////////////////////////////////////////////////////
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
  visual_tools.publishText(text_pose, "Joint-----space_pose_goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();

  // Now, let's modify just one of the joints.
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
  joint_group_positions[0] = 1.57;  // radians
  move_group.setJointValueTarget(joint_group_positions);

  // Change the allowed maximum velocity and acceleration to 20% of their maximum. The default values are 10% (0.1).
  move_group.setMaxVelocityScalingFactor(0.2);
  move_group.setMaxAccelerationScalingFactor(0.2);

  // compute the plan.
  success = static_cast<bool>(move_group.plan(my_plan));

  //execute the plan if successful.
  if(success) 
   move_group.execute(my_plan);
  else 
   RCLCPP_INFO(LOGGER, "Planning failed!");
       
  ///////////////////////////////////////Joint-space positioning//////////////////////////////////////////////////////////////////////////////
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");
  
  // Define a desired joint configuration
  std::vector<double> joint_values2 = {pi, 0, 0, -pi/2, 0, pi/2, pi/4};
  
  // Set the joint values for the MoveIt MoveGroup Interface
  move_group.setJointValueTarget(joint_values2);

  // Compute the plan and execute it if successful.
  success = static_cast<bool>(move_group.plan(my_plan));
  if(success) 
   move_group.execute(my_plan);
  else 
    RCLCPP_INFO(LOGGER, "Planning failed!");
    
  /////////////////////////////////////Joint-space positioning////////////////////////////////////////////////////////////////
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");
  
  // Go to the initial position 
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
  move_group.setJointValueTarget(joint_group_positions);

  success = static_cast<bool>(move_group.plan(my_plan));
  if(success) 
   move_group.execute(my_plan);
  else 
   RCLCPP_INFO(LOGGER, "Planning failed!");

/////////////////////////////////////////////////////////Finish//////////////////////////////////////////////////////////////////
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the collision object disappears");

  // END_TUTORIAL
  rclcpp::shutdown();
  return 0;
}

