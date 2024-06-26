#include <memory>
#include <vector>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>


using namespace std;

int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "hello_moveit",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("hello_moveit");

  // Next step goes here

  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "mycobot_arm");
  auto hand_move_group_interface = MoveGroupInterface(node, "hand");

  


  ////// set pose whit position and orientation
  // Set a target Pose
  // auto const target_pose = []{
  //   geometry_msgs::msg::Pose msg;
  //   msg.orientation.w = 0.5;
  //   msg.orientation.x =-0.5;
  //   msg.orientation.y =0.5;
  //   msg.orientation.z =-0.5;
  //   msg.position.x = 0.27;
  //   msg.position.y = 0.008;
  //   msg.position.z = 0.3;
  //   return msg;
  // }();

  // Create a plan to that target pose
  // move_group_interface.setPoseTarget(target_pose);
  // auto const [success, plan] = [&move_group_interface]{
  //   moveit::planning_interface::MoveGroupInterface::Plan msg;
  //   auto const ok = static_cast<bool>(move_group_interface.plan(msg));
  //   return std::make_pair(ok, msg);
  // }();
  // // Execute the plan
  // if(success) {
  //   move_group_interface.execute(plan);
  // } else {
  //   RCLCPP_ERROR(logger, "Planning failed!");
  // }


  ////// pose control with setting name on srdf
  
  // auto target=move_group_interface.getNamedTargetValues("home");
  move_group_interface.setNamedTarget("ready");
  // hand_move_group_interface.setNamedTarget("open");

  auto const [success, plan] = [&move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // Execute the plan
  if(success) {
    move_group_interface.execute(plan);
  } else {
    RCLCPP_ERROR(logger, "Planning failed!");
  }


  // vector<double> pose_rpy=move_group_interface.getCurrentRPY();
  // for(auto& elem:pose_rpy) cout<<elem<<"  ";


  // // Shutdown ROS
  // rclcpp::shutdown();
  // return 0;
}