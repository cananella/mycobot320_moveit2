#include <memory>
#include <iostream>
#include <sstream>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>

int main(int argc, char* argv[]) {
    // Initialize ROS and create the Node
    rclcpp::init(argc, argv);
    auto const node = std::make_shared<rclcpp::Node>(
        "hello_moveit",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );

    // Create a ROS logger
    auto const logger = rclcpp::get_logger("hello_moveit");

    // Create the MoveIt MoveGroup Interface
    using moveit::planning_interface::MoveGroupInterface;
    auto move_group_interface = MoveGroupInterface(node, "mycobot_arm");

    int choice;
    geometry_msgs::msg::Pose target_pose;
    std::string input_line;
    bool exit_requested = false;

    while (!exit_requested) {
        std::cout << "Select action: [1] Adjust Position, [2] Adjust Orientation, [3] Adjust Both, [0] Exit: ";
        std::cin >> choice;
        std::cin.ignore();  // Ignore the newline character left in the input buffer

        switch (choice) {
            case 1: // Position only
                std::cout << "Enter target position (x y z): ";
                std::getline(std::cin, input_line);
                std::istringstream(input_line) >> target_pose.position.x >> target_pose.position.y >> target_pose.position.z;
                break;
            case 2: // Orientation only
                std::cout << "Enter target orientation (x y z w): ";
                std::getline(std::cin, input_line);
                std::istringstream(input_line) >> target_pose.orientation.x >> target_pose.orientation.y >> target_pose.orientation.z >> target_pose.orientation.w;
                break;
            case 3: // Both
                std::cout << "Enter target position and orientation (px py pz ox oy oz ow): ";
                std::getline(std::cin, input_line);
                std::istringstream(input_line) >> target_pose.position.x >> target_pose.position.y >> target_pose.position.z
                                               >> target_pose.orientation.x >> target_pose.orientation.y >> target_pose.orientation.z >> target_pose.orientation.w;
                break;
            case 0: // Exit
                exit_requested = true;
                continue;
            default:
                std::cout << "Invalid choice. Please select again." << std::endl;
                continue;
        }

        // Set the target Pose
        move_group_interface.setPoseTarget(target_pose);

        // Create a plan to that target pose
        auto const [success, plan] = [&move_group_interface] {
            moveit::planning_interface::MoveGroupInterface::Plan msg;
            auto const ok = static_cast<bool>(move_group_interface.plan(msg));
            return std::make_pair(ok, msg);
        }();

        // Execute the plan
        if (success) {
            move_group_interface.execute(plan);
            RCLCPP_INFO(logger, "Execution successful!");
        } else {
            RCLCPP_ERROR(logger, "Planning failed!");
        }
    }

    // Shutdown ROS
    rclcpp::shutdown();
    return 0;
}
