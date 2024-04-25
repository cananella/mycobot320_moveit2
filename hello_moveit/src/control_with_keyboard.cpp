#include <memory>
#include <iostream>
#include <sstream>
#include <string>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

int main(int argc, char* argv[]) {
    // Initialize ROS and create the Node
    rclcpp::init(argc, argv);
    auto const node = std::make_shared<rclcpp::Node>(
        "hello_moveit",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );

    // Create a ROS logger
    auto const logger = rclcpp::get_logger("hello_moveit");

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread spinner = std::thread([&executor]() { executor.spin(); });

    // Create the MoveIt MoveGroup Interface
    using moveit::planning_interface::MoveGroupInterface;
    auto move_group_interface = MoveGroupInterface(node, "mycobot_arm");

    int choice;
    geometry_msgs::msg::Pose target_pose;
    std::string input_line;
    bool exit_requested = false;
    tf2::Quaternion quat;

    while (!exit_requested) {
        auto current_pose = move_group_interface.getCurrentPose().pose;
        target_pose = current_pose;
        RCLCPP_INFO(node->get_logger(), "Current pose: %f %f %f %f %f %f %f",
            current_pose.position.x,
            current_pose.position.y,
            current_pose.position.z,
            current_pose.orientation.x,
            current_pose.orientation.y,
            current_pose.orientation.z,
            current_pose.orientation.w);

        std::cout << "Select action: [1] Adjust Position, [2] Adjust Orientation, [3] Adjust Both, [4] Adjust Z Position, [5] Adjust by Roll Pitch Yaw and XYZ, [0] Exit: ";
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
            case 4: // Adjust Z Position only
                std::cout << "Enter new z position (current z: " << target_pose.position.z << "): ";
                std::getline(std::cin, input_line);
                std::istringstream(input_line) >> target_pose.position.z;
                break;
            case 5: // Adjust by Roll Pitch Yaw and XYZ
                double roll, pitch, yaw, x, y, z;
                std::cout << "Enter roll pitch yaw x y z (in radians for angles): ";
                std::getline(std::cin, input_line);
                std::istringstream(input_line) >> roll >> pitch >> yaw >> x >> y >> z;

                // Convert roll, pitch, yaw to quaternion
                
                quat.setRPY(roll, pitch, yaw);
                tf2::convert(quat, target_pose.orientation);

                // Set target position
                target_pose.position.x = x;
                target_pose.position.y = y;
                target_pose.position.z = z;
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
