#include <memory>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <cstdlib>
#include <cstdio>
#include <unistd.h> // for getch() function on Unix/Linux

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>

#ifdef _WIN32
#include <conio.h> // for getch() function on Windows
#else
#include <termios.h>
#endif

// Cross-platform getch() implementation
int getch() {
#ifdef _WIN32
    return _getch();
#else
    struct termios oldt, newt;
    int ch;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    return ch;
#endif
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>(
        "hello_moveit",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );
    auto logger = rclcpp::get_logger("hello_moveit");

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread spinner = std::thread([&executor]() { executor.spin(); });

    moveit::planning_interface::MoveGroupInterface move_group_interface(node, "mycobot_arm");

    std::vector<geometry_msgs::msg::Pose> waypoints;
    geometry_msgs::msg::Pose target_pose;
    auto cur_pose=move_group_interface.getCurrentPose().pose;
    double x = cur_pose.position.x, y = cur_pose.position.y, z = cur_pose.position.z; // Initial position

    while (rclcpp::ok()) {
        // Check if a key is pressed
        int key = getch();

        // Update target position based on user input
        switch (key) {
            case 'w':
                y += 0.1; // Move forward
                break;
            case 's':
                y -= 0.1; // Move backward
                break;
            case 'a':
                x -= 0.1; // Move left
                break;
            case 'd':
                x += 0.1; // Move right
                break;
            case 'q':
                z += 0.1; // Move up
                break;
            case 'e':
                z -= 0.1; // Move down
                break;
            case '\n':
                // Stop execution for Enter key
                RCLCPP_INFO(logger, "Exiting...");
                goto exit_loop;
            default:
                // Ignore other keys
                continue;
        }

        // Update target pose
        target_pose.position.x = x;
        target_pose.position.y = y;
        target_pose.position.z = z;

        // Add the updated pose to waypoints
        waypoints.push_back(target_pose);

        // Prompt for adding more waypoints
        std::cout << "Add another waypoint? Press Enter to exit." << std::endl;
    }

exit_loop:
    moveit_msgs::msg::RobotTrajectory trajectory;
    const double jump_threshold = 0.5;
    const double eef_step = 0.01;
    double fraction = move_group_interface.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

    RCLCPP_INFO(logger, "Visualizing plan (Cartesian path) (%.2f%% achieved)", fraction * 100.0);

    // Cleanup
    rclcpp::shutdown();
    spinner.join();

    return 0;
}