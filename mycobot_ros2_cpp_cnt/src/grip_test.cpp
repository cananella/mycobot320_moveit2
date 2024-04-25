#include <iostream>
#include <vector>
#include <tuple>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <geometry_msgs/msg/pose.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <mycobot_interfaces/srv/detection_rq.hpp>
#include <cmath>

using namespace std::chrono_literals;
using namespace std;

vector<tuple<string,float,float,float,float>> detected_cube;

void planAndExecute(moveit::planning_interface::MoveGroupInterface& arm_move_group_interface, moveit::planning_interface::MoveGroupInterface::Plan& plan, rclcpp::Logger logger) {
    bool success = static_cast<bool>(arm_move_group_interface.plan(plan));
    if (success) {
        arm_move_group_interface.execute(plan);
        RCLCPP_INFO(logger, "Execution successful!");
    } else {
        RCLCPP_ERROR(logger, "Planning failed!");
    }
}
tuple<double, double, double> quaternionToRPY(const geometry_msgs::msg::Quaternion& quaternion){
    tf2::Quaternion tf_quaternion;
    tf2::fromMsg(quaternion, tf_quaternion);

    double roll, pitch, yaw;
    tf2::Matrix3x3(tf_quaternion).getRPY(roll, pitch, yaw);
    return {roll, pitch, yaw};
}


void detectionRequest(rclcpp::Node::SharedPtr detection_request_node, rclcpp::Logger logger) {
    detected_cube.clear();

    auto detection_client = detection_request_node->create_client<mycobot_interfaces::srv::DetectionRQ>("color_detection");
    auto detection_request = make_shared<mycobot_interfaces::srv::DetectionRQ::Request>();

    detection_request->trigger = true;
    if (!detection_client->wait_for_service(1s)) {
        RCLCPP_ERROR(logger, "Service not available, exiting.");
        return;
    }

    auto detection_result = detection_client->async_send_request(detection_request);

    if (rclcpp::spin_until_future_complete(detection_request_node, detection_result) == rclcpp::FutureReturnCode::SUCCESS){
        auto detections = detection_result.get()->result.detections;
        for (auto &detection : detections){
            auto color = detection.id;
            auto centerx = detection.bbox.center.position.x;
            auto centery = detection.bbox.center.position.y;
            auto angle = detection.bbox.center.theta;
            float m_per_pixel = (0.025/detection.bbox.size_y + 0.025/detection.bbox.size_y)/2;
            detected_cube.push_back({color, centerx, centery, angle, m_per_pixel});
        }
    } else {
        RCLCPP_ERROR(logger, "Failed to call service detection");
    }
}

tuple<bool,int,int> checkMidPoint(string target_color ,rclcpp::Node::SharedPtr detection_request_node, rclcpp::Logger logger){
    detectionRequest(detection_request_node,logger);
    int idx = 0, dy=0, dx=0;
    bool alignment_flag=false;
    for(auto& elem:detected_cube){
        if (get<0>(elem)== target_color){
            int new_dx = (get<1>(detected_cube[idx]) - 345);
            int new_dy = (93 - get<2>(detected_cube[idx]));
            if(abs(dx)+abs(dy)<abs(new_dy)+abs(new_dx)){
                dy=new_dy;
                dx=new_dx;
            }
        }
        else detected_cube.erase(detected_cube.begin()+idx);
        idx++;
    }
    if(abs(dx)<25&&abs(dy)<25) alignment_flag = true;

    return{alignment_flag,dx,dy};
}

int main(int argc, char* argv[]) {

    rclcpp::init(argc, argv);

    auto node = make_shared<rclcpp::Node>("grip_test_node", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

    auto const logger =rclcpp::get_logger("grip_test_node");
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    thread spinner = thread([&executor]() { executor.spin(); });

    using moveit::planning_interface::MoveGroupInterface;
    auto arm_move_group_interface = MoveGroupInterface(node, "mycobot_arm");
    auto hand_move_group_interface = MoveGroupInterface(node, "hand");

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    geometry_msgs::msg::Quaternion quat_msg;
    geometry_msgs::msg::Pose target_pose;


    arm_move_group_interface.setNamedTarget("ready");
    planAndExecute(arm_move_group_interface, plan, logger);
    this_thread::sleep_for(3s);

    
    rclcpp::Node::SharedPtr detection_request_node = rclcpp::Node::make_shared("detection_request_node");

    detectionRequest(detection_request_node, logger);

    cout<<"select cube\n";
    int idx=0;
    for(auto& elem:detected_cube){
        cout<<"no."<<idx<<"   color: "<<get<0>(elem)<<"   center point: ("<<get<1>(elem)<<","<<get<2>(elem)<<")   theta: "<<get<3>(elem)<<" \n";
        idx++;
    }
    cin>>idx;
    cout<<"no."<<idx<<"   color: "<<get<0>(detected_cube[idx])<<"   center point: ("<<get<1>(detected_cube[idx])<<","<<get<2>(detected_cube[idx])<<") selected";
    cout<<"\n";

    string target_color = get<0>(detected_cube[idx]);

    geometry_msgs::msg::Pose current_pose = arm_move_group_interface.getCurrentPose().pose;
    float end_Y = get<2>(quaternionToRPY(current_pose.orientation));
    float m_per_pixel = get<4>(detected_cube[idx]);
    float dx=(get<1>(detected_cube[idx]) - 345) * m_per_pixel;
    float dy=(93 - get<2>(detected_cube[idx])) * m_per_pixel;
    float position_x=dx*cos(end_Y)-dy*sin(end_Y)+current_pose.position.x;
    float position_y=dx*sin(end_Y)+dy*cos(end_Y)+current_pose.position.y;


    // RCLCPP_INFO(node->get_logger(), "Cube pose: %f %f %f",position_x, position_y, end_Y);
    // RCLCPP_INFO(node->get_logger(), "Current pose: %f %f %f %f %f %f %f", current_pose.position.x, current_pose.position.y, current_pose.position.z, current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w);
    
    target_pose=current_pose;
    target_pose.position.x=position_x;
    target_pose.position.y=position_y;
    RCLCPP_INFO(node->get_logger(), "target pose: %f %f %f %f %f %f %f", target_pose.position.x, target_pose.position.y, target_pose.position.z, target_pose.orientation.x, target_pose.orientation.y, target_pose.orientation.z, target_pose.orientation.w);



    arm_move_group_interface.setPoseTarget(target_pose);
    planAndExecute(arm_move_group_interface, plan, logger);
    this_thread::sleep_for(3s);

    bool alignment_flag=false;
    while(!alignment_flag){
        RCLCPP_INFO(node->get_logger(),"on alignment between arm and cube");
        tuple<bool,int,int>alignment_data=checkMidPoint(target_color,detection_request_node,logger);
        alignment_flag= get<0>(alignment_data);
        if (alignment_flag) continue;

        geometry_msgs::msg::Pose current_pose = arm_move_group_interface.getCurrentPose().pose;
        float end_Y = get<2>(quaternionToRPY(current_pose.orientation));
        float m_per_pixel = get<4>(detected_cube[idx]);
        float dx=get<1>(alignment_data) * m_per_pixel;
        float dy=get<2>(alignment_data) * m_per_pixel;
        float position_x=dx*cos(end_Y)-dy*sin(end_Y)+current_pose.position.x;
        float position_y=dx*sin(end_Y)+dy*cos(end_Y)+current_pose.position.y;

        target_pose=current_pose;
        target_pose.position.x=position_x;
        target_pose.position.y=position_y;
        arm_move_group_interface.setPoseTarget(target_pose);
        planAndExecute(arm_move_group_interface, plan, logger);
    }
    RCLCPP_INFO(node->get_logger(),"done alignment");

    

    rclcpp::shutdown();
    return 0;
}