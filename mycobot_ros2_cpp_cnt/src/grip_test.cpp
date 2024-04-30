#include <iostream>
#include <vector>
#include <tuple>
#include <string>
#include <algorithm>

#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <geometry_msgs/msg/pose.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <mycobot_interfaces/srv/detection_rq.hpp>
#include <moveit/kinematic_constraints/utils.h>
#include <cmath>

using namespace std::chrono_literals;
using namespace std;

#define CAMFOCUS_X 288
#define CAMFOCUS_Y 288

vector<tuple<string,float,float,float,float>> detected_cube;
geometry_msgs::msg::Pose target_pose;
geometry_msgs::msg::Pose current_pose;
sensor_msgs::msg::JointState arm_target_joint;
sensor_msgs::msg::JointState hand_target_joint;
int stacked_cube[4]={0,};
string cubes[4]={"orange","blue","purple","green"};
vector<tuple<float,float,float>> cube_drop_point;


bool planAndExecute(moveit::planning_interface::MoveGroupInterface& move_group_interface, moveit::planning_interface::MoveGroupInterface::Plan& plan, rclcpp::Logger logger) {
    bool success = static_cast<bool>(move_group_interface.plan(plan));
    if (success) {
        move_group_interface.execute(plan);
        RCLCPP_INFO(logger, "Execution successful!");
        return true;
    } else {
        RCLCPP_ERROR(logger, "Planning failed!");
        return false;
    }
}


tuple<double, double, double> quaternionToRPY(const geometry_msgs::msg::Quaternion& quaternion){
    tf2::Quaternion tf_quaternion;
    tf2::fromMsg(quaternion, tf_quaternion);

    double roll, pitch, yaw;
    tf2::Matrix3x3(tf_quaternion).getRPY(roll, pitch, yaw);
    return {roll, pitch, yaw};
}

geometry_msgs::msg::Quaternion rpyToQuaternion(double roll, double pitch, double yaw) {
    tf2::Quaternion quaternion;
    quaternion.setRPY(roll, pitch, yaw);

    geometry_msgs::msg::Quaternion msg_quaternion;
    msg_quaternion.x = quaternion.x();
    msg_quaternion.y = quaternion.y();
    msg_quaternion.z = quaternion.z();
    msg_quaternion.w = quaternion.w();

    return msg_quaternion;
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

tuple<bool,int,int,float> checkMidPoint(string target_color ,rclcpp::Node::SharedPtr detection_request_node, rclcpp::Logger logger){
    detectionRequest(detection_request_node,logger);
    int idx = 0, dy=0, dx=0;
    float theta=0;
    bool alignment_flag=false;

    for(auto& elem:detected_cube){

        if (get<0>(elem) == target_color){
            int new_dx = (get<1>(elem) - CAMFOCUS_X);
            int new_dy = (CAMFOCUS_Y - get<2>(elem));
            if(abs(dx)+abs(dy)<abs(new_dy)+abs(new_dx)){
                dy=new_dy;
                dx=new_dx;
                theta = get<3>(detected_cube[idx]);
            }
        }
    }

    if(abs(dx)<15 && abs(dy)<15) alignment_flag = true;
    return{alignment_flag,dx,dy,theta};
}

auto addCollision(moveit::planning_interface::MoveGroupInterface& move_group_interface){
        auto const collision_object = [frame_id =move_group_interface.getPlanningFrame()] {
        moveit_msgs::msg::CollisionObject collision_object;
        collision_object.header.frame_id = frame_id;
        collision_object.id = "conveyor";
        shape_msgs::msg::SolidPrimitive primitive;

        // Define the size of the box in meters
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[primitive.BOX_X] = 0.3;
        primitive.dimensions[primitive.BOX_Y] = 0.13;
        primitive.dimensions[primitive.BOX_Z] = 0.09;

        // Define the pose of the box (relative to the frame_id)
        geometry_msgs::msg::Pose box_pose;
        box_pose.orientation.w =1.0;
        box_pose.position.x = -0.333;
        box_pose.position.y = 0.12+0.13/2;
        box_pose.position.z = primitive.dimensions[primitive.BOX_Z]/2;

        collision_object.primitives.push_back(primitive);
        collision_object.primitive_poses.push_back(box_pose);
        

        shape_msgs::msg::SolidPrimitive left_side;
        shape_msgs::msg::SolidPrimitive right_side;
        left_side.type = left_side.BOX;
        left_side.dimensions.resize(3);
        left_side.dimensions[primitive.BOX_X] = 0.145;
        left_side.dimensions[primitive.BOX_Y] = 0.04;
        left_side.dimensions[primitive.BOX_Z] = 0.155;
        right_side=left_side;

        box_pose.orientation.w =1.0;
        box_pose.position.x = -0.35;
        box_pose.position.y = 0.10;
        box_pose.position.z = 0.0775;
        collision_object.primitives.push_back(left_side);
        collision_object.primitive_poses.push_back(box_pose);


        box_pose.orientation.w =1.0;
        box_pose.position.x = -0.4;
        box_pose.position.y = 0.27;
        box_pose.position.z = 0.0775;
        collision_object.primitives.push_back(right_side);
        collision_object.primitive_poses.push_back(box_pose);

        shape_msgs::msg::SolidPrimitive camera_frame;
        camera_frame.type = camera_frame.BOX;
        camera_frame.dimensions.resize(3);
        camera_frame.dimensions[primitive.BOX_X] = 0.02;
        camera_frame.dimensions[primitive.BOX_Y] = 0.32;
        camera_frame.dimensions[primitive.BOX_Z] = 0.32;

        box_pose.orientation.w =1.0;
        box_pose.position.x = -0.333;
        box_pose.position.y = 0.25;
        box_pose.position.z = 0.315;
        collision_object.primitives.push_back(camera_frame);
        collision_object.primitive_poses.push_back(box_pose);


        collision_object.operation = collision_object.ADD;
        return collision_object;
        }();
    return collision_object;
    }


    moveit_msgs::msg::CollisionObject addCubeCollision(moveit::planning_interface::MoveGroupInterface& move_group_interface, double cube_x, double cube_y, double cube_z) {
    // Initialize the collision object
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = move_group_interface.getPlanningFrame();
    collision_object.id = "cube";

    // Define the box size
    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
    primitive.dimensions.resize(3); // Box requires 3 dimensions
    primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_X] = 0.02; // Width
    primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y] = 0.02; // Depth
    primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z] = 0.02; // Height

    // Define the pose of the box
    geometry_msgs::msg::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = cube_x;
    box_pose.position.y = cube_y;
    box_pose.position.z = cube_z + primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z] / 2; // Position z plus half height

    // Assign primitive and pose to the collision object
    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = moveit_msgs::msg::CollisionObject::ADD;

    return collision_object;
}

void initSetting(){
    //cubes[4]={"orange","blue","purple","green"};
    
    cube_drop_point.push_back({0.225,0.089,0.155});
    cube_drop_point.push_back({0.225,0.195,0.155});
    cube_drop_point.push_back({0.143,0.195,0.155});
    cube_drop_point.push_back({0.047,0.195,0.155});
}

    


int main(int argc, char* argv[]) {  

    rclcpp::init(argc, argv);
    initSetting();


    auto node = make_shared<rclcpp::Node>("grip_test_node", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
    auto const logger =rclcpp::get_logger("grip_test_node");
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    thread spinner = thread([&executor]() { executor.spin(); });

    //cube detection 을 위한 노드 생성
    rclcpp::Node::SharedPtr detection_request_node = rclcpp::Node::make_shared("detection_request_node");

    using moveit::planning_interface::MoveGroupInterface;
    auto arm_move_group_interface = MoveGroupInterface(node, "mycobot_arm");
    auto hand_move_group_interface = MoveGroupInterface(node, "hand");
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    geometry_msgs::msg::Quaternion quat_msg;

    
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    auto objectname=planning_scene_interface.getKnownObjectNames();
    for (auto& elem:objectname) cout<< elem<< " \n";
    planning_scene_interface.removeCollisionObjects(objectname);

    auto collision_object=addCollision(arm_move_group_interface);
    planning_scene_interface.applyCollisionObject(collision_object);

    
    int stop_flag=0;
    while(!stop_flag){
        // arm 을 ready 위치로 이동
        arm_move_group_interface.setNamedTarget("grip_cube_ready");
        planAndExecute(arm_move_group_interface, plan, logger);
        this_thread::sleep_for(2s);

        string target_color;
        float center_x;
        float center_y;
        float target_theta;
        float m_per_pixel;
        bool detection_flag=true;
        while (detection_flag){
            detectionRequest(detection_request_node, logger);
            vector<tuple<string,float,float,float,float>> tempv;
            for(auto& elem:detected_cube){
                center_x = get<1>(elem);
                center_y = get<2>(elem);
                if (center_y<270||center_y>350||center_x<210||center_x>360) continue;
                tempv.push_back(elem);
            }
            if(tempv.size()==0){
                cout<<"noting detected\n";
                this_thread::sleep_for(2s);
                continue;
            }
            if(tempv.size()>1){
                cout<<"over 2 cube detected\n";
                this_thread::sleep_for(2s);
                continue;
            }
            target_color = get<0>(tempv[0]);
            center_x = get<1>(tempv[0]);
            center_y = get<2>(tempv[0]);
            target_theta = get<3>(tempv[0]);
            m_per_pixel = get<4>(tempv[0]);
            // int flag;
            cout<<"target box color: "<<target_color<<"   center point: ("<<center_x<<","<<center_y<<")   theta: "<<target_theta<<" \n";
            // cout<<"if right press 0 , detect again press 1 : ";
            // cin >>flag;
            // if (flag==0){
            //     detection_flag=false;
            // }    
            break;
        }
        detection_flag=true;
        
        // rotate alignment 
        // bool turn_right_flag=true;
        if (target_theta>45)target_theta=(target_theta-90)*M_PI/180;
        else target_theta=target_theta*M_PI/180;
        // target_theta = (target_theta>45)? (target_theta-90)*M_PI/180 : target_theta*M_PI/180;
        current_pose = arm_move_group_interface.getCurrentPose().pose;
        target_pose = current_pose;

        vector<string> jointname = arm_move_group_interface.getJointNames();
        vector<double> jointstate = arm_move_group_interface.getCurrentJointValues();
        int cnt=0;
        for(auto& elem:jointname){
            if(elem=="joint6output_to_joint6") jointstate[cnt]+=target_theta;
            cnt++;
        }
        arm_target_joint.name=jointname;
        arm_target_joint.position=jointstate;
        arm_move_group_interface.setJointValueTarget(arm_target_joint);
        planAndExecute(arm_move_group_interface, plan, logger);
        this_thread::sleep_for(2s);


        // hand open
        hand_move_group_interface.setNamedTarget("halfopen");
        planAndExecute(hand_move_group_interface,plan,logger);
        this_thread::sleep_for(1s);

        current_pose = arm_move_group_interface.getCurrentPose().pose;
        float end_Y = arm_move_group_interface.getCurrentRPY()[2];
        float dx=(center_x - CAMFOCUS_X) * m_per_pixel;
        float dy=(CAMFOCUS_Y - center_y) * m_per_pixel;


        float position_x = -0.233454 - dy;
        float position_y = 0.177755 + dx;

        RCLCPP_INFO(node->get_logger(), "from center Cube pose x,y,theta: %f %f %f",dy, dx, end_Y);
        RCLCPP_INFO(node->get_logger(), "Current pose: %f %f %f", current_pose.position.x, current_pose.position.y, current_pose.position.z);
        RCLCPP_INFO(node->get_logger(), "target pose xyz: %f %f %f", position_x, position_y, current_pose.position.z);
        //move pose

        //Cartesian path plan   
        std::vector<geometry_msgs::msg::Pose> waypoints;

        current_pose = arm_move_group_interface.getCurrentPose().pose;
        target_pose=current_pose;
        target_pose.position.x=position_x;
        target_pose.position.y=position_y;
        target_pose.position.z=0.29;
        waypoints.push_back(target_pose);


        target_pose.position.z-=0.024;
        waypoints.push_back(target_pose);

        
        moveit_msgs::msg::RobotTrajectory trajectory;
        const double jump_threshold = 0.0;
        const double eef_step = 0.001;
        double fraction = arm_move_group_interface.computeCartesianPath(waypoints,eef_step,jump_threshold,trajectory);
        arm_move_group_interface.execute(trajectory);

        this_thread::sleep_for(2s);



        hand_move_group_interface.setNamedTarget("grip_cube");
        planAndExecute(hand_move_group_interface,plan,logger);
        this_thread::sleep_for(1s);

        arm_move_group_interface.setNamedTarget("ready");
        planAndExecute(arm_move_group_interface,plan,logger);
        this_thread::sleep_for(2s);


        ////drop cube;

        waypoints.clear();

        current_pose = arm_move_group_interface.getCurrentPose().pose;
        target_pose=current_pose;
        waypoints.push_back(target_pose);
        float goal_z;
        for(int i=0; i<4; i++){
            if (target_color!=cubes[i]) continue;
            target_pose.position.x=get<0>(cube_drop_point[i]);
            target_pose.position.y=get<1>(cube_drop_point[i]);
            target_pose.position.z=0.25;
            goal_z=get<2>(cube_drop_point[i])+stacked_cube[i]*0.025 + 0.002;
            stacked_cube[i]++;
        }

        waypoints.push_back(target_pose);
        target_pose.position.z=goal_z;
        waypoints.push_back(target_pose);
        fraction = arm_move_group_interface.computeCartesianPath(waypoints,eef_step,jump_threshold,trajectory);
        RCLCPP_INFO(logger, "Visualizing plan 4 (Cartesian path) (%.2f%% achieved)", fraction * 100.0);
        arm_move_group_interface.execute(trajectory);
        this_thread::sleep_for(2s);

        hand_move_group_interface.setNamedTarget("halfopen");
        planAndExecute(hand_move_group_interface,plan,logger);
        this_thread::sleep_for(1s);
        
        //// ready,close
        arm_move_group_interface.setNamedTarget("ready");
        planAndExecute(arm_move_group_interface,plan,logger);
        this_thread::sleep_for(2s);
        
        hand_move_group_interface.setNamedTarget("close");
        planAndExecute(hand_move_group_interface,plan,logger);
        this_thread::sleep_for(1s);
        

        // cout<<"if want to stop press 1 , test again press 0 : ";
        // cin>>stop_flag;
        cout<<"drop & drop  done wait 2s";
        this_thread::sleep_for(2s);
    }
    rclcpp::shutdown();
    return 0;
}