#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <cmath>

std::shared_ptr<rclcpp::Node> node;
geometry_msgs::msg::Pose current_pose ;

class DetectionVisualizer : public rclcpp::Node{
  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr subscription_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  vision_msgs::msg::Detection2DArray::SharedPtr detection_msg_;

public:
  DetectionVisualizer() : Node("detection_visualizer"){
    subscription_ = this->create_subscription<vision_msgs::msg::Detection2DArray>(
      "cube_detections", 10, std::bind(&DetectionVisualizer::detectionArrayCallback, this, std::placeholders::_1));
    marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 10);
    timer_ = this->create_wall_timer(
                std::chrono::milliseconds(1000), std::bind(&DetectionVisualizer::publishMarkers, this));
  }

private:

  void detectionArrayCallback(const vision_msgs::msg::Detection2DArray::SharedPtr msg) {
        // Store the detection message for later publishing
        detection_msg_ = msg;
    }

    void publishMarkers() {
        if (!detection_msg_) {
            return; // No detection message received yet, do nothing
        }

        // Clear previous markers
        marker_publisher_->publish(visualization_msgs::msg::Marker()); // Publish an empty marker to clear previous ones

        for (const auto &detection : detection_msg_->detections) {
            // Extract x, y size data from each Detection2D message
            // Here you would write the logic to extract x, y data from each detection

            // Create a marker message for each detection
            float m_per_pixel = 0.025 / detection.bbox.size_y;
            auto marker = std::make_shared<visualization_msgs::msg::Marker>();
            marker->header.frame_id = "world"; // Set the frame ID appropriately
            marker->header.stamp = this->now();
            marker->ns = "detections";
            marker->type = visualization_msgs::msg::Marker::CUBE;
            marker->action = visualization_msgs::msg::Marker::ADD;

            // Set marker pose and scale based on extracted x, y size data
            float dx=(detection.bbox.center.position.x - 345) * m_per_pixel;
            float dy=(93 - detection.bbox.center.position.y) * m_per_pixel;
            float theta = atan2(current_pose.position.y,current_pose.position.x)-1.570796;

            marker->pose.position.x =dx*cos(theta)-dy*sin(theta)+current_pose.position.x;
                    // (detection.bbox.center.position.x - 345) * m_per_pixel + current_pose.position.x; // Set x position
            marker->pose.position.y =dx*sin(theta)+dy*cos(theta)+current_pose.position.y;
                    // (93 - detection.bbox.center.position.y) * m_per_pixel + current_pose.position.y; // Set y position
            marker->pose.position.z = 0; // Set z position (you may need to adjust this)
            marker->pose.orientation.w = detection.bbox.center.theta;
            marker->scale.x = 0.025;    // Set x size
            marker->scale.y = 0.025;    // Set y size
            marker->scale.z = 0.025;    // Set z size (you may need to adjust this)

            // Set marker color, you can customize this as needed
            marker->color.r = 1.0;
            marker->color.g = 0.0;
            marker->color.b = 0.0;
            marker->color.a = 1.0;
            
            RCLCPP_INFO(node->get_logger(), "Current pose: %f %f %f ",
              marker->pose.position.x,
              marker->pose.position.y,
              marker->pose.position.z
            );
            // RCLCPP_INFO(node->get_logger(), "m/p x y: %f %f %f ",
            //   m_per_pixel,
            //   (detection.bbox.center.position.x - 345) * m_per_pixel,
            //   (93 - detection.bbox.center.position.y) * m_per_pixel
            // );

            // Publish the marker for each detection
            marker_publisher_->publish(*marker);
        }

        detection_msg_.reset(); // Reset detection message after publishing
    }

};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  node = std::make_shared<rclcpp::Node>("get_eef_pose",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner = std::thread([&executor]() { executor.spin(); });


  moveit::planning_interface::MoveGroupInterface move_group_interface =
  moveit::planning_interface::MoveGroupInterface(node, "mycobot_arm");
  current_pose = move_group_interface.getCurrentPose().pose;

  RCLCPP_INFO(node->get_logger(), "Current pose: %f %f %f %f %f %f %f",
    current_pose.position.x,
    current_pose.position.y,
    current_pose.position.z,
    current_pose.orientation.x,
    current_pose.orientation.y,
    current_pose.orientation.z,
    current_pose.orientation.w
  );


  auto marker_node = std::make_shared<DetectionVisualizer>();
  rclcpp::spin(marker_node);
  rclcpp::shutdown();
  return 0;
}