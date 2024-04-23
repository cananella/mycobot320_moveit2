#include<rclcpp/rclcpp.hpp>
#include<visualization_msgs/msg/marker.hpp>
#include<chrono>
#include<functional>
#include<string>
#include<tuple>

using namespace std::chrono_literals;

class MarkerPub: public rclcpp::Node{
    private:
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
    public:
        void markerCB(){
            auto marker = visualization_msgs::msg::Marker();
            marker.header.frame_id = "world";
            marker.header.stamp = this->now();
            marker.ns = "object";
            marker.id = 0;
            marker.type = visualization_msgs::msg::Marker::CUBE;
            marker.action =  visualization_msgs::msg::Marker::ADD;
            marker.pose.position.x = 1;
            marker.pose.position.y = 1;
            marker.pose.position.z = 1;
            marker.scale.x = 1.0;
            marker.scale.y = 1.0;
            marker.scale.z = 1.0;
            marker.color.a = 1.0;
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            publisher_->publish(marker);
        }

        MarkerPub():Node("marker_test"){
            publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("marker", 10);
            timer_ = this->create_wall_timer(
            20ms, std::bind(&MarkerPub::markerCB, this)
            );
        }
};



int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MarkerPub>());
    rclcpp::shutdown();


    return 0;
}
