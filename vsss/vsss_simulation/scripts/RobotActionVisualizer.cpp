// /ros/vsss_ws/src/vsss_simulation/scripts/RobotActionVisualizer.cpp
// Visualize a robot's action in RViz via markers.
// Usage: RobotActionVisualizer <robot_name>
// Subscribes to:
//   /<robot_name>/action_type   (std_msgs::msg::Int32)  - action type (e.g. 1=goto,2=goto,3=spin)
//   /<robot_name>/goal_pose    (geometry_msgs::msg::Pose2D) - 2D goal pose (x,y,theta)
//   /<robot_name>/spin_dir     (std_msgs::msg::Int32)  - spin direction: >0 CW, <0 CCW
// Publishes:
//   /<robot_name>/action_marker (visualization_msgs::msg::Marker)

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "vsss_simulation/msg/robot_action.hpp"

using std::placeholders::_1;

class RobotActionVisualizer : public rclcpp::Node
{
public:
    RobotActionVisualizer(const std::string & robot_name)
    : Node("robot_action_visualizer_" + robot_name),
        robot_name_(robot_name),
        last_action_type_(1),
        last_spin_dir_(false)
    {
        std::string ns = "/" + robot_name_;
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(ns + "/action_marker", 10);

        action_sub_ = this->create_subscription<vsss_simulation::msg::RobotAction>(
            ns + "/action", 10,
            std::bind(&RobotActionVisualizer::actionCallback, this, _1));
        RCLCPP_INFO(this->get_logger(), "Visualizer started for robot '%s'", robot_name_.c_str());
    }

private:
    void actionCallback(const vsss_simulation::msg::RobotAction::SharedPtr msg)
    {
        last_action_type_ = msg->type.data;
        if(last_action_type_== 3){
            last_spin_dir_ = msg->spin_direction.data;
        }else{
            last_pose_ = msg->objective;
        }

        publishMarker();
    }

    void publishMarker()
    {
        if (!marker_pub_) return;

        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = this->now();
        marker.ns = robot_name_ + "_action";
        marker.id = 0;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.lifetime = rclcpp::Duration::from_seconds(0); // persistent until overwritten

        if (last_action_type_ == 3) {
            // Spin action -> show color indicating direction and a small circular marker
            marker.type = visualization_msgs::msg::Marker::CYLINDER;
            marker.pose.position.x = last_pose_.x;
            marker.pose.position.y = last_pose_.y;
            marker.pose.position.z = 0.5;
            marker.scale.z = 0.25; // text height

            if (last_spin_dir_ ) {
                marker.color.r = 0.1f; marker.color.g = 0.2f; marker.color.b = 1.0f; marker.color.a = 1.0f;
            } else {
                marker.color.r = 1.0f; marker.color.g = 0.2f; marker.color.b = 1.0f; marker.color.a = 1.0f;
            } 
            marker_pub_->publish(marker);
            return;
        }

        // Default: display pose objective as an arrow
        marker.type = visualization_msgs::msg::Marker::ARROW;
        marker.scale.x = 0.4; // shaft length
        marker.scale.y = 0.08; // shaft diameter
        marker.scale.z = 0.08; // head diameter
        marker.pose.position.x = last_pose_.x;
        marker.pose.position.y = last_pose_.y;
        marker.pose.position.z = 0.05;

        // Build quaternion from theta (rotation about z)
        tf2::Quaternion q;
        q.setRPY(0, 0, last_pose_.theta);
        marker.pose.orientation.x = q.x();
        marker.pose.orientation.y = q.y();
        marker.pose.orientation.z = q.z();
        marker.pose.orientation.w = q.w();

        if(last_action_type_ == 1){
            marker.color.r = 0.0f; marker.color.g = 1.0f; marker.color.b = 0.0f; marker.color.a = 1.0f;
        }else{
            marker.color.r = 1.0f; marker.color.g = 0.2f; marker.color.b = 1.0f; marker.color.a = 1.0f;
        }


        marker_pub_->publish(marker);
    }

    std::string robot_name_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::Subscription<vsss_simulation::msg::RobotAction>::SharedPtr action_sub_;

    int last_action_type_;
    geometry_msgs::msg::Pose2D last_pose_;
    bool last_spin_dir_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    std::string robot_name = "robot";
    if (argc > 1) {
        robot_name = argv[1];
    }

    auto node = std::make_shared<RobotActionVisualizer>(robot_name);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}