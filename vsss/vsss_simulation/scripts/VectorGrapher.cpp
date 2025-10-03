#include <memory>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "tf2_ros/transform_listener.h"
#include <vsss_simulation/UnivectorF.hpp>
#include <vsss_simulation/MsgConvert.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <chrono>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include "nav_msgs/msg/odometry.hpp"
using namespace tf2;
using namespace std;
using std::placeholders::_1;


class VectorGrapher : public rclcpp::Node {
public:
    VectorGrapher() : Node("VectorGrapher") {
        

        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "arrow_grid", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(30),
            std::bind(&VectorGrapher::publish_arrows, this));
        
        imaginary_sub = this->create_subscription<geometry_msgs::msg::Vector3>("robot1/imaginary_position",50, bind(&VectorGrapher::refresh_imag_pos,this,_1));
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    }

private:
    void publish_arrows() {
        //PreDraw
        geometry_msgs::msg::TransformStamped goal_, ball_;
        try {
          goal_ = tf_buffer_->lookupTransform("world", "goal_pos",TimePointZero);
          ball_ = tf_buffer_->lookupTransform("world", "sphere_link", TimePointZero);
          fromMsg(goal_.transform.translation, goal);
          fromMsg(ball_.transform, ball_transform );

        } catch (const TransformException & ex) {
          RCLCPP_INFO(
            this->get_logger(), "Could not transform %s to %s: %s",
            "world", "goal_pos", ex.what());
          return;
        }
        Line optimalPath (ball_transform.getOrigin(), goal);
        //FIll Array
        visualization_msgs::msg::MarkerArray marker_array;

        int id = 0;
        int grid_size = 25;
        float spacing = 0.03;

        for (int i = -grid_size; i <= grid_size; ++i) {
            for (int j = -grid_size; j <= grid_size; ++j) {
                visualization_msgs::msg::Marker marker;
                marker.header.frame_id = "world";
                marker.header.stamp = this->get_clock()->now();
                marker.ns = "arrow_grid";
                marker.id = id++;
                marker.type = visualization_msgs::msg::Marker::ARROW;
                marker.action = visualization_msgs::msg::Marker::ADD;

                // Set position
                marker.pose.position.x = i * spacing;
                marker.pose.position.y = j * spacing;
                marker.pose.position.z = 0.2;
              
                Vector3 pos (marker.pose.position.x, marker.pose.position.y, marker.pose.position.z);
                //get vector from pos to ball
                Vector3 pos_2_ball = pos - ball_transform.getOrigin();
                double pos_t_ball = atan2(pos_2_ball[1], pos_2_ball[0]);
                double theta_ball = phiTuf(pos_t_ball, pos, ball_transform.getOrigin(), optimalPath);
                //Distance from pos to enemy
                Vector3 distan_to_I = pos - imag_pos;
                float theta_enemy = atan2(distan_to_I[1], distan_to_I[0]);
                // Join the angles
                float joined = phiCompose(theta_ball,theta_enemy, distan_to_I.length());
                // Orientation (pointing in +X direction)
                Quaternion orientation;
                orientation.setRPY (0.0,0.0,joined);
                marker.pose.orientation.x = orientation.x();
                marker.pose.orientation.y = orientation.y();
                marker.pose.orientation.z = orientation.z();
                marker.pose.orientation.w = orientation.w();

                // Scale
                marker.scale.x = 0.02; // shaft length
                marker.scale.y = 0.007; // shaft diameter
                marker.scale.z = 0.015; // head diameter

                // Color
                marker.color.r = 1.0f;
                marker.color.g = 0.0f;
                marker.color.b = 1.0f;
                marker.color.a = 1.0f;

                marker.lifetime = rclcpp::Duration::from_seconds(0); 

                marker_array.markers.push_back(marker);
            }
        }

        if(!is_enemy){
          marker_pub_->publish(marker_array);
          return;
        }

        // 2. Add horizontal marker at imag_pos (NEW CODE)
        visualization_msgs::msg::Marker horizontal_marker;
        horizontal_marker.header.frame_id = "world";
        horizontal_marker.header.stamp = this->get_clock()->now();
        horizontal_marker.ns = "imaginary_marker";
        horizontal_marker.id = id++;  // Continue with next available ID
        horizontal_marker.type = visualization_msgs::msg::Marker::CYLINDER;
        horizontal_marker.action = visualization_msgs::msg::Marker::ADD;

        // Position (using imag_pos)
        horizontal_marker.pose.position.x = imag_pos[0];
        horizontal_marker.pose.position.y = imag_pos[1];
        horizontal_marker.pose.position.z = 0; 

        // Orientation (horizontal)
        Quaternion horiz_orientation;
        horiz_orientation.setRPY(0,0, M_PI/2);
        horizontal_marker.pose.orientation.x = horiz_orientation.x();
        horizontal_marker.pose.orientation.y = horiz_orientation.y();
        horizontal_marker.pose.orientation.z = horiz_orientation.z();
        horizontal_marker.pose.orientation.w = horiz_orientation.w();

        // Scale (flat cylinder)
        horizontal_marker.scale.x = 0.02;  // Diameter
        horizontal_marker.scale.y = 0.02;  // Diameter
        horizontal_marker.scale.z = 0.05; // Very thin (height)

        // Color (yellow)
        horizontal_marker.color.r = 1.0f;
        horizontal_marker.color.g = 1.0f;
        horizontal_marker.color.b = 0.0f;
        horizontal_marker.color.a = 1.0f;  // Slightly transparent

        horizontal_marker.lifetime = rclcpp::Duration::from_seconds(0.2);
        marker_array.markers.push_back(horizontal_marker);

        marker_pub_->publish(marker_array);
    }

    void refresh_imag_pos(const geometry_msgs::msg::Vector3::SharedPtr msg){
      fromMsg(*msg, imag_pos);
      is_enemy = true;
    }

    Transform ball_transform;
    Vector3 imag_pos;
    Vector3 goal; 
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr imaginary_sub;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    bool is_enemy =false;
};


int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VectorGrapher>());
    rclcpp::shutdown();
    return 0;
}
