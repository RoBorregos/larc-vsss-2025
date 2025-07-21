#include <memory>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "tf2_ros/transform_listener.h"
#include <vsss_simulation/UnivectorF.hpp>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include "nav_msgs/msg/odometry.hpp"
using namespace tf2;

using std::placeholders::_1;


void SetTransformFromOdom(const nav_msgs::msg::Odometry::SharedPtr& msg, Transform& o){
      Vector3 pos(
        msg->pose.pose.position.x,
        msg->pose.pose.position.y,
        msg->pose.pose.position.z
      );
      Quaternion rotation(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w
      );
      o.setOrigin(pos);
      o.setRotation(rotation);
      return;
}


void Vector3fromMSG(geometry_msgs::msg::TransformStamped&a, Vector3& o ){
  o[0] = a.transform.translation.x;
  o[1] = a.transform.translation.y;
  o[2] = a.transform.translation.z;
}


class VectorGrapher : public rclcpp::Node {
public:
    VectorGrapher() : Node("VectorGrapher") {
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "arrow_grid", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(30),
            std::bind(&VectorGrapher::publish_arrows, this));
        
      ball_sub = this->create_subscription<nav_msgs::msg::Odometry>("ball/odom", 10, std::bind(&VectorGrapher::refresh_ball_odom, this, _1));

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    }

private:
    void publish_arrows() {
        //PreDraw
        geometry_msgs::msg::TransformStamped goal_;
        try {
          goal_ = tf_buffer_->lookupTransform(
            "world", "goal_pos",
            TimePointZero);
        } catch (const TransformException & ex) {
          RCLCPP_INFO(
            this->get_logger(), "Could not transform %s to %s: %s",
            "world", "goal_pos", ex.what());
          return;
        }
      //Set the optimal trayectory
        Vector3 goal; 
        Vector3fromMSG(goal_,goal);
        Line optimalPath (ball_transform.getOrigin(), goal);

    //FIll Array
        visualization_msgs::msg::MarkerArray marker_array;

        int id = 0;
        int grid_size = 30;
        float spacing = 0.08;

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
                marker.pose.position.z = 0.5;
                Vector3 pos (marker.pose.position.x, marker.pose.position.y, marker.pose.position.z);
                Vector3 pos_2_ball = pos - ball_transform.getOrigin();
                double pos_t_ball = atan2(pos_2_ball[1], pos_2_ball[0]);
                double theta_res = phiTuf(pos_t_ball, pos, ball_transform.getOrigin(), optimalPath);

                // Orientation (pointing in +X direction)
                Quaternion orientation;
                orientation.setRPY (0.0,0.0,theta_res);
                marker.pose.orientation.x = orientation.x();
                marker.pose.orientation.y = orientation.y();
                marker.pose.orientation.z = orientation.z();
                marker.pose.orientation.w = orientation.w();

                // Scale
                marker.scale.x = 0.05; // shaft length
                marker.scale.y = 0.015; // shaft diameter
                marker.scale.z = 0.03; // head diameter

                // Color
                marker.color.r = 1.0f;
                marker.color.g = 0.0f;
                marker.color.b = 1.0f;
                marker.color.a = 1.0f;

                marker.lifetime = rclcpp::Duration::from_seconds(0.2); 

                marker_array.markers.push_back(marker);
            }
        }

        marker_pub_->publish(marker_array);
    }

    void refresh_ball_odom(const nav_msgs::msg::Odometry::SharedPtr msg) 
    {
      //printOdom(msg, this->get_logger());
      SetTransformFromOdom(msg, ball_transform);

    }

    Transform ball_transform;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ball_sub;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VectorGrapher>());
    rclcpp::shutdown();
    return 0;
}
