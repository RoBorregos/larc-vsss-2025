#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include "geometry_msgs/msg/twist.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "tf2_ros/transform_listener.h"
#include <tf2_ros/buffer.h>
#include "vsss_simulation/Kinematic.hpp"
#include "vsss_simulation/Line.hpp"
#include "vsss_simulation/UnivectorF.hpp"
#include <iostream>  
#include <string> 
using std::placeholders::_1;
using namespace tf2;
using namespace std;


void printOdom(const nav_msgs::msg::Odometry::SharedPtr  & msg, const rclcpp::Logger & logger)
{
    const auto & pos = msg->pose.pose.position;
    const auto & ori = msg->pose.pose.orientation;

    RCLCPP_INFO(logger, "  Position:     (%.3f, %.3f, %.3f)", pos.x, pos.y, pos.z);
    RCLCPP_INFO(logger, "  Orientation:  (x=%.3f, y=%.3f, z=%.3f, w=%.3f)", ori.x, ori.y, ori.z, ori.w);
}

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


class Robot_Controller : public rclcpp::Node
{
  public:
    Robot_Controller()
    : Node("robot_controller")
    {
      this->declare_parameter<int>("number",  0);
      
      id = this->get_parameter("number").as_int();
      std::string robot_topic = "robot"+ std::to_string(id);
      
      ball_sub = this->create_subscription<nav_msgs::msg::Odometry>("ball/odom", 10, std::bind(&Robot_Controller::refresh_ball_odom, this, _1));
      self_sub = this->create_subscription<nav_msgs::msg::Odometry>(robot_topic+"/odom", 10, std::bind(&Robot_Controller::refresh_self_odom, this, _1));

      self_vel_pub = this->create_publisher<geometry_msgs::msg::Twist>(robot_topic+"/cmd_vel",500 );
      main_timer = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&Robot_Controller::Main, this));

      tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
      RCLCPP_INFO(get_logger(), "Robot Node With ID: '%i' .", id);

      
    }

  private:
    void Main(){
        //Update class 

        kinematic.setTrans(self_transform);
        

        //Look for the goal

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
        //Get angle considering the ball as the objective;
        Vector3 robot_2_ball = self_transform.getOrigin() - ball_transform.getOrigin(); 
        //obtain the angle from the functions
        float robot_t_ball = atan2(robot_2_ball[1], robot_2_ball[0]);
        float theta_res = phiTuf(robot_t_ball, self_transform.getOrigin(),ball_transform.getOrigin(),  optimalPath); 
        //transform the angle to a vector
        Vector3 result =  Theta2Vector(theta_res);
        //Publish
        self_vel_pub->publish(kinematic.result_to_msg(result));
    }
    void refresh_ball_odom(const nav_msgs::msg::Odometry::SharedPtr msg) 
    {
      //printOdom(msg, this->get_logger());
      SetTransformFromOdom(msg, ball_transform);

    }
    void refresh_self_odom(const nav_msgs::msg::Odometry::SharedPtr msg) 
    {
      SetTransformFromOdom(msg, self_transform);
    }

    Kinematic kinematic;
    int id;
    Transform ball_transform;
    Transform self_transform;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ball_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr self_sub;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr self_vel_pub;
    rclcpp::TimerBase::SharedPtr main_timer;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Robot_Controller>());
  rclcpp::shutdown();
  return 0;
}


