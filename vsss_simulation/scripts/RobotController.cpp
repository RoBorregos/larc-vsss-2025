#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include "geometry_msgs/msg/twist.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "tf2_ros/transform_listener.h"
#include <tf2_ros/buffer.h>
#include "vsss_simulation/Kinematic.hpp"
#include "vsss_simulation/Line.hpp"
#include "vsss_simulation/UnivectorF.hpp"
#include "vsss_simulation/MsgConvert.hpp"
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


class Robot_Controller : public rclcpp::Node
{
  public:
    Robot_Controller()
    : Node("robot_controller")
    {
      this->declare_parameter<int>("number",  0);
      
      id = this->get_parameter("number").as_int();
      std::string robot_topic = "robot"+ std::to_string(id);
      
      ball_sub = this->create_subscription<nav_msgs::msg::Odometry>("/ball/odom", 10, std::bind(&Robot_Controller::refresh_ball_odom, this, _1));
      self_vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel",50 );
      imag_pub = this->create_publisher<geometry_msgs::msg::Vector3>("imaginary_position",50);

      main_timer = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&Robot_Controller::Main, this));

      tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
      RCLCPP_INFO(get_logger(), "Robot Node With ID: '%i' .", id);

      
    }

  private:
    void Main(){

        //Update classes
        for(int i  = 1; i <= 3; i++){
          string rName = "robot";
          rName += to_string(i);
          rName += "_base_link";
          geometry_msgs::msg::TransformStamped temp;

          try{
            temp = tf_buffer_->lookupTransform(
              "world",rName.c_str(), TimePointZero
            );
            robots[i].setTrans(temp);
          }catch(const TransformException & ex){
            RCLCPP_INFO(
            this->get_logger(), "Could not transform %s to %s: %s",
            "world", rName.c_str(), ex.what());
          }
        }
        

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

        Transform self_transform = robots[id].transform;
        //Set the optimal trayectory
        Vector3 goal; 
        VectorFromMSG(goal_.transform.translation, goal);
        Line optimalPath (ball_transform.getOrigin(), goal);
        //Get angle considering the ball as the objective;
        Vector3 robot_2_ball = self_transform.getOrigin() - ball_transform.getOrigin(); 
        //obtain the angle from the functions
        float robot_t_ball = atan2(robot_2_ball[1], robot_2_ball[0]);
        float theta_ball = phiTuf(robot_t_ball, self_transform.getOrigin(),ball_transform.getOrigin(),  optimalPath); 
        //transform the angle to a vector
        Vector3 vector2ball =  Theta2Vector(theta_ball);


        if(robots.size() == 1){
          //Publish if no enemy to search
          self_vel_pub->publish(robots[id].result_to_msg(vector2ball));
          return;
        }

        //Look up for nearest enemy
        int nearObstID = 0;
        for(auto rob: robots){
          if(rob.first == id){
            continue;
          }
          if(nearObstID == 0){
            nearObstID= rob.first;
            continue;
          }else{
            nearObstID = (robots[nearObstID].transform.getOrigin() - robots[id].transform.getOrigin()).length() >
                             (rob.second.transform.getOrigin() - robots[id].transform.getOrigin()).length() ? 
                                  rob.first : nearObstID;
          }
        }

        //Publish imaginary position for the vector grapher
        Vector3 imaginary_obst = getImagePos(robots[id],robots[nearObstID]);
        geometry_msgs::msg::Vector3 imaginary_position;
        MSGFromVector3(imaginary_obst, imaginary_position );
        imag_pub->publish(imaginary_position);
        float dist_2_imag = (imaginary_obst - self_transform.getOrigin()).length();

        //Get the angle coefficient of avoidance
        float theta_enemy = phiAuf(robots[id],robots[nearObstID]);
        // Join the angles
        float joined = phiCompose(theta_ball,theta_enemy, dist_2_imag);
        //Publish final 
        Vector3 result = Theta2Vector(joined);
        self_vel_pub->publish(robots[id].result_to_msg(result));

        
        //Needs Function to get union of theta_ball and theta_enemy
      // Avoid Each Obst
    }
    void refresh_ball_odom(const nav_msgs::msg::Odometry::SharedPtr msg) 
    {
      //printOdom(msg, this->get_logger());
      SetTransformFromOdom(msg, ball_transform);

    }


    unordered_map<int, Kinematic> robots;
    int id;
    Transform ball_transform;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ball_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr self_sub;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr self_vel_pub;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr imag_pub;
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


