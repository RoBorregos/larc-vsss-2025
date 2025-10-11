#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "tf2_ros/transform_listener.h"
#include <tf2_ros/buffer.h>
#include "vsss_simulation/Kinematic.hpp"
#include "vsss_simulation/Line.hpp"
#include "vsss_simulation/UnivectorF.hpp"
#include "vsss_simulation/MsgConvert.hpp"
#include "vsss_simulation/msg/robot_action.hpp"
#include "vsss_simulation/Polygon.hpp"
#include "vsss_simulation/MathU.hpp"
#include <iostream>  
#include <string> 
using std::placeholders::_1;
using namespace tf2;
using namespace std;

//Square on the front of the robot
vector<Vector3> square = Rectangle(Vector3(0.035,0,0), 0.035, 0.035);
//Thinks about the use of static transforms. but they seem over engennier for the same hardcoded values

vector<Vector3> field = Rectangle(Vector3(0,0,0), 1.5, 1.3);
                          


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
      

      self_vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel",50 );
      imag_pub = this->create_publisher<geometry_msgs::msg::Vector3>("imaginary_position",50);
      robot_action_sub = this->create_subscription<vsss_simulation::msg::RobotAction>(
        "action", 50, bind(&Robot_Controller::refresh_action,this,_1));

      main_timer = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&Robot_Controller::Main, this));

      stuck_timer = this->create_wall_timer(std::chrono::milliseconds(300), std::bind(&Robot_Controller::end_stuck, this));

      tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
      RCLCPP_INFO(get_logger(), "Robot Node With ID: '%i' .", id);
      boxCollider = Polygon(square);
      field_box_Collider = Polygon(field);

      
    }

  private:

    void end_stuck(){
      stuck  = false;
      stuck_timer->cancel();
    }  

    void Main(){
        //Update classes
        for(int i  = 1; i <= 3; i++){
          string rName = "robot";
          rName += to_string(i);
          rName += "_base_link";
          geometry_msgs::msg::TransformStamped temp;

          try{
            temp = tf_buffer_->lookupTransform( "world",rName.c_str(), TimePointZero );
            robots[i].setTrans(temp);
          }catch(const TransformException & ex){
            //RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s","world", rName.c_str(), ex.what());
          }
        }

        //GoBack if Needed
        boxCollider.origin = robots[id].transform.getOrigin();
        Vector3 vectorR  =  quatRotate(robots[id].transform.getRotation(), Vector3(1,0,0));
        boxCollider.rotation = atan2(vectorR.y(), vectorR.x());
        boxCollider.translade();
        if(!field_box_Collider.fullInside(boxCollider) && type == 1){
          geometry_msgs::msg::Twist backwards;
          geometry_msgs::msg::Vector3 linear;
          linear.x = 0.4f;
          backwards.set__linear(linear);
          self_vel_pub->publish(backwards);
          stuck = true;
          stuck_timer->reset();
          return;
        }


        //Spin if needed
        if(type == 3){
          geometry_msgs::msg::Twist gira_gira;
          gira_gira.angular.set__z(12 *( direction? 1 : -1));
          self_vel_pub->publish(gira_gira);
          return;
        }
        if(stuck && type == 1){
          return;
        }
        Transform self_transform = robots[id].transform;
        //if the objective is near, just achieve its rotation
        if(type == 2 && (objective_position - self_transform.getOrigin()).length()< 0.08){
          Vector3 tieso(0,1,0);
          self_vel_pub->publish(robots[id].orient_to_msg(tieso));
          cout<<"Tiesing"<<endl;
          return;

        }
        Vector3 vector2ball;
        float theta_obj ;

        //attack with angle or just achieve a position (attack or defend)
        if(type== 1){
          Line optimalPath (objective_position, theta);
          //Get angle considering the ball as the objective;
          Vector3 robot_2_obj = self_transform.getOrigin() - objective_position; 
          //obtain the angle from the functions
          float robot_t_obj = atan2(robot_2_obj[1], robot_2_obj[0]);
          theta_obj = phiTuf(robot_t_obj, self_transform.getOrigin(),objective_position,  optimalPath); 
          //transform the angle to a vector
          vector2ball =  Theta2Vector(theta_obj);
        
          if(robots.size() <= 1){
            //Publish if no enemy to search
            self_vel_pub->publish(robots[id].result_to_msg(vector2ball, type));
            return;
          }
        }else{
          vector2ball = (objective_position - self_transform.getOrigin()).normalize();
          theta_obj = atan2(vector2ball[1], vector2ball[0]);
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
        float joined = phiCompose(theta_obj,theta_enemy, dist_2_imag);
        //Publish final 
        Vector3 result = Theta2Vector(joined);
        self_vel_pub->publish(robots[id].result_to_msg(result, type));

        
        //Needs Function to get union of theta_obj and theta_enemy
      // Avoid Each Obst
    }
    
    void refresh_action(const vsss_simulation::msg::RobotAction m){
      type = m.type.data;
      direction = m.spin_direction.data;
      objective_position.setX( m.objective.x);
      objective_position.setY( m.objective.y);
      theta = m.objective.theta;
      
    }

    unordered_map<int, Kinematic> robots;
    int id;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr self_vel_pub;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr imag_pub;
    rclcpp::Subscription<vsss_simulation::msg::RobotAction>::SharedPtr robot_action_sub;
    rclcpp::TimerBase::SharedPtr main_timer;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    //action varaibles
    int type;
    bool direction;
    Vector3 objective_position;
    float theta;

    //BoxCollider
    Polygon boxCollider;
    Polygon field_box_Collider;
    bool stuck;
    rclcpp::TimerBase::SharedPtr stuck_timer;

    
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Robot_Controller>());
  rclcpp::shutdown();
  return 0;
}


