#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <rclcpp/parameter_client.hpp>
#include <rclcpp/parameter_event_handler.hpp>
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


//Field Sizes
float field_width = 1.5f;
float field_height = 1.3f;
float goal_height = 0.4f;
float defender_height = 0.7f;


//Square on the front of the robot
vector<Vector3> square = Rectangle(Vector3(0.035,0,0), 0.015, 0.035);
//Thinks about the use of static transforms. but they seem over engennier for the same hardcoded values

vector<Vector3> field = Rectangle(Vector3(0,0,0), field_width, field_height);

//Triangle 

                          





void vector_2_pose(std::unique_ptr<geometry_msgs::msg::PoseStamped, std::default_delete<geometry_msgs::msg::PoseStamped>>& res, Vector3& dir, float angle){
  res->header.frame_id ="world";
  Quaternion q;
  q.setRPY(0,0,angle);
  res->pose.orientation = toMsg(q);
  res->pose.position.set__x(dir.x());
  res->pose.position.set__y(dir.y());
  return;

}



class Robot_Controller : public rclcpp::Node
{
  public:
    Robot_Controller()
    : Node("robot_controller")
    {

      this->declare_parameter<bool>("Robot_side", false);
      field_side = this->get_parameter("Robot_side").as_bool();
      this->declare_parameter<int>("number",  0);
      id = this->get_parameter("number").as_int();
      

      self_vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel",50 );
      imag_pub = this->create_publisher<geometry_msgs::msg::Vector3>("imaginary_position",50);
      robot_action_sub = this->create_subscription<vsss_simulation::msg::RobotAction>(
        "action", 50, 
        bind(&Robot_Controller::refresh_action,this,_1));

      main_timer = this->create_wall_timer(
        std::chrono::milliseconds(50), 
        std::bind(&Robot_Controller::Main, this));

      stuck_timer = this->create_wall_timer(
        std::chrono::milliseconds(300), 
        std::bind(&Robot_Controller::end_stuck, this));


      //Debuger arrowDirection
      robot_direction = this->create_publisher<geometry_msgs::msg::PoseStamped>("objective_pose", 10);


      tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
      RCLCPP_INFO(get_logger(), "Robot Node With ID: '%i' .", id);
      boxCollider = Polygon(square);
      field_box_Collider = Polygon(field);
      //Set the area depending on the side of the field
      int triangle_flip = field_side ? 1 : -1;
      vector<Vector3> triangle_area = {Vector3(-field_width/2, -field_height/2, 0) , Vector3(field_width/2, -field_height/2, 0) , Vector3(field_width/2 * triangle_flip, -goal_height/2, 0)};
      kickArea_down = Polygon(triangle_area);
      for(int i = 0; i < triangle_area.size(); i++){
        triangle_area[i].setY(-triangle_area[i].getY());  
      }
      kickArea_up = Polygon(triangle_area);

      //External Params Recieve Declaration
      param_event_sub_ = this->create_subscription<rcl_interfaces::msg::ParameterEvent>(
        "/parameter_events",
        10,
        bind(&Robot_Controller::update_parameters, this, placeholders::_1)
      );

      //Recieve All params
      // Initialize updatable parameters from the global parameter server
      {
        auto param_client = std::make_shared<rclcpp::SyncParametersClient>(this, "/global_parameter_server_node");
        if (!param_client->wait_for_service(std::chrono::seconds(2))) {
          RCLCPP_WARN(this->get_logger(), "Parameter service '/global_parameter_server_node' not available, using defaults");
        } else {
          std::vector<std::string> names;
          names.reserve(updatable_parameters.size());
          for (const auto &kv : updatable_parameters) {
            names.push_back(kv.first);
          }

          try {
            auto params = param_client->get_parameters(names);
            for (const auto &p : params) {
              const std::string name = p.get_name();
              if (updatable_parameters.find(name) != updatable_parameters.end()) {
                // use as_double() and cast to float
                *updatable_parameters[name] = static_cast<float>(p.as_double());
              }
            }
            RCLCPP_INFO(this->get_logger(), "Loaded parameters from '/global_parameter_server_node'");
          } catch (const std::exception &e) {
            RCLCPP_WARN(this->get_logger(), "Failed to get parameters: %s", e.what());
          }
        }
      }
    }

  private:

    void update_parameters(const rcl_interfaces::msg::ParameterEvent::SharedPtr event){
      if(event->node != "/global_parameter_server_node"){
        return;
      }
      for (const auto& changed_param : event->changed_parameters) {
        auto param = rclcpp::Parameter::from_parameter_msg (changed_param);
        const string name = param.get_name();
        if(updatable_parameters.find(name) != updatable_parameters.end()){
          *updatable_parameters[name] = static_cast<float>(param.get_value<double>());
        }
      }

    }


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
        //Set the variables recieved by the global_parameters_server
        robots[id].ANGULAR_PROPORTIONAL_CONSTANT =  kpAngular;
        robots[id].ANGULAR_INTEGRAL_CONSTANT = kiAngular;
        robots[id].LINEAR_CONSTANT = kLinear;
        //GoBack if Needed
        boxCollider.origin = robots[id].transform.getOrigin();
        Vector3 vectorR  =  quatRotate(robots[id].transform.getRotation(), Vector3(1,0,0));
        boxCollider.rotation = atan2(vectorR.y(), vectorR.x());
        boxCollider.translade();
        if(!field_box_Collider.fullInside(boxCollider) && type == 1){
          geometry_msgs::msg::Twist backwards;
          geometry_msgs::msg::Vector3 linear;
          linear.x = 0.2f;
          backwards.set__linear(linear);
          self_vel_pub->publish(backwards);
          stuck = true;
          stuck_timer->reset();
          return;
        }


        if(kickArea_down.isInside(objective_position)|| kickArea_up.isInside(objective_position)){
          type = 2;
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
        if(type == 2 && (objective_position - self_transform.getOrigin()).length()< 0.12){
          Vector3 tieso(0,1,0);
          self_vel_pub->publish(robots[id].orient_to_msg(tieso));
          //cout<<"Tiesing"<<endl;
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
          theta_obj = phiTuf(robot_t_obj, self_transform.getOrigin(),objective_position,  optimalPath,de, kr); 
          //transform the angle to a vector
          vector2ball =  Theta2Vector(theta_obj);

          if(robots.size() <= 1){
            //Publish if no enemy to search
            self_vel_pub->publish(robots[id].result_to_msg(vector2ball, type));

            auto msg = std::make_unique<geometry_msgs::msg::PoseStamped>();

            msg->header.stamp = this->now();
            vector_2_pose(msg, robots[id].transform.getOrigin(), theta_obj);
            robot_direction->publish(move(msg));


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
        Vector3 imaginary_obst = getImagePos(robots[id],robots[nearObstID], ko);
        geometry_msgs::msg::Vector3 imaginary_position;
        MSGFromVector3(imaginary_obst, imaginary_position );
        imag_pub->publish(imaginary_position);
        float dist_2_imag = (imaginary_obst - self_transform.getOrigin()).length();

        //Get the angle coefficient of avoidance
        float theta_enemy = phiAuf(robots[id],robots[nearObstID], ko);
        // Join the angles
        float joined = phiCompose(theta_obj,theta_enemy, dist_2_imag, d_min, delta__);
        //Publish final 
        Vector3 result = Theta2Vector(joined);
        self_vel_pub->publish(robots[id].result_to_msg(result, type));


        auto msg = std::make_unique<geometry_msgs::msg::PoseStamped>();

          msg->header.stamp = this->now();
          vector_2_pose(msg, robots[id].transform.getOrigin(), joined);
          robot_direction->publish(move(msg));
        
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
    bool field_side;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr self_vel_pub;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr imag_pub;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr robot_direction;
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
    Polygon kickArea_down;
    Polygon kickArea_up;

    bool stuck;
    rclcpp::TimerBase::SharedPtr stuck_timer;

    //External Preferences Declaration
    
    rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr param_event_sub_;
      //Campo Vectorialfield_side
    float de, kr, ko, d_min, delta__;
      //Constantes de Movimiento
    float kLinear, kiAngular, kpAngular;

    unordered_map<string, float*> updatable_parameters = {
      {"KLinear", &kLinear},
      {"KpAngular", &kpAngular},
      {"KiAngular", &kiAngular},
      {"Campo_DE", &de},
      {"Campo_KR", &kr},
      {"Enemigo_KO", &ko},
      {"Campo_deltaMin", &d_min},
      {"Campo_delta__", &delta__},

    };

    
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Robot_Controller>());
  rclcpp::shutdown();
  return 0;
}
