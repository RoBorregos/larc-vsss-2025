#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/convert.h>
#include <std_msgs/msg/int64.hpp>
#include <std_msgs/msg/bool.hpp>
#include "vsss_simulation/msg/robot_action.hpp"
#include "vsss_simulation/Line.hpp"
#include "vsss_simulation/Kinematic.hpp"
#include "vsss_simulation/MsgConvert.hpp"
#include <vector>
#include <string>
using namespace std;
using namespace tf2;
class Strategist : public rclcpp::Node
{
public:
    Strategist(): Node("Strategist")
    {
        
        this->declare_parameter<int>("Robot_count",  0);

        robot_count = this->get_parameter("Robot_count").as_int();
        timer_ = this->create_wall_timer(
            chrono::milliseconds(50),
            bind(&Strategist::publish_poses, this));

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        //Create the publishers for the actions each robot should do
        for(int i = 0; i < robot_count; i++){
            pubs_actions[i+1] = (this->create_publisher<vsss_simulation::msg::RobotAction>(
             "robot" + to_string(i + 1) + "/action", 10));
        }
        /*
        Type: 
        1 -> GoToAttack <This mode is for the robot to achiheve a goal and  to continue to follow the traced path>
        2 -> GoToDefend <This mode is for the robot to achieve a goal and just to stay in that place with and that position>
        3 -> Spin <This mode is for the robot to get eject the ball as fast as posible from its side of the field>
        */
        //Create the publisher for the spin action direction

        RCLCPP_INFO(this->get_logger(), "Strategist Started with %i robots", robot_count);

    
    }

private:
    void publish_poses()
    {

        //not rotative rolls;
        //Set robots init position
        for(int i  = 1; i <= robot_count; i++){
          string rName = "robot";
          rName += to_string(i);
          rName += "_base_link";

          try{
            auto temp = tf_buffer_->lookupTransform( "world",rName.c_str(), TimePointZero );
            Transform real;
            tf2::fromMsg(temp.transform, real);
            robots[i] = real;
          }catch(const TransformException & ex){
            //RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s","world", rName.c_str(), ex.what());
          }
        }
//-------------------------------------------------------------------------------------------\\
        //Attacker
            // Get ball and goal transform
            geometry_msgs::msg::TransformStamped ball_tf, goal_tf;
            try {
                ball_tf = tf_buffer_->lookupTransform("world", "sphere_link", TimePointZero);
                goal_tf = tf_buffer_->lookupTransform("world", "goal_pos", TimePointZero);
               
            } catch (const TransformException &ex) {
                RCLCPP_INFO(this->get_logger(), "No transform sphere_link or  goal_pos to world: %s", ex.what());
                return;
            }

        ball.setTrans(ball_tf);
        fromMsg(goal_tf.transform, goal);
        Line trayectory(ball.transform.getOrigin(), goal.getOrigin());
        int attacker_ID = 1;
        vsss_simulation::msg::RobotAction attacker_msg;
        attacker_msg.type.data = 1;
        attacker_msg.objective.set__x(ball.transform.getOrigin().x());
        attacker_msg.objective.set__y(ball.transform.getOrigin().y());
        attacker_msg.objective.set__theta(trayectory.getTheta());

        pubs_actions[attacker_ID]->publish(attacker_msg);
        if(robot_count < 2) return;
//----------------------------------------------------------------------------------------------------\\

        //Defender
        int defender_ID = 2;
        //Get limits for defense
        try {
            auto ole = tf_buffer_->lookupTransform("world", "own_lower_end_goal", TimePointZero);
            fromMsg(ole.transform, own_lower_end);
            auto oue = tf_buffer_->lookupTransform("world", "own_upper_end_goal", TimePointZero);
            fromMsg(oue.transform, own_upper_end);
        } catch (const TransformException & ex) {
            RCLCPP_INFO(this->get_logger(), "Could not transform own goal ends: %s", ex.what());
            return;
        }

        Line defensive_range(own_lower_end.getOrigin(), own_upper_end.getOrigin());

        
        //See if intersection in these lines  between the trayectory of the ball;
        Line ball_trayectory = Line(ball.transform.getOrigin(), ball.transform.getOrigin() + ball.velocity);
        pair<int, Vector3> intersect_result = defensive_range.Intersect(ball_trayectory);
        if(intersect_result.first == 1){
            defend_point = intersect_result.second;
        }
        
        //Go to Intersection or spin to get the ball out of the place
        vsss_simulation::msg::RobotAction defense_action;
        if((ball.transform.getOrigin() - robots[defender_ID].getOrigin()).length() < 0.3 ){
            defense_action.type.data = 3;
            defense_action.spin_direction.data = (ball.transform.getOrigin() - robots[defender_ID].getOrigin()).y() < 0;
        }else{
            defense_action.type.data = 2;
            defense_action.objective.set__x(defend_point.x());
            defense_action.objective.set__y(defend_point.y());
            defense_action.objective.set__theta(M_PI / 2);
        }

        pubs_actions[defender_ID]->publish(defense_action);
        
        

        


    }
    int robot_count;
    //Publishers for information in each robot
    unordered_map<int, rclcpp::Publisher<vsss_simulation::msg::RobotAction>::SharedPtr> pubs_actions;
    //Saved information for each robot
    unordered_map<int, Transform> robots_transform;
    //Transform listener
    shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    unique_ptr<tf2_ros::Buffer> tf_buffer_;
    //Main timer
    rclcpp::TimerBase::SharedPtr timer_;
    //Robots_Pos 
    unordered_map<int, Transform> robots;

    //Atacker
    Kinematic ball;
    Transform goal;
    
    //Defender
    Transform own_lower_end;
    Transform own_upper_end;
    Vector3 defend_point;

};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(make_shared<Strategist>());
    rclcpp::shutdown();
    return 0;
}