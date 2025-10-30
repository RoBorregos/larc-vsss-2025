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
#include "vsss_simulation/Polygon.hpp"
#include <vector>
#include <string>
using namespace std;
using namespace tf2;


//Values of map
//Field Sizes
const float field_width = 1.5f;
const float field_height = 1.3f;
const float goal_height = 0.45f;
const float defender_height = 0.8f;
const float defender_width = 0.15;
Vector3 vertical_dif =  Vector3(0, defender_height/2 ,0); //Diference from the start of the center of the goal towards its vertical limits
Vector3 horizontal_dif = Vector3(defender_width/2, 0, 0); //Diference from the start of the center of the goal towards its horizontal limits


float kick_distance = 0.08;



class Strategist : public rclcpp::Node
{
public:
    Strategist(): Node("Strategist")
    {
        
        this->declare_parameter<int>("Robot_count",  0);
        this->declare_parameter<bool>("Robot_side", false);

        robot_count = this->get_parameter("Robot_count").as_int();
        field_side = this->get_parameter("Robot_side").as_bool();

        objective_name = field_side ? "goal_pos" : "own_goal";
        defender_name = field_side ? "own_goal": "goal_pos";

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
        //Triangle of unaccessible area a.k.a area where is better to kick than to push the ball
          //Set the area depending on the side of the field
        int triangle_flip = field_side ? 1 : -1;
        vector<Vector3> triangle_area = {Vector3(-field_width/2, -field_height/2, 0) , Vector3(field_width/2, -field_height/2, 0) , Vector3(field_width/2 * triangle_flip, -goal_height/2, 0)};
        kick_area_down = Polygon(triangle_area);
        for(int i = 0; i < triangle_area.size(); i++){
            triangle_area[i].setY(-triangle_area[i].getY());  
        }
        kick_area_up = Polygon(triangle_area);



    
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

        // Get ball and goal transform
        geometry_msgs::msg::TransformStamped ball_tf, goal_tf;
        try {
            ball_tf = tf_buffer_->lookupTransform("world", "sphere_link", TimePointZero);
            goal_tf = tf_buffer_->lookupTransform("world", objective_name, TimePointZero);
            
        } catch (const TransformException &ex) {
            RCLCPP_INFO(this->get_logger(), "No transform sphere_link or  goal_pos to world: %s", ex.what());
            return;
        }
        ball.setTrans(ball_tf);

//-------------------------------------------------------------------------------------------\\

        
        //Attacker
   
        fromMsg(goal_tf.transform.translation, attacker_goal);
        Line trayectory(ball.transform.getOrigin(), attacker_goal  );
        
        vsss_simulation::msg::RobotAction attacker_msg;
        if((ball.transform.getOrigin().x() < 0) != field_side ||( robots[support_ID].getOrigin() - ball.transform.getOrigin()).length() > 0.35){
            attacker_msg.type.data = 1;
            attacker_msg.objective.set__x(ball.transform.getOrigin().x());
            attacker_msg.objective.set__y(ball.transform.getOrigin().y());
            attacker_msg.objective.set__theta(trayectory.getTheta());
            
            //Check if ball imposible to move, so try to kick
            if(kick_area_down.isInside(ball.transform.getOrigin())|| kick_area_up.isInside(ball.transform.getOrigin())){
                attacker_msg.type.data = 2;
                //If near enough just rotate
                if((ball.transform.getOrigin() - robots[attacker_ID].getOrigin()).length() < kick_distance){
                    attacker_msg.type.data = 3;
                    attacker_msg .spin_direction.data = ((ball.transform.getOrigin() - robots[attacker_ID].getOrigin()).y() > 0) != field_side;
                }
            }
        }else{
            attacker_msg.type.data = 2;
            float attacker_range_start = field_side ? 0.0 : attacker_goal.getX();
            float attacker_range_end = !field_side ? 0.0 : attacker_goal.getX(); 
            float attacker_x_coord = max(min(attacker_range_end, (float)robots[attacker_ID].getOrigin().x()), attacker_range_start);
            attacker_msg.objective.set__x(attacker_x_coord);
            attacker_msg.objective.set__y(-ball.transform.getOrigin().y());
            attacker_msg.objective.set__theta(M_PI / 2);
        }
        



        pubs_actions[attacker_ID]->publish(attacker_msg);
        if(robot_count < 2) return;
//----------------------------------------------------------------------------------------------------\\

        //Get point for defense
        //$ Could be in the init function to avoid doing this every time;
        try {
            auto ole = tf_buffer_->lookupTransform("world", defender_name, TimePointZero);
            fromMsg(ole.transform, own_goal);
        } catch (const TransformException & ex) {
            RCLCPP_INFO(this->get_logger(), "Could not transform own goal ends: %s", ex.what());
            return;
        }
        own_goal.getOrigin().setX(own_goal.getOrigin().getX() + (field_side ? 1 : -1) * (defender_width/2 + 0.04)); //Little offset because the goal is the line between the field and the goal
        Vector3 upper_end = own_goal.getOrigin() + vertical_dif;
        Vector3 lower_end = own_goal.getOrigin() - vertical_dif;

        Vector3 ul = own_goal.getOrigin() - horizontal_dif + vertical_dif;
        Vector3 ur = own_goal.getOrigin() + horizontal_dif + vertical_dif;
        Vector3 lr = own_goal.getOrigin() + horizontal_dif - vertical_dif;
        Vector3 ll = own_goal.getOrigin() - horizontal_dif - vertical_dif;
        vector<Vector3> p {ul, ur, lr, ll};
        //Create the defender zone for the defender
        Polygon defenderZone (p);
        //if the robot is inside its own defensive zone, the prediction should be in the own line of robot, or its own defensive range (what ever is near to the goal)
        //set the x coord of the line
        float defender_x_coordinate = max(min(ur.x(), robots[defender_ID].getOrigin().x()), ul.x()); // max(min(r,v), l) -> l < v < r
        upper_end.setX(defender_x_coordinate);
        lower_end.setX(defender_x_coordinate);
       
    
        Line defensive_range(lower_end , upper_end);

        //$$$

        
        //See if intersection in these lines  between the trayectory of the ball;
        Line ball_trayectory = Line(ball.transform.getOrigin(), ball.transform.getOrigin() + ball.velocity);
        pair<int, Vector3> intersect_result = defensive_range.Intersect(ball_trayectory);
        intersect_result.first &= ball.velocity.length() > 0.1;
        bool ball_inside_detected = defenderZone.isInside(ball.transform.getOrigin());
        if(ball_inside_detected){
            defender_point = ball.transform.getOrigin();
        }else if(intersect_result.first == 1){
            defender_point = intersect_result.second ;
        }
        
        //Go to Intersection or spin to get the ball out of the place
        vsss_simulation::msg::RobotAction defense_action;
        if((ball.transform.getOrigin() - robots[defender_ID].getOrigin()).length() < kick_distance ){
            defense_action.type.data = 3;
            defense_action.spin_direction.data = ((ball.transform.getOrigin() - robots[defender_ID].getOrigin()).y() > 0) != field_side;
        }else{
            defense_action.type.data = 2;
            defense_action.objective.set__x(defender_point.x());
            defense_action.objective.set__y(defender_point.y());
            defense_action.objective.set__theta(M_PI / 2);
            if(!ball_inside_detected){
                defense_action.objective.set__x(defender_x_coordinate);
            }
        }

        pubs_actions[defender_ID]->publish(defense_action);
        if(robot_count < 3) return;
        

//----------------------------------------------------------------------------------------------------\\
    //support

        
        trayectory = Line(ball.transform.getOrigin(), robots[attacker_ID].getOrigin()  );
        
        vsss_simulation::msg::RobotAction support_msg;
        if((ball.transform.getOrigin().x() > 0 )!= field_side){
            support_msg.type.data = 1;
            support_msg.objective.set__x(ball.transform.getOrigin().x());
            support_msg.objective.set__y(ball.transform.getOrigin().y());
            support_msg.objective.set__theta(trayectory.getTheta());
            
            //Check if ball imposible to move, so try to kick
            if(abs(ball.transform.getOrigin().getY()) > (defender_height) /2 ){
                support_msg.type.data = 2;
                //If near enough just rotate
                if((ball.transform.getOrigin() - robots[support_ID].getOrigin()).length() < kick_distance){
                    support_msg.type.data = 3;
                    support_msg .spin_direction.data = ((ball.transform.getOrigin() - robots[support_ID].getOrigin()).y() > 0) != field_side;
                }
            }
        }else{
            support_msg.type.data = 2;
            float support_range_start = !field_side ? 0.0 : defender_point.getX();
            float support_range_end = field_side ? 0.0 : defender_point.getX(); 
            float support_x_coord = max(min(support_range_end, (float)robots[support_ID].getOrigin().x()), support_range_start);
            support_msg.objective.set__x(support_x_coord);
            support_msg.objective.set__y(ball.transform.getOrigin().y());
            support_msg.objective.set__theta(M_PI / 2);
        }
        
        pubs_actions[support_ID]->publish(support_msg);

        


    }
    //Parameters
    bool field_side;
    int robot_count;

    //Publishers for information in each robot
    unordered_map<int, rclcpp::Publisher<vsss_simulation::msg::RobotAction>::SharedPtr> pubs_actions;
    //Transform listener
    shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    unique_ptr<tf2_ros::Buffer> tf_buffer_;
    //Main timer
    rclcpp::TimerBase::SharedPtr timer_;
    //Robots_Pos 
    unordered_map<int, Transform> robots;

    //Atacker
    Kinematic ball;
    Vector3 attacker_goal;
    Polygon kick_area_up, kick_area_down;
    
    //Defender
    Transform own_goal;
    Vector3 defender_point;

    //Global Roles varaibles
    string objective_name = "";
    string defender_name = "";


    //IDS
        //Attacker
        int attacker_ID = 1;
        //Defender
        int defender_ID = 2;
        //Support
        int support_ID = 3;

};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(make_shared<Strategist>());
    rclcpp::shutdown();
    return 0;
}