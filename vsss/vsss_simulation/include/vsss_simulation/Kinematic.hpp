#pragma once
#include <iostream>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "vsss_simulation/MathU.hpp"
#include "vsss_simulation/MsgConvert.hpp"
using namespace std;
using namespace tf2;

class Kinematic{
    private:
        Transform prevTransform;
        rclcpp::Time prevTime;
        bool firstUpdate = true;
        float prev_dif_angle;
        
    public:
        Kinematic();
        void setTrans(geometry_msgs::msg::TransformStamped t);
        geometry_msgs::msg::Twist result_to_msg(Vector3, int);
        geometry_msgs::msg::Twist orient_to_msg(Vector3);
        Vector3 velocity;
        Transform transform;
        float ANGULAR_PROPORTIONAL_CONSTANT = 2.3f;
        float ANGULAR_INTEGRAL_CONSTANT = 0;
        float LINEAR_CONSTANT = 0.3f;
};
