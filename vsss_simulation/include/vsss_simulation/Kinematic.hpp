#pragma once
#include <iostream>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Transform.h>
#include "geometry_msgs/msg/twist.hpp"
#include "vsss_simulation/MathU.hpp"
using namespace std;

class Kinematic{
    private:
        tf2::Transform transform;
    public:
        Kinematic();
        void setTrans(tf2::Transform);
        geometry_msgs::msg::Twist result_to_msg(tf2::Vector3);
};
