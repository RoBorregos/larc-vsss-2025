#include "vsss_simulation/Kinematic.hpp"
using namespace tf2;

Kinematic::Kinematic(){}
void Kinematic::setTrans(geometry_msgs::msg::TransformStamped t){
    if(firstUpdate){
        TransformFromMSG(t.transform, transform);
        prevTime = rclcpp::Time(t.header.stamp);
        firstUpdate = false;
        velocity = Vector3(0,0,0);
        return;
    }
    t.transform.translation.z = 0;
    prevTransform = transform;
    TransformFromMSG(t.transform, transform);
    rclcpp::Time newTime = rclcpp::Time(t.header.stamp);
    
    float delta_time = (newTime-prevTime).seconds();
    if(delta_time < 1e-6)
        return;
    Vector3 delta_position = (transform.getOrigin() - prevTransform.getOrigin());
    velocity += delta_position/delta_time;
    velocity /= 2;
    prevTime = newTime;
    if(velocity.length() < 1e-6)
        velocity = Vector3(0,0,0);
}



float dif_vector(Vector3 obj, Transform robot){
    Vector3 front = quatRotate(robot.getRotation(), Vector3(1,0,0));
    front[2] = 0;
    obj[2] = 0;
    float dif  = atan2(front.x(), front.y()) - atan2(obj.x(), obj.y());
    dif = wrapToPI(dif);
    return dif;
}

geometry_msgs::msg::Twist Kinematic::result_to_msg(Vector3 objective, int type){

    float dif = dif_vector(objective, transform);
    geometry_msgs::msg::Twist response;
    bool invert = false;
    if(type == 2 && abs(dif) > M_PI/2){
        dif += M_PI;
        dif = wrapToPI(dif);
        invert = true;
    }

    response.angular.z = dif*ANGULAR_PROPORTIONAL_CONSTANT + ((dif+prev_dif_angle)/2) * ANGULAR_INTEGRAL_CONSTANT;
    response.linear.x = -LINEAR_CONSTANT;
    response.linear.x *= invert ? -1 : 1;
    response.angular.z = type ==2 ? response.angular.z*4: response.angular.z;
    prev_dif_angle = dif;

    return response;
}

geometry_msgs::msg::Twist Kinematic::orient_to_msg(Vector3 objective){
    float dif = dif_vector(objective, transform);
    geometry_msgs::msg::Twist response;
     if(abs(dif) > M_PI/2){
        dif += M_PI;
        dif = wrapToPI(dif);
     }
    response.angular.z = dif * ANGULAR_PROPORTIONAL_CONSTANT;
    return response;
}

