#include "vsss_simulation/Kinematic.hpp"
using namespace tf2;

Kinematic::Kinematic(){}
void Kinematic::setTrans(geometry_msgs::msg::TransformStamped t){
    if(firstUpdate){
        TransformFromMSG(t.transform, transform);
        prevTime = rclcpp::Time(t.header.stamp);
        firstUpdate = false;
        return;
    }
    t.transform.translation.z = 0;
    prevTransform = transform;
    TransformFromMSG(t.transform, transform);
    rclcpp::Time newTime = rclcpp::Time(t.header.stamp);
    velocity = (transform.getOrigin() - prevTransform.getOrigin())/(newTime-prevTime).seconds();
    prevTime = newTime;
}




geometry_msgs::msg::Twist Kinematic::result_to_msg(Vector3 objective){

    Vector3 front = quatRotate(transform.getRotation(), Vector3(-1,0,0));
    front[2] = 0;
    objective[2] = 0;
    float dif  = atan2(front.x(), front.y()) - atan2(objective.x(), objective.y());
    dif = wrapToPI(dif);
    geometry_msgs::msg::Twist response;
    response.angular.z = dif*2;
    response.linear.x = 0.8;
    return response;
}

