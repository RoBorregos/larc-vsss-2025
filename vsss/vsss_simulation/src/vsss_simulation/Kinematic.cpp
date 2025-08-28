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



float dif_vector(Vector3 obj, Transform robot){
    Vector3 front = quatRotate(robot.getRotation(), Vector3(-1,0,0));
    front[2] = 0;
    obj[2] = 0;
    float dif  = atan2(front.x(), front.y()) - atan2(obj.x(), obj.y());
    dif = wrapToPI(dif);
    return dif;
}

geometry_msgs::msg::Twist Kinematic::result_to_msg(Vector3 objective, int type){

    float dif = dif_vector(objective, transform);
    geometry_msgs::msg::Twist response;

    if(type == 2 && abs(dif) > M_PI/2){
        dif += M_PI;
        dif = wrapToPI(dif);
        response.angular.z = dif*0.75;
        response.linear.x = -0.4;

    }else{
        response.angular.z = dif*0.5;
        response.linear.x = 0.4;
    }
    response.angular.z = type ==2 ? response.angular.z*2: response.angular.z;

    return response;
}

geometry_msgs::msg::Twist Kinematic::orient_to_msg(Vector3 objective){
    float dif = dif_vector(objective, transform);
    geometry_msgs::msg::Twist response;
     if(abs(dif) > M_PI/2){
        dif += M_PI;
        dif = wrapToPI(dif);
     }
    response.angular.z = dif*0.75;
    return response;
}

