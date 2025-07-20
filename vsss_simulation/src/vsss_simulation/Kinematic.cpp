#include "vsss_simulation/Kinematic.hpp"
using namespace tf2;

Kinematic::Kinematic(){}
void Kinematic::setTrans(Transform n){
    transform = n;
}
geometry_msgs::msg::Twist Kinematic::result_to_msg(Vector3 objective){

    Vector3 front = quatRotate(transform.getRotation(), Vector3(1,0,0));
    front[2] = 0;
    objective[2] = 0;
    float dif  = atan2(front.x(), front.y()) - atan2(objective.x(), objective.y());
    dif = wrapToPI(dif);
    geometry_msgs::msg::Twist response;
    response.angular.z = dif*2;
    response.linear.x = 0.3;
    return response;
}

