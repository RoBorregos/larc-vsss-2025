#include "vsss_simulation/MsgConvert.hpp"


void VectorFromMSG( geometry_msgs::msg::Vector3& r, Vector3& o){
    o.setValue(r.x, r.y, r.z);
    return;
}

void QuaternionFromMSG( geometry_msgs::msg::Quaternion& r, Quaternion& o){
    o = Quaternion(r.x,r.y,r.z,r.w);    
}

void TransformFromMSG( geometry_msgs::msg::Transform& r, Transform& o){
    Vector3 t;
    Quaternion q;
    VectorFromMSG(r.translation, t);
    QuaternionFromMSG(r.rotation, q);
    o.setOrigin(t);
    o.setRotation(q);
    return;
}

void SetTransformFromOdom(const nav_msgs::msg::Odometry::SharedPtr& msg, Transform& o){
      Vector3 pos(
        msg->pose.pose.position.x,
        msg->pose.pose.position.y,
        msg->pose.pose.position.z
      );
      Quaternion rotation(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w
      );
      o.setOrigin(pos);
      o.setRotation(rotation);
      return;
}


void MSGFromVector3(Vector3& r,  geometry_msgs::msg::Vector3&o){
    o.x = r[0];
    o.y = r[1];        
    o.z = r[2];
    
}