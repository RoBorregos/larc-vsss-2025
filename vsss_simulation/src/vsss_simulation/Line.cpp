#include "vsss_simulation/Line.hpp"
using namespace tf2;


Line::Line(){}
Line::Line(Vector3 pos, Vector3 obj){
    Refresh(pos, obj);
}
void Line::Refresh(Vector3 pos, Vector3 obj){
    m = (obj.y() - pos.y())/(obj.x() - pos.x());
    b = pos.y()- m*pos.x();
    theta = atan2(obj.y() - pos.y() , obj.x() - pos.x());
}

float Line::Evaluate(float x){
    return m*x + b;
}

float Line::getTheta(){
    return theta;
}

float Line::Dist(Vector3 pos){
    float over = abs(b+m*pos.x()-pos.y());
    float down = sqrt(1+pow(m,2));
    return over/down;
}
