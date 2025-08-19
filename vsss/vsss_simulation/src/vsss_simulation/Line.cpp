#include "vsss_simulation/Line.hpp"
using namespace tf2;


Line::Line(){}
Line::Line(Vector3 pos, Vector3 obj){
    Refresh(pos, obj);
}

Line::Line(Vector3 pos, float theta){
    this->theta = theta;
    m = tan(theta);
    b = pos.y() - m*pos.x();
}

void Line::Refresh(Vector3 pos, Vector3 obj){
    m = (obj.y() - pos.y())/(obj.x() - pos.x());
    b = pos.y()- m*pos.x();
    theta = atan2(obj.y() - pos.y() , obj.x() - pos.x());
    Init = pos; 
    End = obj;
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

pair<int, Vector3> Line::Intersect(Line other){
    Vector3 r = End - Init; Vector3 s = other.End- other.Init;
    float down =  (r.cross(s)[2]);
    float over = (other.Init - Init).cross(s)[2] ;
    if(down  == 0 && over == 0){
        //Collinear
        return pair<int, Vector3>(2, Vector3());
    }else if(down == 0){
        //Paralel
        return pair<int, Vector3>(0, Vector3());
    }
    float t  =over / down;
    if(0 <= t && t <= 1){
        return pair<int, Vector3>(1, Init + r * t);
    }


}
