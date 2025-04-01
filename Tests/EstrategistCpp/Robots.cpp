

#include "Robots.h"
#include <math.h>
//position
robot:: robot(){
    x = 0;
    y = 0;
    z = 0;
    impact = 0;

}
robot::robot(float x_,float y_, float z_, float imp){
    x = x_;
    y = y_;
    z = z_;
    impact = imp;
}
force robot::GetForce(robot a){
    force temp;
    return temp;
}

bool robot::operator==( const robot & other){
    return x == other.x && y == other.y && z == other.z;
}


//force
force::force(float m, float t){
    x = m;
    y = t;
}
force::force(){
    x = 0;
    y = 0;   
}
force& force::operator += (const force& other){
    x += other.x;
    y += other.y;
    return *this;
}

force force::operator * (const float& value){
    force temp;
    temp.x = this->x * value;
    temp.y = this->y * value;
    return temp;
}
force force::operator +(const force& other){
    force temp;
    temp.x = this->x + other.x;
    temp.y = this->y + other.y;
    return temp;

}


//allies

allie::allie() : robot(){

    rol = -1;
}
allie::allie(float x_, float y_, float z_, float imp, int rol_ ) : robot(x_,y_,z_,imp){
    rol  =rol_;
}

// allie
force allie::GetForce (robot main ) {
    
    float temp_x = -(main.y - y)/(pow(main.x - x , 2) + pow(main.y - y , 2));
    float temp_y = (main.x - x)/(pow(main.x - x , 2) + pow(main.y - y , 2));
    temp_x *= impact;
    temp_y *= impact;
    return force(temp_x, temp_y);

}

