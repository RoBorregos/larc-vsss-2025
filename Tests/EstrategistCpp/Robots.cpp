

#include "Robots.h"
#include <math.h>
//position
position:: position(){
    x = 0;
    y = 0;
    z = 0;
    impact = 0;

}
position::position(float x_,float y_, float z_, float imp){
    x = x_;
    y = y_;
    z = z_;
    impact = imp;
}
force position::GetForce(position a){
    force temp;
    return temp;
}

bool position::operator==( const position & other){
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

//allies

allie::allie() : position(){

    rol = -1;
}
allie::allie(float x_, float y_, float z_, float imp, int rol_ ) : position(x_,y_,z_,imp){
    rol  =rol_;
}

// allie
force allie::GetForce (position main ) {
    
    float temp_x = -(main.y - y)/(pow(main.x - x , 2) + pow(main.y - y , 2));
    float temp_y = (main.x - x)/(pow(main.x - x , 2) + pow(main.y - y , 2));
    temp_x * impact;
    temp_y * impact;
    return force(temp_x, temp_y);

}

