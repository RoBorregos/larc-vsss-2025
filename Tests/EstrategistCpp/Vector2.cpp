#include "Vector2.h"
#include <cmath>
using namespace std;

Vector2::Vector2(){
    x = 0;
    y = 0;
}
Vector2::Vector2(float x_, float y_){
    x = x_;
    y = y_;   
}

float Vector2::Magnitude(){
    return sqrt(x*x + y*y);
}

Vector2 Vector2::operator-( const Vector2& b) const{
    return Vector2(this->x -b.x, this->y - b.y);
}



Vector2::Vector2(){
    x = 0;
    y = 0;
}

