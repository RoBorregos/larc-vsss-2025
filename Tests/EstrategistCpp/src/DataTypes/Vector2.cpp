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

float Vector2::GetAngle() const{
    return atan2(y, x);
}
void Vector2::Normallize() {
    float mag = Magnitude();
    if (mag > 0) {
        x /= mag;
        y /= mag;
    }
}

Vector2 Vector2::operator-( const Vector2& b) const{
    return Vector2(this->x -b.x, this->y - b.y);
}
Vector2 Vector2::operator+(const Vector2& b) const{
    return Vector2(this->x + b.x, this->y + b.y);
}

Vector2& Vector2::operator+= (const Vector2& b){
    this->x += b.x;
    this->y += b.y;
    return *this;
}

Vector2& Vector2::operator*=(const float& value){
    this->x *= value;
    this->y *= value;
    return *this;
}

Vector2 Vector2::operator*(const float& value) const{
    return Vector2(this->x * value, this->y * value);
}

ostream& operator<<(ostream& os, const Vector2& v) {
    os << "( " << v.x << ", " << v.y<< " )";
    return os;
}


