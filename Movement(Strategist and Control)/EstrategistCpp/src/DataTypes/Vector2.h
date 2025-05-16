#ifndef VECTOR2_H
#define VECTOR2_H

#include <iostream>
//Data Type used as a way to store multiple varibles, such as position, force etc...
class Vector2 
{
    public:
    float x;
    float y;
    Vector2(float _x, float _y); //x, y
    Vector2();
    float Magnitude();
    float GetAngle() const;
    void Normallize() ;
    Vector2 operator-(const Vector2& b) const;
    Vector2 operator+(const Vector2& b) const;
    Vector2& operator+= (const Vector2& b);
    Vector2& operator*=(const float& value) ;
    Vector2 operator *(const float& value) const;
    friend std::ostream& operator<<(std::ostream& os, const Vector2& v);

};


#endif