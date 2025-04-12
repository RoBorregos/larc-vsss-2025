#ifndef VECTOR2_H
#define VECTOR2_H

class Vector2 
{
    public:
    float x;
    float y;
    Vector2(float, float);
    Vector2();
    float Magnitude();
    Vector2 operator-(const Vector2& b) const;

};


#endif