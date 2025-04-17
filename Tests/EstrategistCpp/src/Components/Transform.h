#ifndef TRANSFORM_H
#define TRANSFORM_H
#include "Vector2.h"
#include <iostream>
//class defined to store the position and rotational data of entities. 
//In the future this class will store more than just the pose info
//each of the functions are explained inside the Transforn.cpp file
class Transform
{
    public :
        Vector2 position;
        mutable float rotation;
        Transform(float x, float y, float r) ; // x,y,rotation
        Transform(Vector2 p, float r) ;
        Transform() ;
        Transform operator-(const Transform& b) const;
        friend std::ostream& operator<<(std::ostream& os, const Transform& t) ;
        void SetAngule() const;
        void CheckAngle() const;
        float GetRotationalDifference(float objective) const;
        void SetTransform(float, float, float);
};

#endif