#ifndef TRANSFORM_H
#define TRANSFORM_H
#include "Vector2.h"
class Transform
{
    public :
        Vector2 position;
        mutable float rotation;
        Transform(float x, float y, float r) ;
        Transform(Vector2 p, float r) ;
        Transform() ;
        Transform operator-(const Transform& b) const;
        void SetAngule() const;
        void CheckAngle() const;
        float GetRotationalDifference(float objective) const;


};

#endif