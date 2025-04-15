#ifndef BALL_H
#define BALL_H

#include "Entity.h"
//entidad de pelota
class Ball: public Entity{
    public:
        Transform& goal;
        Ball(Transform& t, Transform& goal,int i, float f);
};

#endif