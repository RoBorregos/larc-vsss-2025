#ifndef BALL_H
#define BALL_H

#include "Entity.h"
//Ball Entity
// Represents the ball entity in the system.
// Inherits from the Entity class and adds a reference to the goal Transform.
class Ball: public Entity{
    public:
        Transform& goal; // Reference to the goal's Transform, representing the target position for the ball.
        Ball(Transform& t, Transform& goal,int i, float f, int port);
};

#endif