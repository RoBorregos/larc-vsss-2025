#ifndef KINEMATIC_H
#define KINEMATIC_H
#include "Transform.h"
#include "Output.h"

class Kinematic
{
    public:
        Transform &transform;
        float ANGULAR_CONSTANT = 0.66f;
        float LINEAR_CONSTANT = 0.33f;
        float RADIUS = 0.03f;
        float WHEEL_DISTANCE = 0.076f;
        float CIRCUMFERENCE;
        Kinematic();
        Kinematic(Transform &t);
        Output GetVelocities(Transform target);
        //Optionaly I can add odometry function to not just send an static rpm


        
};

#endif