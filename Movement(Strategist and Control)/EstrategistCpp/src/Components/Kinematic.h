#ifndef KINEMATIC_H
#define KINEMATIC_H
#include "Transform.h"
#include "Output.h"
//class created to generate necessary info for the robot.
// The output data is made specifically to use as rpm for the robot.
class Kinematic
{
    public:
        Transform &transform;
        //Constants for the movement in the wheels
        float ANGULAR_CONSTANT =0.08f;
        float LINEAR_CONSTANT = 0.04f;
        float RADIUS = 0.03f;
        float WHEEL_DISTANCE = 0.076f;
        float CIRCUMFERENCE;
        Kinematic(Transform &t);
        //Functions to obtain the desaired rpm for the robot depending on the input (Transform or vector2)
        Output GetVelocities(Transform target);
        Output GetVelocities(Vector2 target);
        Output GetVelocitiesForRotation(Transform target);

        Kinematic(const Kinematic& other) ;
        Kinematic& operator=(const Kinematic& other) ;
        //Optionaly I can add odometry function to not just send an static rpm


        
};

#endif