#ifndef BALL_
#define BALL_
//header file content
#include "Robots.h"



class Ball:  public robot{
public:
    float dist;
    Ball();
    Ball (float, float, float, float, robot, float);
    robot otherSide;
    robot goal;
    force GetForce(robot) override;
    
};
#endif