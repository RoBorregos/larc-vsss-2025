#include "Ball.h"

#include <math.h>

Ball::Ball():robot(){
    otherSide;
    goal;
}

Ball::Ball(float x_, float y_, float z_, float imp, robot gl , float dist_):robot(x_,y_,z_,imp){
    otherSide;
    goal = gl;
    dist= dist_;
}

force Ball::GetForce(robot attacker){
    float m = (goal.y - y)/(goal.x-x);
    float d = sqrt(pow(dist,2) / (1+pow(m,2)));
    cout<<dist<<endl;
    cout<<d<<endl;
    otherSide.x = x+d;
    otherSide.y = m*otherSide.x - m*x + y;
    cout<<otherSide.x<<" " <<otherSide.y<<endl;
    force B, O;
    B.x =( attacker.x - x)/(pow(attacker.x-x,2) + pow(attacker.y - y, 2));
    B.y =( attacker.y - y)/(pow(attacker.x-x,2) + pow(attacker.y - y, 2));
    O.x = ( attacker.x- otherSide.x)/(pow(attacker.x-otherSide.x,2) + pow(attacker.y - otherSide.y, 2));
    O.y = ( attacker.y- otherSide.y)/(pow(attacker.x-otherSide.x,2) + pow(attacker.y - otherSide.y, 2));
    force fin;
    cout<<B.x<<" " <<O.x<<endl;
    fin = (B*-impact) + (O*(impact));
    return fin;
}

