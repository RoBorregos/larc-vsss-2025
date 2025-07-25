#pragma once
#include <cmath>
#include <iostream>
#include "vsss_simulation/MathU.hpp"
#include <math.h>
#include <tf2/LinearMath/Vector3.h>
#include "vsss_simulation/Line.hpp"
#include "vsss_simulation/Kinematic.hpp"
using namespace tf2;
using namespace std;



float phiH(float rho,  float theta, bool cw);    //Hyperbolic angle


float phiTuf(float theta, Vector3 p ,Vector3 b,  Line& trajectory) ; // Move to Goal

Vector3 getImagePos(Kinematic main, Kinematic obst);    //Posible Position of enemy position;

float phiAuf(Kinematic main, Kinematic enemy );         //Angle Phi 

float phiCompose(float ball_c, float enemy_c, float distance_);


    /*
    Returns a coefficient of a hyperbolic spiral that guides the robot to the ball
    '''
    '''
    The direction of rotation of the spiral has been inverted, cause by passing as in the article, 
    the clockwise direction becomes counterclockwise and vice versa
    '''*/
