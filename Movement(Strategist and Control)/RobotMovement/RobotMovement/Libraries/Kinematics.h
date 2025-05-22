
#ifndef KINEMATICS_H
#define KINEMATICS_H

#include "Arduino.h"
#include "Output.cpp"
#include "Velocities.cpp"


class Kinematics
{
  public:
    
    Kinematics(float motor_max_rpm, float wheel_diameter, float lr_wheels_dist, int pwm_bits, float ConstVelDiff, float  ConstThetaDiff);
    velocities getVelocities(output actualVel, float theta);
    output getRPM(velocities);
    output getPWM(velocities);

    float max_vel;

  private:
    float radius;
    float circumference_;
    float max_rpm_;
    float lr_wheels_dist_;
    float pwm_res_;
    float ConstThetaDiff;
    float ConstVelDiff;
};




#endif