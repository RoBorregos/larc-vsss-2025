
#ifndef KINEMATICS_H
#define KINEMATICS_H

#include "Arduino.h"


class output
{
  public:
  float motor1; //left
  float motor2; //right
  //Scale function to Chanage the maximum value and continue to have the proportion of the force
  void Scale(float MaxRPM){
    if(abs(motor1) > MaxRPM || abs(motor2) > MaxRPM){
      if(abs(motor1) > abs(motor2)){
        float scale = MaxRPM/abs(motor1);
        motor1 = MaxRPM * (signbit(motor1) ? -1 : 1);
        motor2 = motor2 * scale;
      }else{
        float scale = MaxRPM/abs(motor2);
        motor2 = MaxRPM * (signbit(motor2) ? -1 : 1);
        motor1 = motor1 * scale;
      }
    }
  }
};
class velocities
{
  public:
  float _x;
  float _y;
  float _z;
  float Magnitude()
  {
    return sqrt(pow (_x, 2) + pow(_y, 2));
  }
  //Get the angle of a vector if it is in a x y form
  void setAngule(){
    if(_x  == 0) {
      _z = _y > 0 ? PI/2 : 3*PI/2;
    }else if(_y == 0){
      _z = _x > 0 ? 0 : PI;
    }else{
        _z = atan(_y/_x);                
        _z = _x > 0? (_z < 0? 2*PI + _z : _z) : PI + _z;   
    }

  }
  //Get the least difference between angles
  float getThetaDif(float desire, float actual){
      float dif = desire - actual;
      while(abs(dif) > 2*PI){
        dif /= 2*PI;
      }
      if(abs(dif) < PI){
        return dif;
      }else{
        return (signbit(dif) ? 1 : -1) * 2*PI + dif;
      }
  }
  

};


class Kinematics
{
  public:
    
    Kinematics(float motor_max_rpm, float wheel_diameter, float lr_wheels_dist, int pwm_bits, float ConstVelDiff, float  ConstThetaDiff);
    velocities getVelocities(output actualVel, float theta);
    output getRPM(velocities);
    output getPWM(velocities);
    output rpmToPWM(output);
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