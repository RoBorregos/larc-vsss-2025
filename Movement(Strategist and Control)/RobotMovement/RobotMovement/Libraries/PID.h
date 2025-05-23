
#ifndef PID_H
#define PID_H

#include "Arduino.h"

class PID{
  public: 
  float kp;
  float ki;
  float kd;
  float previo;
  float acumulativo;
  float diferencial;
  PID(float kp, float ki, float kd);
  float GetCorrection(float actuError);
};
#endif