#include "Arduino.h"
#include "PID.h"

//Constant Deffinition

PID::PID(float kp_, float ki_, float kd_){
  kp = kp_;
  ki = ki_;
  kd = kd_;
}



float PID::GetCorrection(float error){
  acumulativo += error; // Integral
  if ((error*acumulativo)<0) acumulativo=0;  // corrige el overshooting 
  diferencial = error - previo; // diferential
  previo = error;             
  return kp * error + ki * acumulativo + kd * diferencial;
}

