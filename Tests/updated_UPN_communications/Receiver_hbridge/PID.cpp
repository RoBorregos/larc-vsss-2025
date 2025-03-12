#include "Arduino.h"
#include "PID.h"

//Definicion de constantes

PID::PID(float kp_, float ki_, float kd_){
  kp = kp_;
  ki = ki_;
  kd = kd_;
}

//Magia del PID donde se usan las constantes para hacer la correccion;

float PID::GetCorrection(float error){
  acumulativo += error;
  if ((error*acumulativo)<0) acumulativo=0;  // corrige el overshooting - integral windup
  diferencial = error - previo;
  previo = error;
  return kp * error + ki * acumulativo + kd * diferencial;
}

void PID::Reset(){
  acumulativo = 0;
  diferencial = 0;
  previo = 0;
}