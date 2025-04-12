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

  void Print(){
    Serial.print(motor1); Serial.print(" ");Serial.print(motor2);
  }
};