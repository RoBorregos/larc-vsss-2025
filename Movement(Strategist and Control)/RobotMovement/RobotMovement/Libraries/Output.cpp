#include "Arduino.h"

class output
{
  public:
  float motor1; //left
  float motor2; //right
  float pwm_res_ = 255;
  float max_rpm_ = 210;
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

  output rpmToPWM(output rpm)
  {
    //remap scale of target RPM vs MAX_RPM to PWM
    rpm.motor1 = ((rpm.motor1 /  max_rpm_) * pwm_res_);
    rpm.motor2 = ((rpm.motor2 /  max_rpm_) * pwm_res_);
    return rpm;
  }

  void Print(){
    Serial.print(motor1); Serial.print(" ");Serial.print(motor2);
  }
};