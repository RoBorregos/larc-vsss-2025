/*
  Copyright (c) 2016, Juan Jimeno
  Source: http://research.ijcaonline.org/volume113/number3/pxc3901586.pdf
  All rights reserved.
  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:
   Redistributions of source code must retain the above copyright notice,
  this list of conditions and the following disclaimer.
   Redistributions in binary form must reproduce the above copyright
  notice, this list of conditions and the following disclaimer in the
  documentation and/or other materials provided with the distribution.
   Neither the name of  nor the names of its contributors may be used to
  endorse or promote products derived from this software without specific
  prior written permission.
  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORTPPIPI (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE.
*/

#include "Arduino.h"
#include "Kinematics.h"

Kinematics::Kinematics(float motor_max_rpm, float wheel_diameter, float lr_wheels_dist, int pwm_bits){
  circumference_ = PI * wheel_diameter;
  max_rpm_ = motor_max_rpm;
  lr_wheels_dist_ = lr_wheels_dist;
  pwm_res_  = pow ( 2, pwm_bits) - 1;
  max_vel = (float)motor_max_rpm * circumference_;
  radius = wheel_diameter/2;
}
  
  

output Kinematics::getRPM(velocities ObVel) // the objective velocityes x,y,theta
{
//For now it is that x is the force and z the angule. However this would change depending on the development we could do later (odometry)
  
  float ObVelMagnitude = ObVel._x;    //ObVel.Magnitude(); //ToGet the magnitude of the force
  //ObVel.setAngule();
  //ObVel._z = ObVel.getThetaDif(ObVel._z, PI/2);

  float leftVel = ObVelMagnitude/ radius - lr_wheels_dist_ / (2*radius) * ObVel._z; 
  float rightVel = ObVelMagnitude/ radius + lr_wheels_dist_ / (2*radius) * ObVel._z;

  //Change the data to rpm insted of m/s;
  output rpm;
  rpm.motor1 = leftVel/circumference_  * 60;
  rpm.motor2  = rightVel / circumference_ * 60;
  rpm.Scale(max_rpm_);
  return rpm;
}

output Kinematics::getPWM(velocities ObVel)
{
  output rpm;
  output pwm;

  rpm = getRPM(ObVel);

  //convert from RPM to PWM
  //front-left motor
  pwm = rpmToPWM(rpm);

  return pwm;
}

velocities Kinematics::getVelocities(output actualRPM, float theta)
{
  velocities vel;

  actualRPM.motor1 = (actualRPM.motor1 *circumference_ / 60);
  actualRPM.motor2 = (actualRPM.motor2 *circumference_ / 60);

  //With the formulas we determine the velocities and send them;

  vel._x = (radius * actualRPM.motor1)/2 *cos(theta) + (radius * actualRPM.motor2) /2 *cos(theta);
  vel._y = (radius * actualRPM.motor1)/2 *sin(theta) + (radius * actualRPM.motor2) /2 *sin(theta);
  vel._z = -radius / lr_wheels_dist_ * actualRPM.motor1 +  radius / lr_wheels_dist_ * actualRPM.motor2; 


  return vel;


}



output Kinematics::rpmToPWM(output rpm)
{
  //remap scale of target RPM vs MAX_RPM to PWM
  rpm.motor1 = ((rpm.motor1 /  max_rpm_) * pwm_res_);
  rpm.motor2 = ((rpm.motor2 /  max_rpm_) * pwm_res_);
  return rpm;
}