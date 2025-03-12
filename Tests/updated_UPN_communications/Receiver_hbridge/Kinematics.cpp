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

Kinematics::Kinematics(int motor_max_rpm, float wheel_diameter, float lr_wheels_dist, int pwm_bits):
  circumference_(PI * wheel_diameter),
  max_rpm_(motor_max_rpm),
  lr_wheels_dist_(lr_wheels_dist),
  pwm_res_ (pow(2, pwm_bits) - 1)
{
  radius = wheel_diameter/2;
}

Kinematics::output Kinematics::getRPM(velocities ObVel) // the objective velocityes x,y,theta
{
  /*//convert m/s to m/min
  linear_vel_x_mins_ = linear_x * 60;
  linear_vel_y_mins_ = linear_y * 60;

  //convert rad/s to rad/min
  angular_vel_z_mins_ = angular_z * 60;

  //Vt = ω * radius
  tangential_vel_ = angular_vel_z_mins_ * lr_wheels_dist_;

  x_rpm_ = linear_vel_x_mins_ / circumference_;
  y_rpm_ = linear_vel_y_mins_ / circumference_;
  tan_rpm_ = tangential_vel_ / circumference_;

  Kinematics::output rpm;

  //calculate for the target motor RPM and direction
  //front-left motor
  rpm.motor1 = x_rpm_ - y_rpm_ - tan_rpm_;
  //rear-left motor
  rpm.motor3 = x_rpm_ + y_rpm_ - tan_rpm_;

  //front-right motor
  rpm.motor2 = x_rpm_ + y_rpm_ + tan_rpm_;
  //rear-right motor
  rpm.motor4 = x_rpm_ - y_rpm_ + tan_rpm_;*/

  float ObVelMagnitude =ObVel.Magnitude(); //ToGet the magnitude of the force
  ObVel._z = atan(ObVel._y/ObVel._x);                 //To get the direction of the force taking in consideration that we are always 
  ObVel._z = ObVel._y < 0? ObVel._z + PI : ObVel._z;   // wanting to rotate into this direction == "Cirucle trajecotry"
  float leftVel = ObVelMagnitude/ radius - lr_wheels_dist_ / (2*radius) * ObVel._z; 
  float rightVel = ObVelMagnitude/ radius + lr_wheels_dist_ / (2*radius) * ObVel._z;

  //Change the data to rpm insted of m/s;
  Kinematics::output rpm;
  rpm.motor1 = leftVel/circumference_  * 60;
  rpm.motor2  = rightVel / circumference_ * 60;
  return rpm;
}

Kinematics::output Kinematics::getPWM(velocities ObVel)
{
  Kinematics::output rpm;
  Kinematics::output pwm;

  rpm = getRPM(ObVel);

  //convert from RPM to PWM
  //front-left motor
  pwm.motor1 = rpmToPWM(rpm.motor1);
  //rear-left motor
  pwm.motor2 = rpmToPWM(rpm.motor2);

  return pwm;
}

Kinematics::velocities Kinematics::getVelocities(output actualRPM, float theta)
{
  Kinematics::velocities vel;
  //We want to change the rpm of the motors to distance traveled in the  x y  coord
  //So we first change the data from rpm to m/s;

  actualRPM.motor1 = (actualRPM.motor1 *circumference_ / 60);
  actualRPM.motor2 = (actualRPM.motor2 *circumference_ / 60);

  //With the formulas we determine the velocities and send them;

  vel._x = (radius * actualRPM.motor1)/2 *cos(theta) + (radius * actualRPM.motor2) /2 *cos(theta);
  vel._y = (radius * actualRPM.motor1)/2 *sin(theta) + (radius * actualRPM.motor2) /2 *sin(theta);
  vel._z = -radius / lr_wheels_dist_ * actualRPM.motor1 +  radius / lr_wheels_dist_ * actualRPM.motor2; 
/*  float average_rpm_x = (float)(motor1 + motor2) / 2; // RPM
  //convert revolutions per minute to revolutions per second
  float average_rps_x = average_rpm_x / 60; // RPS
  vel.linear_x = (average_rps_x * circumference_); // m/s

  float average_rpm_a = (float)(motor2 - motor1) / 2;
  //convert revolutions per minute to revolutions per second
  float average_rps_a = average_rpm_a / 60;
  vel.angular_z =  (average_rps_a * circumference_) / (lr_wheels_dist_ / 2);*/

  return vel;

  //probably later this could be used to make odometry.
  //However since the forces from the computer will be constantly updated there would be no nececity to create inter odometry
  //With a litter PID taking the objective velocities of the motors would be enough to make a decent movement
}



int Kinematics::rpmToPWM(int rpm)
{
  //remap scale of target RPM vs MAX_RPM to PWM
  rpm = rpm > max_rpm_ ? max_rpm_ : rpm;    //avoid rpm going faster that the prestablish max
  return (((float) rpm / (float) max_rpm_) * pwm_res_);
}