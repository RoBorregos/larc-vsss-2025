#include "Arduino.h"
#include "Kinematics.h"

Kinematics::Kinematics(float motor_max_rpm, float wheel_diameter, float lr_wheels_dist, int pwm_bits, float vel, float theta){
  circumference_ = PI * wheel_diameter;
  max_rpm_ = motor_max_rpm;
  lr_wheels_dist_ = lr_wheels_dist;
  pwm_res_  = pow ( 2, pwm_bits) - 1;
  max_vel = (float)motor_max_rpm * circumference_;
  radius = wheel_diameter/2;
  ConstThetaDiff = theta;
  ConstVelDiff = vel;
}
  
  

output Kinematics::getRPM(velocities ObVel) //Obtener las velocidades de las llantas con el angulo y la velocidad
{
  output rpm;
  float ObVelMagnitude = ObVel.Magnitude() * ConstVelDiff; //ToGet the magnitude of the vel
  ObVel._z *= ConstThetaDiff;


  //Funciton to determine the vel of each wheel with the values of the robot
  float leftVel = ObVelMagnitude/ radius - lr_wheels_dist_ / (2*radius) * ObVel._z; 
  float rightVel = ObVelMagnitude/ radius + lr_wheels_dist_ / (2*radius) * ObVel._z;

  //Change the data to rpm insted of m/s;
  rpm.motor1 = leftVel/circumference_  * 60;
  rpm.motor2  = rightVel / circumference_ * 60;
  return rpm;
}

output Kinematics::getPWM(velocities ObVel)
{
  output rpm;
  output pwm;

  rpm = getRPM(ObVel);

  //convert from RPM to PWM
  pwm = rpm.rpmToPWM(rpm);

  return pwm;
}
//Function to determine the velocities of the robot in the x and y plane with the real wheel velocities
velocities Kinematics::getVelocities(output actualRPM, float theta)
{
  velocities vel;
  //Change from rpm to rad/s
  actualRPM.motor1 = (actualRPM.motor1 *2 *PI/ 60);
  actualRPM.motor2 = (actualRPM.motor2 *2 *PI/ 60);
  
  //With the formulas we determine the velocities and send them;
  vel._x = radius * (actualRPM.motor1 + actualRPM.motor2)/2 *cos(theta);
  vel._y = radius * (actualRPM.motor1 + actualRPM.motor2)/2 *sin(theta);
  vel._z = radius / lr_wheels_dist_ *(actualRPM.motor2 - actualRPM.motor1); 
  return vel;

}


