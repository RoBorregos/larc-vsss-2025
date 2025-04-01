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
  
  

output Kinematics::getRPM(velocities ObVel) // the objective velocityes x,y,theta
{
//For now it is that x is the force and z the angule. However this would change depending on the development we could do later (odometry)
  
  float ObVelMagnitude = ObVel.Magnitude() * ConstVelDiff; //ToGet the magnitude of the force
  ObVel._z *= ConstThetaDiff;
  
  //Funciton to determine the vel of each wheel with the values of vel and theta dif
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
  pwm = rpmToPWM(rpm);

  return pwm;
}
//Function to determine the velocities of the robot in the x and y plane with the real wheel velocities
velocities Kinematics::getVelocities(output actualRPM, float theta)
{
  velocities vel;
  //Change from rpm to m/s
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