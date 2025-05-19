#include "Arduino.h"
#include "Kinematics.h"

Kinematics::Kinematics(int motor_max_rpm, float wheel_diameter, float fr_wheels_dist, float lr_wheels_dist, int pwm_bits):
  circumference_(PI * wheel_diameter),
  max_rpm_(motor_max_rpm),
  fr_wheels_dist_(fr_wheels_dist),
  lr_wheels_dist_(lr_wheels_dist),
  pwm_res_ (pow(2, pwm_bits) - 1)
{
}

Kinematics::output Kinematics::getRPM(float linear_x, float linear_y, float angular_z)
{
  //convert m/s to m/min
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
  rpm.motor4 = x_rpm_ - y_rpm_ + tan_rpm_;

  return rpm;
}

Kinematics::output Kinematics::getPWM(float linear_x, float linear_y, float angular_z)
{
  Kinematics::output rpm;
  Kinematics::output pwm;

  rpm = getRPM(linear_x, linear_y, angular_z);

  //convert from RPM to PWM
  //front-left motor
  pwm.motor1 = rpmToPWM(rpm.motor1);
  //rear-left motor
  pwm.motor2 = rpmToPWM(rpm.motor2);

  //front-right motor
  pwm.motor3 = rpmToPWM(rpm.motor3);
  //rear-right motor
  pwm.motor4 = rpmToPWM(rpm.motor4);

  return pwm;
}

Kinematics::velocities Kinematics::getVelocities(int motor1, int motor2)
{
  Kinematics::velocities vel;

  float average_rpm_x = (float)(motor1 + motor2) / 2; // RPM
  //convert revolutions per minute to revolutions per second
  float average_rps_x = average_rpm_x / 60; // RPS
  vel.linear_x = (average_rps_x * circumference_); // m/s

  float average_rpm_a = (float)(motor2 - motor1) / 2;
  //convert revolutions per minute to revolutions per second
  float average_rps_a = average_rpm_a / 60;
  vel.angular_z =  (average_rps_a * circumference_) / (lr_wheels_dist_ / 2);

  return vel;
}

Kinematics::velocities Kinematics::getVelocities(int motor1, int motor2, int motor3, int motor4)
{
  Kinematics::velocities vel;

  float average_rpm_x = (float)(motor1 + motor2 + motor3 + motor4) / 4; // RPM
  //convert revolutions per minute to revolutions per second
  float average_rps_x = average_rpm_x / 60; // RPS
  vel.linear_x = (average_rps_x * circumference_); // m/s

  float average_rpm_y = (float)(-motor1 + motor2 + motor3 - motor4) / 4; // RPM
  //convert revolutions per minute in y axis to revolutions per second
  float average_rps_y = average_rpm_y / 60; // RPS
  vel.linear_y = (average_rps_y * circumference_); // m/s

  float average_rpm_a = (float)(-motor1 + motor2 - motor3 + motor4) / 4;
  //convert revolutions per minute to revolutions per second
  float average_rps_a = average_rpm_a / 60;
  vel.angular_z =  (average_rps_a * circumference_) / ((fr_wheels_dist_ / 2) + (lr_wheels_dist_ / 2));

  return vel;
}

int Kinematics::rpmToPWM(int rpm)
{
  //remap scale of target RPM vs MAX_RPM to PWM
  return (((float) rpm / (float) max_rpm_) * pwm_res_);
}