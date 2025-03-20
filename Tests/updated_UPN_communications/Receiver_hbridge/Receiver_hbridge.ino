// Device with motor control (ESP32 receiver)
/*#include "WiFi.h"
#include "WiFiUdp.h"*/
#include "Kinematics.h"
#include "PID.h"
/*WiFiUDP udp;

// Wi-Fi credentials
const char* ssid = "RoBorregos2";
const char* password = "RoBorregos2025";*/

/*unsigned int localUdpPort = 1111;  // Port to listen on
char incomingPacket[255];          // Buffer for incoming packets
IPAddress otherDeviceIP(192,168,1,110); // Replace with other device's IP*/
const int otherDevicePort = 1234;
const int motorSpeed = 255;

// TB6612FNG Motor Driver pins
#define MotorA1 26 
#define MotorA2 27
#define MotorA_PWM 33

#define MotorB1 14
#define MotorB2 12
#define MotorB_PWM 25

// Defining constants for control
#define MOTOR_MAX_RPM 240        // motor's maximum rpm
#define WHEEL_DIAMETER 0.06  // distance between front wheel and rear wheel
#define LR_WHEEL_DISTANCE 0.08    // distance between left wheel and right wheel
#define PWM_BITS 8              // microcontroller's PWM pin resolution. Arduino Uno/Mega Teensy is using 8 bits(0-255)
#define VelConst 1
#define ThetaConst 1
//Faltar medir los valores despues con exactitud
  //Variables de Vel para los motores
  Kinematics kinematics(MOTOR_MAX_RPM, WHEEL_DIAMETER, LR_WHEEL_DISTANCE, PWM_BITS, VelConst, ThetaConst);
  velocities Force;
// Definir variables de Motores
/*
#define rkp 1
#define rkd 0 
#define rki 0
#define lkp 1
#define lki 0 
#define lkd 0
  RPID = PID(rkp, rkd, rki);
  LPID = PID(lkp, lkd, lki);

*/

//Definir PID
  PID RWheelPID(0,0,0);
  PID LWheelPID(0,0,0);

void setup() {
  Serial.begin(115200);
  //WiFi.begin(ssid, password);

  // Configure motor pins
  pinMode(MotorA1, OUTPUT);
  pinMode(MotorA2, OUTPUT);
  pinMode(MotorA_PWM, OUTPUT);
  pinMode(MotorB1, OUTPUT);
  pinMode(MotorB2, OUTPUT);
  pinMode(MotorB_PWM, OUTPUT);
  
  // Initialize motors to stop
  Drive(0, 0);

 /* while (WiFi.status() != WL_CONNECTED) {
    Serial.println(WiFi.status());
    Serial.print(".");
  }
  Serial.println("\nConnected to Wi-Fi");
  Serial.printf("Device IP: %s, UDP port: %d\n", WiFi.localIP().toString().c_str(), localUdpPort);
  
  udp.begin(localUdpPort);  // Start UDP*/
}

void handleCommand(const char* command) {
  //Serial.printf("Received command: %s\n", command);
  
  if(strcmp(command, "W") == 0) {
    Serial.println("Moving forward");
    Force._x = 1;
    Force._y = 0;
    //Drive(motorSpeed, motorSpeed);
  }
  else if(strcmp(command, "S") == 0) {
    Serial.println("Moving backward");
    Force._x = -1;
    Force._y = 0;
    //Drive(-motorSpeed, -motorSpeed);
  }
  else if(strcmp(command, "A") == 0) {
    Serial.println("Turning left");
    Force._x = 0;
    Force._y = -1;
    //Drive(-motorSpeed, motorSpeed);  // Pivot left
  }
  else if(strcmp(command, "D") == 0) {
    Force._x = 0;
    Force._y = 1;
    Serial.println("Turning right");
    //Drive(motorSpeed, -motorSpeed);  // Pivot right
  }
  else if(strcmp(command, " ") == 0 || strcmp(command, "STOP") == 0) {
    Serial.println("Stopping");
    //Drive(0, 0);
  }
  else {
    Serial.println("Unknown command");
  }

  
}

// Motor control function for TB6612FNG
void Drive(int MotorL, int MotorR){
  // Control left motor (Motor A)
  if(MotorL > 0){
    digitalWrite(MotorA1, HIGH);
    digitalWrite(MotorA2, LOW);
  } else if(MotorL < 0){
    digitalWrite(MotorA1, LOW);
    digitalWrite(MotorA2, HIGH);
  } else {
    digitalWrite(MotorA1, LOW);
    digitalWrite(MotorA2, LOW);
  }

  // Control right motor (Motor B)
  if(MotorR > 0){
    digitalWrite(MotorB1, HIGH);
    digitalWrite(MotorB2, LOW);
  } else if(MotorR < 0){
    digitalWrite(MotorB1, LOW);
    digitalWrite(MotorB2, HIGH);
  } else {
    digitalWrite(MotorB1, LOW);
    digitalWrite(MotorB2, LOW);
  }

  // Set motor speeds
  analogWrite(MotorA_PWM, abs(MotorL));
  analogWrite(MotorB_PWM, abs(MotorR));
}

void loop() {

  Force._x = -0.2;
  Force._y = 0.2;

  output pwm = kinematics.getPWM(Force);
  Force.setAngule();
  //GetRPM();
  // float Rcorrection = RPID.GetCorrection(RGetRPM - Force.motor1);
  // float Ldif = LPID.GetCorrection(LGetRPM - Force.motor2);
  //Drive(rpm.motor1 + Rdif, rpm.motor2 + Ldif); 
  /*
  rpm.motor1 += Rcorrection;
  rpm.motor2 += Lcorrection;
  kinematics.rpmToPWM(rpm);

  */
  Force._z = Force.getThetaDif(Force._z, PI/2);
  Serial.print(Force._x); Serial.print(" "); Serial.print(Force._y); Serial.print(" "); Serial.println(Force._z);
  Serial.println(kinematics.max_vel);
  output rpm = kinematics.getRPM(Force);
  Serial.print("                ");Serial.print(rpm.motor1); Serial.print(" "); Serial.println(rpm.motor2);
  Serial.print("                ");Serial.print(pwm.motor1); Serial.print(" ");Serial.println(pwm.motor2);
  Drive(pwm.motor1,pwm.motor2);
  delay(3000);
  //velocities vel = kinematics.getVelocities(pwm.motor1, pwm.motor2);
}