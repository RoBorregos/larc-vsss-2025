// Add this near the top of your ESP32 sketch, after your existing declarations
float x_coord = 0.0;
float y_coord = 0.0;

//deltaPositionTime
unsigned long previousTime = 0;
unsigned long currentTime;
float deltaTime;


// Device with motor control (ESP32 receiver)
#include <WiFi.h>
#include <WiFiUdp.h>
WiFiUDP udp;
//Kinematics library
#include <Kinematics.h>
#include <PID.h>

// Wi-Fi credentials
const char* ssid = "danielaASUS05";
const char* password = "Dw8619a6";

unsigned int localUdpPort = 1111;  // Port to listen on
char incomingPacket[255];          // Buffer for incoming packets
IPAddress otherDeviceIP(192,168,1,110); // Replace with other device's IP
const int otherDevicePort = 1234;
const int motorSpeed = 255;

// TB6612FNG Motor Driver pins
#define MotorA1 26 
#define MotorA2 27
#define MotorA_PWM 33

#define MotorB1 14
#define MotorB2 12
#define MotorB_PWM 25

//Encoders Pins
#define rEncoder 2
#define lEncoder 1
#define NoTicks 20

//Encoders variables
unsigned long lastRPMTime = 0;
unsigned long currentRPMTime;
float deltaRPMTime;
volatile int rpulses;
volatile int lpulses;

// Defining constants for control
#define MOTOR_MAX_RPM 240        // motor's maximum rpm
#define WHEEL_DIAMETER 0.06  // distance between front wheel and rear wheel
#define LR_WHEEL_DISTANCE 0.08    // distance between left wheel and right wheel
#define PWM_BITS 8              // microcontroller's PWM pin resolution. Arduino Uno/Mega Teensy is using 8 bits(0-255)
#define VelConst 1
#define ThetaConst 1

//Constants for PID
#define Rightkp 1
#define Rightki 0 
#define Rightkd 0

#define Leftkp 1
#define Leftki 0
#define Leftkd 0

//Kinematics
  Kinematics kinematics(MOTOR_MAX_RPM, WHEEL_DIAMETER, LR_WHEEL_DISTANCE, PWM_BITS, VelConst, ThetaConst);
  velocities Force;
  //PID 
  PID RPID = PID(Rightkp, Rightkd, Rightki);
  PID LPID = PID(Leftkp, Leftkd, Leftki);

void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid, password);

  // Configure motor pins
  pinMode(MotorA1, OUTPUT);
  pinMode(MotorA2, OUTPUT);
  pinMode(MotorA_PWM, OUTPUT);
  pinMode(MotorB1, OUTPUT);
  pinMode(MotorB2, OUTPUT);
  pinMode(MotorB_PWM, OUTPUT);
  
  // Initialize motors to stop
  Drive(0, 0);

  // Sets the encoders
   attachInterrupt(digitalPinToInterrupt(rEncoder), Rpulses, RISING);
   attachInterrupt(digitalPinToInterrupt(lEncoder), Lpulses, RISING);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected to Wi-Fi");
  Serial.printf("Device IP: %s, UDP port: %d\n", WiFi.localIP().toString().c_str(), localUdpPort);
  
  udp.begin(localUdpPort);  // Start UDP
}

void Rpulses(){
  rpulses++;
}
void Lpulses(){
  lpulses++;
}

void handleCommand(const char* command) {
  Serial.printf("Received command: %s\n", command);
  
  if(strcmp(command, "W") == 0) {
    Serial.println("Moving forward");
    Drive(motorSpeed, motorSpeed);
  }
  else if(strcmp(command, "S") == 0) {
    Serial.println("Moving backward");
    Drive(-motorSpeed, -motorSpeed);
  }
  else if(strcmp(command, "A") == 0) {
    Serial.println("Turning left");
    Drive(-motorSpeed, motorSpeed);  // Pivot left
  }
  else if(strcmp(command, "D") == 0) {
    Serial.println("Turning right");
    Drive(motorSpeed, -motorSpeed);  // Pivot right
  }
  else if(strcmp(command, " ") == 0 || strcmp(command, "STOP") == 0) {
    Serial.println("Stopping");
    Drive(0, 0);
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

//GetRPM
output GetRPM(){
  output rpm;
  currentRPMTime = millis();
  noInterrupts();
  rpm.motor2 = (rpulses / NoTicks) * (currentRPMTime - lastRPMTime) / 1000.0 * 60; // number of revolutions * deltaTime * 60 secs to get min
  rpm.motor1 = (lpulses / NoTicks) * (currentRPMTime - lastRPMTime) / 1000.0 * 60;
  lpulses = 0;
  rpulses = 0;
  lastRPMTime = currentRPMTime;
  interrupts();
  return rpm;
}


//Position Data
float x = 0;
float y = 0;
float z = PI/2;

void loop() {

  /*// Check for incoming packets
  int packetSize = udp.parsePacket();
  if (packetSize) {
    // Read the packet
    int len = udp.read(incomingPacket, sizeof(incomingPacket));
    
    // Check if we received exactly 8 bytes (two floats, 4 bytes each)
    if (len == 8) {
      // Extract the two float values from the packet
      memcpy(&x_coord, incomingPacket, 4);
      memcpy(&y_coord, incomingPacket + 4, 4);
      
      // Print the received coordinates
      Serial.printf("Received coordinates: x=%.2f, y=%.2f from %s\n", 
                    x_coord, y_coord, 
                    udp.remoteIP().toString().c_str());
    }
    else {
      Serial.println("Received packet with unexpected size");
    }
  }
  Force._x = x_coord - x;
  Force._y = y_coord - y;
  Force.setAngule();
  Force._z = Force.getThetaDif(Force._z,z) ;
  output Orpm = kinematics.getRPM(Force);*/

  //Get Real Vel
  output rpm = GetRPM();
  output Orpm;
  Orpm.motor1 = 240;
  Orpm.motor2 = 240;

  //velocities vel = kinematics.getVelocities(rpm , z);   //Vel in coord x y and z

  rpm.motor1 += LPID.GetCorrection(Orpm.motor1 - rpm.motor1);  // add correction from PID
  rpm.motor2 += RPID.GetCorrection(Orpm.motor2 - rpm.motor2);

  output pwm = kinematics.rpmToPWM(rpm);

  Drive(pwm.motor1, pwm.motor2);
  //Time count
  /*currentTime = micros();
  deltaTime = (currentTime - previousTime) / 1000000.0;
  x += deltaTime * vel._x;
  y += deltaTime * vel._y;
  z += deltaTime * vel._z;
  previousTime = currentTime;*/

}

