// Add this near the top of your ESP32 sketch, after your existing declarations
float x_coord = 0.5;
float y_coord = 0.5;

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

unsigned int localUdpPort = 1111;           // Port to listen on
char incomingPacket[255];                   // Buffer for incoming packets
IPAddress otherDeviceIP(192, 168, 1, 110);  // Replace with other device's IP
const int otherDevicePort = 1234;
const int motorSpeed = 255;

// TB6612FNG Motor Driver pins

#define MotorA1 18
#define MotorA2 5
#define MotorA_PWM 22

#define MotorB1 19
#define MotorB2 21
#define MotorB_PWM 23

//Encoders Pins
#define rEncoder 25
#define lEncoder 33
#define NoTicks 20

//Encoders variables
unsigned long lastRPMTime = 0;
unsigned long currentRPMTime;
float deltaRPMTime;
volatile int rpulses;
volatile int lpulses;

// Defining constants for control
#define MOTOR_MAX_RPM 240       // motor's maximum rpm
#define WHEEL_DIAMETER 0.06     // distance between front wheel and rear wheel
#define LR_WHEEL_DISTANCE 0.08  // distance between left wheel and right wheel
#define PWM_BITS 8              // microcontroller's PWM pin resolution. Arduino Uno/Mega Teensy is using 8 bits(0-255)
#define VelConst 0.7
#define ThetaConst 3.5

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
  //WiFi.begin(ssid, password);

  // Configure motor pins
  pinMode(MotorA1, OUTPUT);
  pinMode(MotorA2, OUTPUT);
  ledcAttach(MotorA_PWM, 10000, 8);
  pinMode(MotorB1, OUTPUT);
  pinMode(MotorB2, OUTPUT);
  ledcAttach(MotorB_PWM, 10000, 8);

  // Initialize motors to stop
  //Drive(0, 0);

  // Sets the encoders
  attachInterrupt(digitalPinToInterrupt(rEncoder), Rpulses, RISING);
  attachInterrupt(digitalPinToInterrupt(lEncoder), Lpulses, RISING);
  /*
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected to Wi-Fi");
  Serial.printf("Device IP: %s, UDP port: %d\n", WiFi.localIP().toString().c_str(), localUdpPort);
  
  udp.begin(localUdpPort);  // Start UDP*/

  previousTime = micros();
}

void Rpulses() {
  rpulses++;
}
void Lpulses() {
  lpulses++;
}

void handleCommand(const char* command) {
  Serial.printf("Received command: %s\n", command);

  if (strcmp(command, "W") == 0) {
    Serial.println("Moving forward");
    Drive(motorSpeed, motorSpeed);
  } else if (strcmp(command, "S") == 0) {
    Serial.println("Moving backward");
    Drive(-motorSpeed, -motorSpeed);
  } else if (strcmp(command, "A") == 0) {
    Serial.println("Turning left");
    Drive(-motorSpeed, motorSpeed);  // Pivot left
  } else if (strcmp(command, "D") == 0) {
    Serial.println("Turning right");
    Drive(motorSpeed, -motorSpeed);  // Pivot right
  } else if (strcmp(command, " ") == 0 || strcmp(command, "STOP") == 0) {
    Serial.println("Stopping");
    Drive(0, 0);
  } else {
    Serial.println("Unknown command");
  }
}


// Motor control function for TB6612FNG
void Drive(int MotorL, int MotorR) {
  // Control left motor (Motor A)
  if (MotorL > 0) {
    digitalWrite(MotorA1, HIGH);
    digitalWrite(MotorA2, LOW);
  } else if (MotorL < 0) {
    digitalWrite(MotorA1, LOW);
    digitalWrite(MotorA2, HIGH);
  } else {
    digitalWrite(MotorA1, LOW);
    digitalWrite(MotorA2, LOW);
  }

  // Control right motor (Motor B)
  if (MotorR > 0) {
    digitalWrite(MotorB1, HIGH);
    digitalWrite(MotorB2, LOW);
  } else if (MotorR < 0) {
    digitalWrite(MotorB1, LOW);
    digitalWrite(MotorB2, HIGH);
  } else {
    digitalWrite(MotorB1, LOW);
    digitalWrite(MotorB2, LOW);
  }

  // Set motor speeds
  ledcWrite(MotorA_PWM, abs(MotorL));
  ledcWrite(MotorB_PWM, abs(MotorR));
}

//GetRPM
output GetRPM() {
  output rpm;
  currentRPMTime = millis();
  noInterrupts();
  rpm.motor2 = (rpulses / NoTicks) * (currentRPMTime - lastRPMTime) / 1000.0 * 60;  // number of revolutions * deltaTime * 60 secs to get min
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
float z = PI / 2;
velocities vel;
output Orpm, rpm;

void loop() {

  // Check for incoming packets
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
    } else {
      Serial.println("Received packet with unexpected size");
    }
  }

  z = z < 0 ? 2 * PI + z : z;
  z = z > 2 * PI ? z - 2 * PI : z;

  // Time
  currentTime = micros();
  if (currentTime > previousTime) {
    deltaTime = (currentTime - previousTime) / 1000000.0;
    x += deltaTime * vel._x;
    y += deltaTime * vel._y;
    z += deltaTime * vel._z;
    previousTime = currentTime;
  }
  Force._x = x_coord - x;
  Force._y = y_coord - y;
  Force.setAngule();
  Force._z = Force.getThetaDif(Force._z, z);
  Orpm = kinematics.getRPM(Force);
  //Get Real Vel
  rpm = GetRPM();
  vel = kinematics.getVelocities(rpm, z);                      //Vel in coord x y and z
  float Lcorr = LPID.GetCorrection(Orpm.motor1 - rpm.motor1);  // add correction from PID
  float Rcorr = RPID.GetCorrection(Orpm.motor2 - rpm.motor2);
  rpm.motor1 += Lcorr - Rcorr * 0.5;
  rpm.motor2 += Rcorr - Lcorr * 0.5;
  output pwm = kinematics.rpmToPWM(Orpm);
  digitalWrite(MotorB1, HIGH);
  digitalWrite(MotorB2, LOW);
  ledcWrite(MotorB_PWM, 255);

  digitalWrite(MotorA1, HIGH);
  digitalWrite(MotorA2, LOW);
  ledcWrite(MotorA_PWM, 255);

  Serial.print("                   ");
  Serial.print(x);
  Serial.print(" ");
  Serial.print(y);
  Serial.print(" ");
  Serial.println(z);

  //Drive(255,255);
}
