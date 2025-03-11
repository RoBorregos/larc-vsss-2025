// Device with motor control (ESP32 receiver)
#include <WiFi.h>
#include <WiFiUdp.h>
WiFiUDP udp;

// Wi-Fi credentials
const char* ssid = "RoBorregos2";
const char* password = "RoBorregos2024";

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

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected to Wi-Fi");
  Serial.printf("Device IP: %s, UDP port: %d\n", WiFi.localIP().toString().c_str(), localUdpPort);
  
  udp.begin(localUdpPort);  // Start UDP
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

void loop() {
  // Check for incoming packets
  int packetSize = udp.parsePacket();
  if (packetSize) {
    int len = udp.read(incomingPacket, sizeof(incomingPacket));
    if (len > 0) {
      incomingPacket[len] = '\0'; // Null-terminate the string
      Serial.printf("Received packet from %s: %s\n", udp.remoteIP().toString().c_str(), incomingPacket);
      handleCommand(incomingPacket);
    }
  }
}