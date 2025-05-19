#include <WiFi.h>
#include <WiFiUdp.h>
WiFiUDP udp;


//Wi-Fi credentials
const char* ssid = "CRONO 7087";
const char* password = "&839a33N";
//IPAddress otherDeviceIP(192,168,1,6); // Replace with computer id


unsigned int localUdpPort = 1001;  // Port to listen on
char incomingPacket[255];          // Buffer for incoming packets


/*
float x_coord ;
float y_coord ;
*/

// TB6612FNG Motor Driver pins
#define MotorA1 2 
#define MotorA2 21 // new prototype 
#define MotorA_PWM 0


#define MotorB1 17
#define MotorB2 19
#define MotorB_PWM 1


/*
void Drive2(int b, int a){

  if (a > 0){
    ledcWrite(MotorA1, a);
    ledcWrite(MotorA2, 0);
  }else if(a < 0){
    ledcWrite(MotorA1, 0);
    ledcWrite(MotorA2, abs(a));
  }else{
    ledcWrite(MotorA1,0);
    ledcWrite(MotorA2,0);
  }

  if (b> 0){
    ledcWrite(MotorB1, b);
    ledcWrite(MotorB2, 0);
  }else if(b< 0){
    ledcWrite(MotorB1, 0);
    ledcWrite(MotorB2, abs(b));
  }else{
    ledcWrite(MotorB1,0);
    ledcWrite(MotorB2,0);
  }
}
*/

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
//  // Set motor speeds
  ledcWrite(MotorA_PWM, abs(MotorL));
  ledcWrite(MotorB_PWM, abs(MotorR));
  //Serial.print(MotorL);
}


void setup() {
  pinMode(MotorA1, OUTPUT);
  pinMode(MotorA2, OUTPUT);
  pinMode(MotorB1, OUTPUT);
  pinMode(MotorB2, OUTPUT);
  ledcAttach(MotorA_PWM, 10000, 8);
  ledcAttach(MotorB_PWM, 10000, 8);



  // Configure motor pins
  /*
  Serial.begin(115200);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected to Wi-Fi");
  Serial.printf("Device IP: %s, UDP port: %d\n", WiFi.localIP().toString().c_str(), localUdpPort);
  udp.begin(localUdpPort);  // Start UDP
  */
}

void loop() {
  /*
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
  */
  Drive(255,255);  
}
