// Controller Device (ESP32 sender)
#include <WiFi.h>
#include <WiFiUdp.h>
WiFiUDP udp;

// Wi-Fi credentials
const char* ssid = "exampleSSID"; // Replace with your Wi-Fi SSID
const char* password = "examplePassword"; // Replace with your Wi-Fi password

// Ports
unsigned int localUdpPort = 1234;  // Port to listen on
char incomingPacket[255];          // Buffer for incoming packets

// Motor Device's IP and port
IPAddress motorDeviceIP(192,168,1,105); // Replace with the motor device's IP
const int motorDevicePort = 1111;

void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid, password);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected to Wi-Fi");
  Serial.printf("Controller IP: %s, UDP port: %d\n", WiFi.localIP().toString().c_str(), localUdpPort);
  
  udp.begin(localUdpPort);
}

void sendCommand(const char* command) {
  udp.beginPacket(motorDeviceIP, motorDevicePort);
  udp.write((const uint8_t*)command, strlen(command));
  udp.endPacket();
  Serial.printf("Sent command: %s\n", command);
}

void loop() {
  if (Serial.available() > 0) {
    char command = Serial.read();
    
    // Convert to uppercase
    if (command >= 'a' && command <= 'z') {
      command = command - 'a' + 'A';
    }
    
    // Send valid commands
    if (command == 'W' || command == 'A' || command == 'S' || command == 'D' || command == ' ') {
      char cmdStr[2] = {command, '\0'};
      sendCommand(cmdStr);
    }
  }
  
  // Check for any response from the motor device
  int packetSize = udp.parsePacket();
  if (packetSize) {
    int len = udp.read(incomingPacket, sizeof(incomingPacket));
    if (len > 0) {
      incomingPacket[len] = '\0';
      Serial.printf("Response from motor device: %s\n", incomingPacket);
    }
  }
  
  delay(10); // Small delay to prevent flooding
}