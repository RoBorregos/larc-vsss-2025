#include <WiFi.h>
#include <WiFiUdp.h>
WiFiUDP udp;

// Wi-Fi credentials
const char* ssid = "exampleSSID"; // Replace with your Wi-Fi SSID
const char* password = "examplePassword"; // Replace with your Wi-Fi password
IPAddress otherDeviceIP(192,168,1,6); // Replace with computer id

unsigned int localUdpPort = 1235;  // Port to listen on
char incomingPacket[255];          // Buffer for incoming packets

float x_coord ;
float y_coord ;

void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected to Wi-Fi");
  Serial.printf("Device IP: %s, UDP port: %d\n", WiFi.localIP().toString().c_str(), localUdpPort);
  udp.begin(localUdpPort);  // Start UDP
}

void loop() {
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
}
