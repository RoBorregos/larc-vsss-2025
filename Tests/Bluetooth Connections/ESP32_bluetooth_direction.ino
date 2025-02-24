#include "BluetoothSerial.h"

BluetoothSerial SerialBT;

void setup() {
  Serial.begin(115200); // Start serial communication for debugging
  SerialBT.begin("ESP32RECEIVER"); // Bluetooth device name
  Serial.println("Bluetooth Server started. Pair with ESP32RECEIVER from your laptop.");
  Serial.println(SerialBT.getBtAddressString()); // example: "E4:##:##:##:30:2A" depends on the esp32 
}

void loop() {
  if (SerialBT.available()) { // Check for incoming data
    String incoming = SerialBT.readString();
    Serial.print("Received from laptop: ");
    Serial.println(incoming);
  }
  delay(10); // Small delay to avoid flooding
}
