#include "BluetoothSerial.h"

#define MotorRA 14
#define MotorRB 13
#define MotorR_PWM 12

#define MotorLA 20
#define MotorLB 21
#define MotorL_PWM 23

BluetoothSerial SerialBT;

void setup() {
  Serial.begin(115200); // Start serial communication for debugging
  SerialBT.begin("ESP32RECEIVER"); // Bluetooth device name
  Serial.println("Bluetooth Server started. Pair with ESP32RECEIVER from your laptop.");
  Serial.println(SerialBT.getBtAddressString()); // example: "E4:##:##:##:30:2A" depends on the esp32 
  pinMode(MotorRA, OUTPUT);
  pinMode(MotorRB, OUTPUT);
  pinMode(MotorR_PWM, OUTPUT);
  pinMode(MotorLA, OUTPUT);
  pinMode(MotorLB, OUTPUT);
  pinMode(MotorL_PWM, OUTPUT);
}

void Drive(int MotorL, int MotorR){
  if(MotorL > 0){
    digitalWrite(MotorLA, 1);
    digitalWrite(MotorLB, 0);
  }else if(MotorL < 0 ){
    digitalWrite(MotorLA, 0);
    digitalWrite(MotorLB, 1);
  }else{
    digitalWrite(MotorLA, 0);
    digitalWrite(MotorLB, 0);
  }

  if(MotorR> 0){
    digitalWrite(MotorRA, 1);
    digitalWrite(MotorRB, 0);
  }else if(MotorR< 0 ){
    digitalWrite(MotorRA, 0);
    digitalWrite(MotorRB, 1);
  }else{
    digitalWrite(MotorRA, 0);
    digitalWrite(MotorLB, 0);
  }
  int Rspeed = abs(MotorR);
  int Lspeed = abs(MotorL);
  analogWrite(MotorL_PWM, Lspeed);
  analogWrite(MotorR_PWM, Rspeed);


}

void loop() {
  if (SerialBT.available()) { // Check for incoming data
    String incoming = SerialBT.readString();
    Serial.print("Received from laptop: ");
    if(incoming == "W"){
      Drive(255,255);
    }else if(incoming == "S"){
      Drive(-255,-255);
    }else if(incoming = "R"){
      Drive(255, -255);
    }else if(incoming == "L"){
      Drive(-255, 255);
    }else {
      Drive(0,0);
    }
  }
  delay(10); // Small delay to avoid flooding
}
