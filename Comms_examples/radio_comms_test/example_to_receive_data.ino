#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>




#define MotorRA 26
#define MotorRB 27
#define MotorR_PWM 25

#define MotorLA 20
#define MotorLB 21
#define MotorL_PWM 23

// Example function to drive motors in Arduino IDE
void Drive(int MotorL, int MotorR) {
  // Left motor control
  if (MotorL > 0) {
    digitalWrite(MotorLA, 1);
    digitalWrite(MotorLB, 0);
  } else if (MotorL < 0) {
    digitalWrite(MotorLA, 0);
    digitalWrite(MotorLB, 1);
  } else {
    digitalWrite(MotorLA, 0);
    digitalWrite(MotorLB, 0);
  }

  // Right motor control
  if (MotorR > 0) {
    digitalWrite(MotorRA, 1);
    digitalWrite(MotorRB, 0);
  } else if (MotorR < 0) {
    digitalWrite(MotorRA, 0);
    digitalWrite(MotorRB, 1);  // Fixed typo: Changed MotorLB to MotorRB
  } else {
    digitalWrite(MotorRA, 0);
    digitalWrite(MotorRB, 0);  // Fixed typo: Changed MotorLB to MotorRB
  }

  // Set motor speeds
  int Rspeed = abs(MotorR);
  int Lspeed = abs(MotorL);
  analogWrite(MotorL_PWM, Lspeed);
  analogWrite(MotorR_PWM, Rspeed);
}

RF24 radio(9, 10); // CE, CSN pins of the NRF24L01 modul
const byte address[6] = "00001"; // Address for the RF communication

void setup() {
  Serial.begin(9600);
  radio.begin();
  radio.openReadingPipe(0, address); 
  radio.setPALevel(RF24_PA_MIN); // Set power level to minimum
  radio.startListening(); // Start listening for incoming messages
  
  pinMode(MotorRA, OUTPUT);
  pinMode(MotorRB, OUTPUT);
  pinMode(MotorR_PWM, OUTPUT);
  pinMode(MotorLA, OUTPUT);
  pinMode(MotorLB, OUTPUT);
  pinMode(MotorL_PWM, OUTPUT);
}


struct RF_MESSAGE {
  uint8_t device_id; //Identifier of the device, set by the transmitter
  uint32_t data; //The data being transmitted
};

void loop() {


  if (radio.available()) {
    RF_MESSAGE msg;
    radio.read(&msg, sizeof(msg));
    if(msg.device_id == 1){
      if (msg.data == 8) {
        Drive(255, 255);
      } else if (msg.data == 2) {
        Drive(-255, -255);
      } else if (msg.data == 6) {  // Fixed comparison operator
        Drive(255, -255);
      } else if (msg.data == 4) {
        Drive(-255, 255);
      } else {
        Drive(0, 0);
      }
    }
  }
}
