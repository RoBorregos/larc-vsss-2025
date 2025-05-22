// Variables to store message
float pwmM1 ;
float pwmM2 ;

float x_coord;
float y_coord;
//

//deltaTime for position
unsigned long previousTime = 0;
unsigned long currentTime;
float deltaTime;

//deltaTime for communication
unsigned long previousUDPTime = 0;
unsigned long currentUDPTime = 0;

//Libraries for UDP communication
#include <WiFi.h>
#include <WiFiUdp.h>
WiFiUDP udp;
//HomemadeLibraries
#include <Lib/Kinematics.cpp>
#include <Lib/PID.cpp>

// Wi-Fi credentials
const char* ssid = "Tenda_18D250";
const char* password = "**********";

unsigned int localUdpPort = 1001;       // Port to listen on
const int packetSize = 8;              // Size of 2 floats (4 bytes each)
byte packetBuffer[packetSize];         // Buffer to hold incoming packet
                                        // Buffer for incoming packets
const int motorSpeed = 255;

// MotorPins
#define MotorA1 19
#define MotorA2 17 // new prototype 
#define MotorA_PWM 1


#define MotorB1 2
#define MotorB2 21
#define MotorB_PWM 0

//Encoders Pins
#define rEncoder 23
#define lEncoder 16
#define NoTicks 350.0

//deltaTime for RPM
unsigned long lastRPMTime = 0;
unsigned long currentRPMTime;
float deltaRPMTime;

//vriables for encoders
volatile unsigned int rpulses;
volatile unsigned int lpulses;

// Defining constants for control
#define MOTOR_MAX_RPM 320      // motor's maximum rpm
#define WHEEL_DIAMETER 0.06     // distance between front wheel and rear wheel
#define LR_WHEEL_DISTANCE 0.076  // distance between left wheel and right wheel
#define PWM_BITS 8              // microcontroller's PWM pin resolution. Arduino Uno/Mega Teensy is using 8 bits(0-255)
#define VelConst 0.33         
#define ThetaConst 0.66

//Constants for PID
#define RP 0.1
#define RI 0.003
#define RD 0.045

#define LP 0.1
#define LI 0.003
#define LD 0.045

//Kinematics
Kinematics kinematics(MOTOR_MAX_RPM, WHEEL_DIAMETER, LR_WHEEL_DISTANCE, PWM_BITS, VelConst, ThetaConst);

//PID for two wheels
PID RPID = PID(RP, RI, RD);
PID LPID = PID(LP, LI, LD);

//Position Data
float x = 0;
float y = 0;
float z = PI / 2;
velocities vel, fin;
velocities Force;
output Orpm, Opwm, rpm, pwm;

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

  // Configure motor pins
  pinMode(MotorA1, OUTPUT);
  pinMode(MotorA2, OUTPUT);
  pinMode(MotorB1, OUTPUT);
  pinMode(MotorB2, OUTPUT);
  ledcAttach(MotorB_PWM, 10000,8);
  ledcAttach(MotorA_PWM, 10000,8);

  // Initialize motors to stop
  Drive(0, 0);

  // Sets the encoders
  pinMode(rEncoder, INPUT_PULLUP);
  pinMode(lEncoder, INPUT_PULLUP);
  attachInterrupt(rEncoder, Rpulses, RISING);
  attachInterrupt(lEncoder, Lpulses, RISING);

  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected to Wi-Fi");
  Serial.printf("Device IP: %s, UDP port: %d\n", WiFi.localIP().toString().c_str(), localUdpPort);
  
  udp.begin(localUdpPort);  // Start UDP*/
  rpulses = 0;
  lpulses = 0;

  //Initialize all timers
  lastRPMTime = micros();
  currentRPMTime = micros();
  previousTime = micros();
  currentTime = micros();
  previousUDPTime = millis();
  currentUDPTime = millis();
}

void Rpulses() {
  rpulses++;
}
void Lpulses() {
  lpulses++;
}




// Motor control function for TB6612FNG
void Drive(int MotorL, int MotorR) {
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
  output temp;
  currentRPMTime = micros();
  noInterrupts();
  if(currentRPMTime - lastRPMTime > 0){
    temp.motor2 = ((float)rpulses / NoTicks) /((currentRPMTime - lastRPMTime) / 1000000.0 )* 60;  // number of revolutions * deltaTime(in milis) * 60sec = to get rpm
    temp.motor1 = ((float)lpulses / NoTicks) /((currentRPMTime - lastRPMTime) / 1000000.0 )* 60;
  }
//Because we only have one pin to measure the rpm,
// the way to determine if the rpm is positive or negative
// is by giving it the sign of the given pwm.
  temp.motor1 *= signbit(pwm.motor1) ? -1 : 1;
  temp.motor2 *= signbit(pwm.motor2) ? -1 : 1;
  lpulses = 0;
  rpulses = 0;
  lastRPMTime = currentRPMTime;
  interrupts();
  return temp;
}


bool inic = true;
unsigned long a = 0;
void loop() {
  // Mini delay
  
  if(inic){
    inic = false;
    delay(5000);
  }
  
  //Code to recieve data from the estrategist code
      int packetSize = udp.parsePacket();
      currentUDPTime = millis();
      bool deltaUDP = (currentUDPTime - deltaUDP ) > 30;
      if (packetSize && deltaUDP ) {
        // Read the packet into the buffer
        udp.read(packetBuffer, sizeof(packetBuffer));

        // Convert bytes to floats
        memcpy(&pwmM1, &packetBuffer[0], sizeof(float));
        memcpy(&pwmM2, &packetBuffer[4], sizeof(float));
        previousUDPTime = currentUDPTime;

      }
      //Check for nan values
      if (isnan(pwmM1) || isnan(pwmM2)) {
          pwmM1 = 0; // Set default value
          pwmM2 = 0; // Set default value
        }

    VelocityTracker();
   


}

void VelocityTracker()
{
    Orpm.motor1 = pwmM1;
    Orpm.motor2 = pwmM2;
    rpm = GetRPM();
  //Basic PID class where the error is considered as the difference
  //between the current PWM of the wheels and the target PWM
  //to then be added to the current PWM limiting the pwm range to -255 < pwm < 255
    float Lcorr = LPID.GetCorrection(Orpm.motor1 - rpm.motor1);  
    float Rcorr = RPID.GetCorrection(Orpm.motor2 - rpm.motor2);
    pwm.motor1 += Lcorr ;
    pwm.motor2 += Rcorr ;
    pwm.motor1 = abs(pwm.motor1) > 254  ? 254 * (signbit(pwm.motor1) ? -1 : 1): pwm.motor1; 
    pwm.motor2 = abs(pwm.motor2) > 254  ? 254 * (signbit(pwm.motor2) ? -1 : 1) : pwm.motor2; 

    //Print info to debug errors.
    Orpm.Print(); Serial.print("\nR");
    rpm.Print(); Serial.print("\n               P");
    pwm.Print();Serial.print("\n                             O");
    Drive((int)pwm.motor1,(int)pwm.motor2);
    

  delay(5);

}


void PositionTracker(){

  fin._x = x_coord/-100;
  fin._y = y_coord/100; //Info a recivir
  rpm = GetRPM();
  vel = kinematics.getVelocities(rpm, z);  //Obtener la velocidad del robot en las coordenadas globales
  // Update Position
  currentTime = micros();
  deltaTime = (currentTime - previousTime) / 1000000.0;
  x += deltaTime * vel._x;
  y += deltaTime * vel._y;
  z += deltaTime * vel._z;
  previousTime = currentTime;

  velocities Pos; //Facilidad de Manejo
  Pos._x = x; Pos._y = y; Pos._z = z;
  //Verificacion que el angulo este entre 0<theta<2*pi
  z = z < 0 ? 2 * PI + z : z;
  z = z > 2 * PI ? z - 2 * PI : z;
  //Encontrar el vector de diferencia
  Force._x = x_coord - x;
  Force._y = y_coord - y;
  //Encontrar el angulo de este vector diferencia
  Force.setAngule();
  //Diferencia de angulos
  Force._z = Force.getThetaDif(Force._z, z);   
  //Consigue las velocidades (en pwm) necesarias para cada uno de los motores
  Opwm = kinematics.getPWM(Force);
  //Los escala para que las velocidades de las llantas siempre sean proporcionales
  // entre si. O sea de que 510 y 420 seran 255 y 210 en vez de solo 255 255
  Opwm.Scale(255);
  //Clase basica de PID en donde se considera el error la diferencia 
  //entre el pwm que se tienen actualmente las llantas y el pwm objetivo
  // para luego ser sumados al pwm actual
  float Lcorr = LPID.GetCorrection(Opwm.motor1 - pwm.motor1);  
  float Rcorr = RPID.GetCorrection(Opwm.motor2 - pwm.motor2);
  pwm.motor1 += Lcorr ;
  pwm.motor2 += Rcorr ;
  //Volver a escalar por si acaso
  pwm.Scale(255);
  //Impresion de Info
  Opwm.Print(); Serial.print("\n              ");
  pwm.Print(); Serial.print("\n                               ");
  Pos.Print(); Serial.print("\n                                                       ");
  fin.Print(); Serial.print("\n");
  Drive(pwm.motor1,pwm.motor2);

  
}
