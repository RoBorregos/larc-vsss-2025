// Add this near the top of your ESP32 sketch, after your existing declarations
//destiny data
float x_coord ;
float y_coord ;


//deltaPositionTime
unsigned long previousTime = 0;
unsigned long currentTime;
float deltaTime;

//Connection Delta Time
unsigned long previousUDPTime = 0;
unsigned long currentUDPTime = 0;

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

unsigned int localUdpPort = 1234;           // Port to listen on
const int packetSize = 8;       // Size of 2 floats (4 bytes each)
byte packetBuffer[packetSize];  // Buffer to hold incoming packet
              // Buffer for incoming packets
const int motorSpeed = 255;

// TB6612FNG Motor Driver pins

#define MotorA1 19
#define MotorA2 18
#define MotorA_PWM 0

#define MotorB1 27
#define MotorB2 26
#define MotorB_PWM 1

//Encoders Pins
#define rEncoder 23
#define lEncoder 4
#define NoTicks 350.0

//Encoders variables
unsigned long lastRPMTime = 0;
unsigned long currentRPMTime;
float deltaRPMTime;
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
#define Rightkp 1.4
#define Rightki 0.3
#define Rightkd 0.001

#define Leftkp 1.4
#define Leftki 0.3
#define Leftkd 0.001

//Kinematics
Kinematics kinematics(MOTOR_MAX_RPM, WHEEL_DIAMETER, LR_WHEEL_DISTANCE, PWM_BITS, VelConst, ThetaConst);

//PID
PID RPID = PID(Rightkp, Rightkd, Rightki);
PID LPID = PID(Leftkp, Leftkd, Leftki);

//Position Data
float x = 0;
float y = 0;
float z = PI / 2;
velocities vel, fin;
velocities Force;
output Opwm, rpm, pwm;

void setup() {

  Serial.begin(115200);
  WiFi.begin(ssid, password);

  // Configure motor pins
  ledcAttach(MotorA1, 10000,8);
  ledcAttach(MotorA2, 10000,8);
  ledcAttach(MotorB1, 10000,8);
  ledcAttach(MotorB2, 10000,8);

  // Initialize motors to stop
  //Drive(0, 0);

  // Sets the encoders
  attachInterrupt(digitalPinToInterrupt(rEncoder), Rpulses, RISING);
  attachInterrupt(digitalPinToInterrupt(lEncoder), Lpulses, RISING);

  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected to Wi-Fi");
  Serial.printf("Device IP: %s, UDP port: %d\n", WiFi.localIP().toString().c_str(), localUdpPort);
  
  udp.begin(localUdpPort);  // Start UDP*/
  rpulses = 0;
  lpulses = 0;
  lastRPMTime = millis();
  currentRPMTime = millis();
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
/*void Drive(int MotorL, int MotorR) {
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
}*/

//Funcion actual para usar el puente H
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

//GetRPM
output GetRPM() {
  output temp;
  currentRPMTime = millis();
  noInterrupts();
  if(currentRPMTime - lastRPMTime > 0){
    temp.motor2 = ((float)rpulses / NoTicks) /((currentRPMTime - lastRPMTime) / 1000.0 )* 60;  // number of revolutions * deltaTime(in milis) * 60sec = to get rpm
    temp.motor1 = ((float)lpulses / NoTicks) /((currentRPMTime - lastRPMTime) / 1000.0 )* 60;
  }
  //Debido a que solo tenemos un pin para poder medir el rpm,
  // la forma en la que se determina si es positivo o negativo el rpm
  // es si es dandole el signo del pwm que se dio.
  temp.motor1 *= signbit(pwm.motor1) ? -1 : 1;
  temp.motor2 *= signbit(pwm.motor2) ? -1 : 1;
  lpulses = 0;
  rpulses = 0;
  lastRPMTime = currentRPMTime;
  interrupts();
  return temp;
}


bool inic = true;

void loop() {
  // Mini delay

  if(inic){
    inic = false;
    delay(5000);
  }
  //Codigo para recivir la informacion por parte de vision
      int packetSize = udp.parsePacket();
      currentUDPTime = millis();
      bool deltaUDP = (currentUDPTime - deltaUDP ) > 35;
      if (packetSize && deltaUDP ) {
        // Read the packet into the buffer
        udp.read(packetBuffer, sizeof(packetBuffer));

        // Convert bytes to floats
        memcpy(&x_coord, &packetBuffer[0], sizeof(float));
        memcpy(&y_coord, &packetBuffer[4], sizeof(float));
        x_coord /= -100; //Debido a que el eje x de vision esta invertido, 
                          // se decidio optar por invertir este valor para solucionar el problema
        y_coord /= 100;
        previousUDPTime = currentUDPTime;
      }

  fin._x = x_coord;
  fin._y = y_coord; //Info a recivir
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
  Drive2(pwm.motor1,pwm.motor2);

  


}
