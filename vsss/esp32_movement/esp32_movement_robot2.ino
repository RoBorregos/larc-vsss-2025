
  #include <WiFi.h>
  #include "PID.hpp"

  template<typename T1, typename T2>
  struct pair {
    T1 first;
    T2 second;
  };


  // Replace with your server/router ,etc 
  const char* ssid = "vsss_r";
  const char* password = "vsss1234";

  // TCP Server
  WiFiServer server(8081);
  WiFiClient client;

  //MotorPins
  #define MotorA1 19
  #define MotorA2 17  
  #define MotorA_PWM_PIN 1  

  #define MotorB1 2
  #define MotorB2 21
  #define MotorB_PWM_PIN 0  

  //TimeControl Variables
  unsigned long previousTime;
  unsigned long currentTime;
  float deltaTime;

  //Encoder Pins
  #define rEncoder 23
  #define lEncoder 16
  #define NoTicks 350.0

  //Encoder Values
  volatile unsigned int lpulses;
  volatile unsigned int  rpulses;

  // PWM Properties
  const int PWM_FREQUENCY = 5000; // 5 kHz
  const int PWM_RESOLUTION = 8; // 8-bit resolution (0-255)
  const int MAX_PWM_VALUE = 255;

  //Robot ObjVel
  pair<float,float> objective_vel; //Left is first and right is second item in the pair
  pair<float,float> actual_vel;
  pair<int, int> actual_pwm;
  pair<PID,PID> velocity_PID;


  void drive(int speedL, int speedR) {
    // Constrain speeds to the valid PWM range
    speedL = constrain(speedL, -MAX_PWM_VALUE, MAX_PWM_VALUE);
    speedR = constrain(speedR, -MAX_PWM_VALUE, MAX_PWM_VALUE);

    // Left Motor Control
    if (speedL > 0) { // Forward
      digitalWrite(MotorA1, HIGH);
      digitalWrite(MotorA2, LOW);
    } else if (speedL < 0) { // Reverse
      digitalWrite(MotorA1, LOW);
      digitalWrite(MotorA2, HIGH);
    } else { // Stop
      digitalWrite(MotorA1, LOW);
      digitalWrite(MotorA2, LOW);
    }
    ledcWrite(MotorA_PWM_PIN, abs(speedL));
    
    // Right Motor Control
    if (speedR > 0) { 
      digitalWrite(MotorB1, HIGH);
      digitalWrite(MotorB2, LOW);
    } else if (speedR < 0) { 
      digitalWrite(MotorB1, LOW);
      digitalWrite(MotorB2, HIGH);
    } else { 
      digitalWrite(MotorB1, LOW);
      digitalWrite(MotorB2, LOW);
    }
    ledcWrite(MotorB_PWM_PIN, abs(speedR));
  }

  void Rpulses() {
    rpulses = rpulses +1;
  }
  void Lpulses() {
    lpulses = lpulses +1;
  }

  void PrintValues(){
    Serial.print("Objective Vel: "); 
    Serial.print(objective_vel.first);  // Left value
    Serial.print(", ");
    Serial.print(objective_vel.second); // Right value

    Serial.print(" | Actual Vel: ");
    Serial.print(actual_vel.first);
    Serial.print(", ");
    Serial.print(actual_vel.second);

    Serial.print(" | Actual PWM: ");
    Serial.print(actual_pwm.first);
    Serial.print(", ");
    Serial.println(actual_pwm.second); // println for the last value
    delay(10);
  }

  void setup() {
    Serial.begin(115200);

    pinMode(MotorA1, OUTPUT);
    pinMode(MotorA2, OUTPUT);
    pinMode(MotorB1, OUTPUT);
    pinMode(MotorB2, OUTPUT);

    // Setup PWM (updated for core 3.x+)
    ledcAttach(MotorA_PWM_PIN, PWM_FREQUENCY, PWM_RESOLUTION);
    ledcAttach(MotorB_PWM_PIN, PWM_FREQUENCY, PWM_RESOLUTION);

    // Set up the encoders
    pinMode(rEncoder, INPUT_PULLUP);
    pinMode(lEncoder, INPUT_PULLUP);
    attachInterrupt(rEncoder, Rpulses, RISING);
    attachInterrupt(lEncoder, Lpulses, RISING);
    rpulses = 0;
    lpulses = 0;

    //Setup timer
    previousTime = micros();
    currentTime = micros();

    // Connect to WiFi
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(ssid);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
    }
    Serial.println("\nWiFi connected");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());

    // Start server
    server.begin();
    Serial.println("Server started. Waiting for client...");
  }

  void tcp_recieve_RPM() {
    if (client.available() >= 8) { // Wait for 2 floats (8 bytes)
      byte buffer[8];
      client.read(buffer, 8);

      
      memcpy(&objective_vel.first, buffer, 4);
      memcpy(&objective_vel.second, buffer + 4, 4);

      Serial.print("Received RPM L: ");
      Serial.print(objective_vel.first);
      Serial.print(", RPM R: ");
      Serial.println(objective_vel.second);

    }
  }

  
  void encoder_receive_RPM(){
    currentTime = micros();
    noInterrupts();
    if(currentTime - previousTime > 0){
      actual_vel.first = ((float)lpulses / NoTicks) /((currentTime - previousTime) / 1000000.0 )* 60;
      actual_vel.second = ((float)rpulses / NoTicks) /((currentTime - previousTime) / 1000000.0 )* 60;  // number of revolutions * deltaTime(in milis) * 60sec = to get rpm
      lpulses = 0;
      rpulses = 0;
      previousTime = currentTime;
      //Because we only have one pin to measure the rpm,
      // the way to determine if the rpm is positive or negative
      // is by giving it the sign of the given pwm.
      actual_vel.first *= signbit(actual_pwm.first) ? -1 : 1;
      actual_vel.second *= signbit(actual_pwm.second) ? -1 : 1;
    }
    interrupts();
  }

  void VelocityTracker()
  {
    //Basic PID class where the error is considered as the difference
    //between the current PWM of the wheels and the target PWM
    //to then be added to the current PWM limiting the pwm range to -255 < pwm < 255
  float Lcorr = velocity_PID.first.GetCorrection(objective_vel.first - actual_vel.first);  
  float Rcorr = velocity_PID.second.GetCorrection(objective_vel.second - actual_vel.second);
  actual_pwm.first += Lcorr;
  actual_pwm.second += Rcorr;
  // Clamp PWM smoothly
  actual_pwm.first = constrain(actual_pwm.first, -254, 254);
  actual_pwm.second = constrain(actual_pwm.second, -254, 254);

      //Print info to debug
      PrintValues();
      drive(actual_pwm.first,actual_pwm.second);
  }





  void loop() {
    if (!client.connected()) {
      client = server.available();
      if (client) {
        Serial.println("Client connected!");
      }
      return; // No client, so nothing to do
    }

    tcp_recieve_RPM();      //Recieve data from tcp
    encoder_receive_RPM();  //Read and interpret the data from the encoders
    VelocityTracker();      //Use both prev values and a PID to manage motors

    // Handle client disconnection
    if (!client.connected()) {
      Serial.println("Client disconnected.");
      //Segun yo nunca va a llegar a este punto, sino que simplemente iba a llegar al otro case de desconexion de arriba
      
    }
  }
