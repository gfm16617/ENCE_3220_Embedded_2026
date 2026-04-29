#include <Wire.h>
#include <TB6612_ESP32.h>
#include <WiFi.h>

//---------------- TB6612 MOTOR DRIVER -------------//
#define AIN1 13 // ESP32 Pin D13 to TB6612FNG Pin AIN1
#define BIN1 12 // ESP32 Pin D12 to TB6612FNG Pin BIN1
#define AIN2 14 // ESP32 Pin D14 to TB6612FNG Pin AIN2
#define BIN2 17 // ESP32 Pin D27 to TB6612FNG Pin BIN2
#define PWMA 26 // ESP32 Pin D26 to TB6612FNG Pin PWMA
#define PWMB 25 // ESP32 Pin D25 to TB6612FNG Pin PWMB
#define STBY 33 // ESP32 Pin D33 to TB6612FNG Pin STBY

const int offsetA = 1;
const int offsetB = 1;

Motor motor1 = Motor(AIN1, AIN2, PWMA, offsetA, STBY,5000 ,8,1 );
Motor motor2 = Motor(BIN1, BIN2, PWMB, offsetB, STBY,5000 ,8,2 );
//--------------------------------------------------//

//---------------- MPU6050 -------------//
#define SDA_PIN 21
#define SCL_PIN 22
#define MPU_ADDR 0x68

// Raw data
int16_t ax, ay, az;
int16_t gx, gy, gz;

// Converted data
float accX, accY, accZ;
float gyroX, gyroY, gyroZ;

// Filter
float angle = 0;
float alpha = 0.995;

unsigned long lastTime = 0;
//-----------------------------//

//---------------- PID VARIABLES -------------//
// PID gains (start small)
float Kp = 28.10; 
float Kd = 0.38;
float Ki = 0.01;

// PID state
float error = 0;
float prevError = 0;
float integral = 0;
float derivative = 0;

float output = 0;

// Setpoint (upright)
float setpoint = -14.2; // -12.5

float maxAngle = 20.0;  // degrees

const unsigned long loopTime = 10; // 10ms (100 Hz)
unsigned long lastLoopTime = 0;

float derivativeFiltered = 0;
float alpha_d = 0.9;  // 0.6–0.9 works well

float velocityEstimate = 0;

//-----------------------------------------//

// -------- WIFI --------
const char* ap_ssid = "2Wheel_Robot";
const char* ap_password = "";

WiFiServer server(80);
WiFiClient client;

// -------- TUNING STEPS --------
float stepKp = 0.1;
float stepKd = 0.01;
float stepKi = 0.001;
float stepSP = 0.1;

// -------------------- INIT --------------------
void initMPU() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); // PWR_MGMT_1
  Wire.write(0);    // Wake up MPU6050
  Wire.endTransmission(true);
}

// -------------------- READ --------------------
void readMPU() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); // starting register
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true);

  ax = Wire.read() << 8 | Wire.read();
  ay = Wire.read() << 8 | Wire.read();
  az = Wire.read() << 8 | Wire.read();

  Wire.read(); Wire.read(); // skip temperature

  gx = Wire.read() << 8 | Wire.read();
  gy = Wire.read() << 8 | Wire.read();
  gz = Wire.read() << 8 | Wire.read();
}

// -------------------- CONVERT --------------------
void convertData() {
  accX = ax / 16384.0;
  accY = ay / 16384.0;
  accZ = az / 16384.0;

  gyroX = gx / 131.0;
  gyroY = gy / 131.0;
  gyroZ = gz / 131.0;
}

// -------------------- FILTER --------------------
void updateAngle(float dt) {
  float accAngle = atan2(accY, accZ) * 180.0 / PI;
  angle = alpha * (angle + gyroX * dt) + (1 - alpha) * accAngle;
  //angle = accAngle;
}

// -------------------- PID FUNCTIONS --------------------//
float computePID(float angle, float dt) {
  error = setpoint - angle;

  // Integral
  integral += error * dt;

  // Clamp integral
  if (integral > 50) integral = 50;
  if (integral < -50) integral = -50;

  // Reset integral
  if (abs(error) < 0.3) {
    integral = 0;
  }

  // Derivative
  //derivative = (error - prevError) / dt;

  //float rawDerivative = (error - prevError) / dt;
  //derivativeFiltered = alpha_d * derivativeFiltered + (1 - alpha_d) * rawDerivative;
  
  // Use gyro directly (already deg/s)
  //derivativeFiltered = alpha_d * derivativeFiltered + (1 - alpha_d) * (-gyroX);
  derivativeFiltered = -gyroX;
  //derivativeFiltered = 0.8 * derivativeFiltered + 0.2 * (-gyroX);

  // This is here to see if it works instead of using velocity from encoders
  velocityEstimate = 0.9 * velocityEstimate + 0.1 * output;

  // PID output
  //float u = Kp * error + Ki * integral + Kd * derivative;
  //float u = Kp * error + Ki * integral + Kd * derivativeFiltered; // main output

  float u = Kp * error 
        + Ki * integral 
        + Kd * derivativeFiltered
        - 0.03 * velocityEstimate;  // test - try to substitute velocity from encoders

  prevError = error;

  return u;
}

float constrainOutput(float u) {
  if (u > 255) return 255;
  if (u < -255) return -255;
  return u;
}


void driveMotors(float u) {
  int speed = (int)u;

  // Deadband (optional but useful)
  //if (abs(speed) < 2) speed = 0;

  motor1.drive(speed);
  motor2.drive(speed);
}

/*
void driveMotors(float u) {
  float scaled = u;

  // Smooth nonlinear boost
  if (abs(u) < 50) {
    scaled = u * 3.0;
  }

  if (scaled > 255) scaled = 255;
  if (scaled < -255) scaled = -255;

  motor1.drive((int)scaled);
  motor2.drive((int)scaled);
}
*/

// ----------------------------------------//

// ----------------- WIFI -----------------------//
void handleWiFi() {
  WiFiClient newClient = server.available();

  if (newClient) {
  if (client) client.stop();  // close previous client
    client = newClient;
  }

  if (client && client.connected() && client.available()) {
    char cmd = client.read();

    // Ignore newline / carriage return
    if (cmd == '\n' || cmd == '\r') return;

    switch (cmd) {
      case 'q': Kp += stepKp; break;
      case 'a': Kp -= stepKp; break;

      case 'w': Kd += stepKd; break;
      case 's': Kd -= stepKd; break;

      case 'e': Ki += stepKi; break;
      case 'd': Ki -= stepKi; break;

      case 'z': setpoint += stepSP; break;
      case 'x': setpoint -= stepSP; break;
    }

    // Prevent negative gains
    if (Kp < 0) Kp = 0;
    if (Kd < 0) Kd = 0;
    if (Ki < 0) Ki = 0;

    // Send back values
    client.print("Kp: "); client.print(Kp);
    client.print(" | Kd: "); client.print(Kd);
    client.print(" | Ki: "); client.print(Ki);
    client.print(" | SP: "); client.println(setpoint);

    //Serial.print("Kp: "); Serial.print(Kp);
    //Serial.print(" | Kd: "); Serial.print(Kd);
    //Serial.print(" | Ki: "); Serial.print(Ki);
    //Serial.print(" | SP: "); Serial.println(setpoint);
  }
}
// ----------------------------------------//

// -------------------- SETUP --------------------
void setup() {
  Serial.begin(115200);

  Wire.begin(SDA_PIN, SCL_PIN);
  initMPU();

  delay(1000);
  lastTime = millis();

  // Brake
  brake(motor1, motor2);     // Stop Motor 1 and Motor 2 for 2 seconds 
  //delay(2000);

  // WiFi Init
  WiFi.softAP(ap_ssid, ap_password);

  Serial.println("Access Point Started");
  Serial.print("IP: ");
  Serial.println(WiFi.softAPIP());

  server.begin();
}

// -------------------- LOOP --------------------
void loop() {

  handleWiFi();

  unsigned long now = millis();
  
  if (now - lastLoopTime >= loopTime) {

    //unsigned long start = micros(); // for code timing test only

    lastLoopTime = now;

    float dt_sec = loopTime / 1000.0;

    readMPU();
    convertData();
    updateAngle(dt_sec);

    output = computePID(angle, dt_sec);
    output = constrainOutput(output);

    if (abs(angle - setpoint) > maxAngle) {
      motor1.brake();
      motor2.brake();
      integral = 0;
      prevError = 0;
    } else {
      driveMotors(output);
    }

    Serial.print(dt_sec);
    Serial.print(",");
    Serial.print(angle);
    Serial.print(",");
    Serial.println(output);

    /*
    // for code timing test only
    unsigned long elapsed = micros() - start; 
    Serial.print(",");
    Serial.println(elapsed);
    */
  }

}