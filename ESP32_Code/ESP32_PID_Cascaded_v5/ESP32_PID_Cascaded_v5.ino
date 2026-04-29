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

Motor motor1 = Motor(AIN1, AIN2, PWMA, offsetA, STBY, 5000, 8, 1);
Motor motor2 = Motor(BIN1, BIN2, PWMB, offsetB, STBY, 5000, 8, 2);
//--------------------------------------------------//

//---------------- MPU6050 -------------//
#define SDA_PIN 21
#define SCL_PIN 22
#define MPU_ADDR 0x68

int16_t ax, ay, az;
int16_t gx, gy, gz;

float accX, accY, accZ;
float gyroX, gyroY, gyroZ;

float angle = 0;
float alpha = 0.995;

unsigned long lastTime = 0;
//-----------------------------//

//---------------- MT6701 ENCODER -------------//
#define MT6701_ADDR 0x06
#define MT6701_ANGLE_REG 0x03

int16_t rawAngle = 0;
int16_t prevRawAngle = 0;
int32_t encoderTicks = 0;      // Cumulative ticks (handles wraparound)
float wheelPosition = 0;       // Position in degrees
float wheelVelocity = 0;       // Velocity in deg/s
float prevWheelPosition = 0;

// Encoder filtering
float velocityFiltered = 0;
float alpha_vel = 0.8;         // Velocity low-pass filter

// Wheel parameters (adjust to your setup)
const float TICKS_PER_REV = 16384.0;  // 14-bit encoder
const float WHEEL_DIAMETER_MM = 65.0; // Your wheel diameter
const float MM_PER_TICK = (PI * WHEEL_DIAMETER_MM) / TICKS_PER_REV;
//--------------------------------------------//

//---------------- INNER LOOP: BALANCE PID -------------//
float Kp = 28.10;
float Kd = 0.38;
float Ki = 0.01;

float error = 0;
float prevError = 0;
float integral = 0;

float output = 0;

// Base setpoint (mechanical balance point)
float setpointBase = -14.2;

// Dynamic setpoint = setpointBase + angleOffset (from outer loop)
float angleOffset = 0;
float setpoint = 0;

float maxAngle = 20.0;

const unsigned long loopTime = 10; // 10ms (100 Hz)
unsigned long lastLoopTime = 0;

float derivativeFiltered = 0;
//-----------------------------------------//

//---------------- OUTER LOOP: POSITION PID -------------//
// Runs at slower rate (50 Hz)
const unsigned long outerLoopTime = 20; // 20ms
unsigned long lastOuterLoopTime = 0;

// Position PID gains
float Kp_pos = 0.5;    // Position proportional
float Ki_pos = 0.0;    // Position integral (usually small or zero)
float Kd_pos = 2.0;    // Position derivative (velocity damping)

float positionSetpoint = 0;  // Target position (0 = stay in place)
float positionError = 0;
float positionIntegral = 0;
float prevPositionError = 0;

// Output limits (max lean angle in degrees)
float maxAngleOffset = 5.0;

// Position integral windup limit
float positionIntegralLimit = 50.0;
//-------------------------------------------------------//

// -------- WIFI --------
const char* ap_ssid = "2Wheel_Robot";
const char* ap_password = "";

WiFiServer server(80);
WiFiClient client;

// -------- TUNING STEPS --------
// Inner loop steps
float stepKp = 0.1;
float stepKd = 0.01;
float stepKi = 0.001;
float stepSP = 0.1;

// Outer loop steps
float stepKp_pos = 0.05;
float stepKd_pos = 0.1;
float stepKi_pos = 0.001;

// -------------------- INIT MPU6050 --------------------
void initMPU() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); // PWR_MGMT_1
  Wire.write(0);    // Wake up MPU6050
  Wire.endTransmission(true);
}

// -------------------- INIT MT6701 --------------------
void initMT6701() {
  // MT6701 typically doesn't need initialization
  // Just read initial position
  readMT6701();
  prevRawAngle = rawAngle;
  encoderTicks = 0;
  wheelPosition = 0;
  prevWheelPosition = 0;
}

// -------------------- READ MPU6050 --------------------
void readMPU() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
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

// -------------------- READ MT6701 --------------------
void readMT6701() {
  Wire.beginTransmission(MT6701_ADDR);
  Wire.write(MT6701_ANGLE_REG);
  Wire.endTransmission(false);

  Wire.requestFrom(MT6701_ADDR, 2, true);

  if (Wire.available() >= 2) {
    uint8_t highByte = Wire.read();
    uint8_t lowByte = Wire.read();
    // MT6701: 14-bit angle, upper 6 bits of lowByte are valid
    rawAngle = (highByte << 6) | (lowByte >> 2);
  }
}

// -------------------- UPDATE ENCODER POSITION --------------------
void updateEncoderPosition() {
  int16_t delta = rawAngle - prevRawAngle;

  // Handle wraparound (14-bit: 0-16383)
  if (delta > 8192) {
    delta -= 16384;  // Wrapped backwards
  } else if (delta < -8192) {
    delta += 16384;  // Wrapped forwards
  }

  encoderTicks += delta;
  prevRawAngle = rawAngle;

  // Convert to degrees
  wheelPosition = (encoderTicks / TICKS_PER_REV) * 360.0;
}

// -------------------- CALCULATE VELOCITY --------------------
void calculateVelocity(float dt) {
  float rawVelocity = (wheelPosition - prevWheelPosition) / dt;
  velocityFiltered = alpha_vel * velocityFiltered + (1 - alpha_vel) * rawVelocity;
  wheelVelocity = velocityFiltered;
  prevWheelPosition = wheelPosition;
}

// -------------------- CONVERT MPU DATA --------------------
void convertData() {
  accX = ax / 16384.0;
  accY = ay / 16384.0;
  accZ = az / 16384.0;

  gyroX = gx / 131.0;
  gyroY = gy / 131.0;
  gyroZ = gz / 131.0;
}

// -------------------- UPDATE TILT ANGLE --------------------
void updateAngle(float dt) {
  float accAngle = atan2(accY, accZ) * 180.0 / PI;
  angle = alpha * (angle + gyroX * dt) + (1 - alpha) * accAngle;
}

// -------------------- OUTER LOOP: POSITION PID --------------------
float computePositionPID(float dt) {
  positionError = positionSetpoint - wheelPosition;

  // Integral
  positionIntegral += positionError * dt;

  // Clamp integral
  if (positionIntegral > positionIntegralLimit) positionIntegral = positionIntegralLimit;
  if (positionIntegral < -positionIntegralLimit) positionIntegral = -positionIntegralLimit;

  // Derivative (use velocity directly - already filtered)
  // Note: derivative of position error = -velocity (since setpoint is constant)
  float positionDerivative = -wheelVelocity;

  // PID output -> desired angle offset
  float u = Kp_pos * positionError
          + Ki_pos * positionIntegral
          + Kd_pos * positionDerivative;

  // Clamp output to max angle offset
  if (u > maxAngleOffset) u = maxAngleOffset;
  if (u < -maxAngleOffset) u = -maxAngleOffset;

  return u;
}

// -------------------- INNER LOOP: BALANCE PID --------------------
float computeBalancePID(float dt) {
  // Dynamic setpoint from outer loop
  setpoint = setpointBase + angleOffset;

  error = setpoint - angle;

  // Integral
  integral += error * dt;

  // Clamp integral
  if (integral > 50) integral = 50;
  if (integral < -50) integral = -50;

  // Reset integral near zero error
  if (abs(error) < 0.3) {
    integral = 0;
  }

  // Derivative: use gyro directly
  derivativeFiltered = -gyroX;

  // PID output
  float u = Kp * error + Ki * integral + Kd * derivativeFiltered;

  prevError = error;

  return u;
}

// -------------------- CONSTRAIN OUTPUT --------------------
float constrainOutput(float u) {
  if (u > 255) return 255;
  if (u < -255) return -255;
  return u;
}

// -------------------- DRIVE MOTORS --------------------
void driveMotors(float u) {
  int speed = (int)u;
  motor1.drive(speed);
  motor2.drive(speed);
}

// -------------------- WIFI HANDLER --------------------
void handleWiFi() {
  WiFiClient newClient = server.available();

  if (newClient) {
    if (client) client.stop();
    client = newClient;
  }

  if (client && client.connected() && client.available()) {
    char cmd = client.read();

    if (cmd == '\n' || cmd == '\r') return;

    switch (cmd) {
      // Inner loop: Balance PID
      case 'q': Kp += stepKp; break;
      case 'a': Kp -= stepKp; break;

      case 'w': Kd += stepKd; break;
      case 's': Kd -= stepKd; break;

      case 'e': Ki += stepKi; break;
      case 'd': Ki -= stepKi; break;

      case 'z': setpointBase += stepSP; break;
      case 'x': setpointBase -= stepSP; break;

      // Outer loop: Position PID
      case 'r': Kp_pos += stepKp_pos; break;
      case 'f': Kp_pos -= stepKp_pos; break;

      case 't': Kd_pos += stepKd_pos; break;
      case 'g': Kd_pos -= stepKd_pos; break;

      case 'y': Ki_pos += stepKi_pos; break;
      case 'h': Ki_pos -= stepKi_pos; break;

      // Reset encoder position (useful for testing)
      case 'p':
        encoderTicks = 0;
        wheelPosition = 0;
        prevWheelPosition = 0;
        positionIntegral = 0;
        break;

      // Print current state
      case '?':
        client.println("=== CURRENT GAINS ===");
        client.print("Balance: Kp="); client.print(Kp);
        client.print(" Kd="); client.print(Kd);
        client.print(" Ki="); client.println(Ki);
        client.print("Position: Kp="); client.print(Kp_pos);
        client.print(" Kd="); client.print(Kd_pos);
        client.print(" Ki="); client.println(Ki_pos);
        client.print("Base SP: "); client.println(setpointBase);
        return;
    }

    // Prevent negative gains
    if (Kp < 0) Kp = 0;
    if (Kd < 0) Kd = 0;
    if (Ki < 0) Ki = 0;
    if (Kp_pos < 0) Kp_pos = 0;
    if (Kd_pos < 0) Kd_pos = 0;
    if (Ki_pos < 0) Ki_pos = 0;

    // Send back current values
    client.print("Bal[Kp:"); client.print(Kp);
    client.print(" Kd:"); client.print(Kd);
    client.print(" Ki:"); client.print(Ki);
    client.print("] Pos[Kp:"); client.print(Kp_pos);
    client.print(" Kd:"); client.print(Kd_pos);
    client.print(" Ki:"); client.print(Ki_pos);
    client.print("] SP:"); client.println(setpointBase);
  }
}

// -------------------- SETUP --------------------
void setup() {
  Serial.begin(115200);

  Wire.begin(SDA_PIN, SCL_PIN);
  initMPU();
  initMT6701();

  delay(1000);
  lastTime = millis();
  lastLoopTime = millis();
  lastOuterLoopTime = millis();

  brake(motor1, motor2);

  // WiFi Init
  WiFi.softAP(ap_ssid, ap_password);

  Serial.println("Access Point Started");
  Serial.print("IP: ");
  Serial.println(WiFi.softAPIP());
  Serial.println("=== CASCADED PID CONTROLLER ===");
  Serial.println("WiFi Commands:");
  Serial.println("  Balance: q/a=Kp, w/s=Kd, e/d=Ki, z/x=SP");
  Serial.println("  Position: r/f=Kp, t/g=Kd, y/h=Ki");
  Serial.println("  p=reset position, ?=show all gains");

  server.begin();
}

// -------------------- LOOP --------------------
void loop() {

  handleWiFi();

  unsigned long now = millis();

  // === OUTER LOOP (Position Control - 50 Hz) ===
  if (now - lastOuterLoopTime >= outerLoopTime) {
    float dt_outer = outerLoopTime / 1000.0;
    lastOuterLoopTime = now;

    // Read encoder and update position/velocity
    readMT6701();
    updateEncoderPosition();
    calculateVelocity(dt_outer);

    // Compute position PID -> outputs angle offset
    angleOffset = computePositionPID(dt_outer);
  }

  // === INNER LOOP (Balance Control - 100 Hz) ===
  if (now - lastLoopTime >= loopTime) {
    float dt_inner = loopTime / 1000.0;
    lastLoopTime = now;

    // Read IMU and update tilt angle
    readMPU();
    convertData();
    updateAngle(dt_inner);

    // Compute balance PID (uses angleOffset from outer loop)
    output = computeBalancePID(dt_inner);
    output = constrainOutput(output);

    // Safety: stop if fallen
    if (abs(angle - setpointBase) > maxAngle) {
      motor1.brake();
      motor2.brake();
      integral = 0;
      prevError = 0;
      positionIntegral = 0;
    } else {
      driveMotors(output);
    }

    // Debug output: time, tilt, output, wheel_pos, wheel_vel, angle_offset
    Serial.print(dt_inner);
    Serial.print(",");
    Serial.print(angle);
    Serial.print(",");
    Serial.print(output);
    Serial.print(",");
    Serial.print(wheelPosition);
    Serial.print(",");
    Serial.print(wheelVelocity);
    Serial.print(",");
    Serial.println(angleOffset);
  }
}
