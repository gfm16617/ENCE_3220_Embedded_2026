/**
 * @file esp32_pid_cascaded_v5.ino
 * @brief Cascaded PID controller for two-wheel balancing robot
 * @details Inner loop: Balance control (100 Hz)
 *          Outer loop: Position control (50 Hz)
 */

#include <Wire.h>
#include <TB6612_ESP32.h>
#include <WiFi.h>

/* ============================================================================
 * TB6612 Motor Driver Pin Definitions
 * ============================================================================ */
#define TB6612__AIN1  13
#define TB6612__AIN2  14
#define TB6612__BIN1  12
#define TB6612__BIN2  17
#define TB6612__PWMA  26
#define TB6612__PWMB  25
#define TB6612__STBY  33

/* ============================================================================
 * MPU6050 IMU Definitions
 * ============================================================================ */
#define MPU6050__SDA_PIN   21
#define MPU6050__SCL_PIN   22
#define MPU6050__I2C_ADDR  0x68

/* ============================================================================
 * MT6701 Encoder Definitions
 * ============================================================================ */
#define MT6701__I2C_ADDR    0x06
#define MT6701__ANGLE_REG   0x03
#define MT6701__TICKS_PER_REV  16384.0

/* ============================================================================
 * Wheel Parameters
 * ============================================================================ */
#define WHEEL__DIAMETER_MM  65.0
#define WHEEL__MM_PER_TICK  ((PI * WHEEL__DIAMETER_MM) / MT6701__TICKS_PER_REV)

/* ============================================================================
 * Control Loop Timing
 * ============================================================================ */
#define CONTROL__INNER_LOOP_MS  10
#define CONTROL__OUTER_LOOP_MS  20

/* ============================================================================
 * Safety Limits
 * ============================================================================ */
#define SAFETY__MAX_TILT_ANGLE     20.0
#define SAFETY__MAX_ANGLE_OFFSET   5.0
#define SAFETY__INTEGRAL_LIMIT     50.0

/* ============================================================================
 * Type Definitions
 * ============================================================================ */
typedef struct
{
  int16_t x;
  int16_t y;
  int16_t z;
}Mpu6050_RawData_t;

typedef struct
{
  float x;
  float y;
  float z;
}Mpu6050_ScaledData_t;

typedef struct
{
  float kp;
  float ki;
  float kd;
  float error;
  float prevError;
  float integral;
  float integralLimit;
  float derivativeFiltered;
}Pid_Controller_t;

typedef struct
{
  int16_t  rawAngle;
  int16_t  prevRawAngle;
  int32_t  ticks;
  float    position;
  float    prevPosition;
  float    velocity;
  float    velocityFiltered;
}Mt6701_Encoder_t;

/* ============================================================================
 * Global Variables - Motor Driver
 * ============================================================================ */
static const int gOffsetA = 1;
static const int gOffsetB = 1;

static Motor gMotor1 = Motor(TB6612__AIN1, TB6612__AIN2, TB6612__PWMA, gOffsetA, TB6612__STBY, 5000, 8, 1);
static Motor gMotor2 = Motor(TB6612__BIN1, TB6612__BIN2, TB6612__PWMB, gOffsetB, TB6612__STBY, 5000, 8, 2);

/* ============================================================================
 * Global Variables - MPU6050
 * ============================================================================ */
static Mpu6050_RawData_t    gAccelRaw;
static Mpu6050_RawData_t    gGyroRaw;
static Mpu6050_ScaledData_t gAccelScaled;
static Mpu6050_ScaledData_t gGyroScaled;

static float gTiltAngle      = 0.0f;
static float gComplementaryAlpha = 0.995f;

/* ============================================================================
 * Global Variables - MT6701 Encoder
 * ============================================================================ */
static Mt6701_Encoder_t gEncoder;
static float gVelocityFilterAlpha = 0.8f;

/* ============================================================================
 * Global Variables - Inner Loop (Balance PID)
 * ============================================================================ */
static Pid_Controller_t gBalancePid = {
  .kp = 28.10f,
  .ki = 0.01f,
  .kd = 0.38f,
  .error = 0.0f,
  .prevError = 0.0f,
  .integral = 0.0f,
  .integralLimit = 50.0f,
  .derivativeFiltered = 0.0f
};

static float gSetpointBase  = -14.2f;
static float gAngleOffset   = 0.0f;
static float gSetpoint      = 0.0f;
static float gBalanceOutput = 0.0f;

static unsigned long gLastInnerLoopTime = 0;

/* ============================================================================
 * Global Variables - Outer Loop (Position PID)
 * ============================================================================ */
static Pid_Controller_t gPositionPid = {
  .kp = 0.5f,
  .ki = 0.0f,
  .kd = 2.0f,
  .error = 0.0f,
  .prevError = 0.0f,
  .integral = 0.0f,
  .integralLimit = 50.0f,
  .derivativeFiltered = 0.0f
};

static float gPositionSetpoint = 0.0f;
static unsigned long gLastOuterLoopTime = 0;

/* ============================================================================
 * Global Variables - WiFi
 * ============================================================================ */
static const char* gApSsid     = "2Wheel_Robot";
static const char* gApPassword = "";

static WiFiServer gServer(80);
static WiFiClient gClient;

/* ============================================================================
 * Global Variables - Tuning Steps
 * ============================================================================ */
static float gStepKp    = 0.1f;
static float gStepKd    = 0.01f;
static float gStepKi    = 0.001f;
static float gStepSp    = 0.1f;
static float gStepKpPos = 0.05f;
static float gStepKdPos = 0.1f;
static float gStepKiPos = 0.001f;

/* ============================================================================
 * Private Function Prototypes
 * ============================================================================ */
static void mpu6050_init (void);
static void mpu6050_read (void);
static void mpu6050_convertData (void);
static void mpu6050_updateTiltAngle (float deltaTime);

static void mt6701_init (void);
static void mt6701_read (void);
static void mt6701_updatePosition (void);
static void mt6701_calculateVelocity (float deltaTime);

static float pid_computeBalance (float deltaTime);
static float pid_computePosition (float deltaTime);

static float control_constrainOutput (float value);
static void motor_drive (float speed);

static void wifi_init (void);
static void wifi_handleCommands (void);

/* ============================================================================
 * MPU6050 Functions
 * ============================================================================ */

/**
 * @brief Initialize MPU6050 IMU
 */
static void mpu6050_init (void)
{
  Wire.beginTransmission(MPU6050__I2C_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
}

/**
 * @brief Read raw data from MPU6050
 */
static void mpu6050_read (void)
{
  Wire.beginTransmission(MPU6050__I2C_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050__I2C_ADDR, 14, true);

  gAccelRaw.x = Wire.read() << 8 | Wire.read();
  gAccelRaw.y = Wire.read() << 8 | Wire.read();
  gAccelRaw.z = Wire.read() << 8 | Wire.read();

  //Skip temperature registers
  Wire.read();
  Wire.read();

  gGyroRaw.x = Wire.read() << 8 | Wire.read();
  gGyroRaw.y = Wire.read() << 8 | Wire.read();
  gGyroRaw.z = Wire.read() << 8 | Wire.read();
}

/**
 * @brief Convert raw MPU6050 data to scaled values
 */
static void mpu6050_convertData (void)
{
  gAccelScaled.x = gAccelRaw.x / 16384.0f;
  gAccelScaled.y = gAccelRaw.y / 16384.0f;
  gAccelScaled.z = gAccelRaw.z / 16384.0f;

  gGyroScaled.x = gGyroRaw.x / 131.0f;
  gGyroScaled.y = gGyroRaw.y / 131.0f;
  gGyroScaled.z = gGyroRaw.z / 131.0f;
}

/**
 * @brief Update tilt angle using complementary filter
 * @param deltaTime Time since last update in seconds
 */
static void mpu6050_updateTiltAngle (float deltaTime)
{
  float accAngle = atan2(gAccelScaled.y, gAccelScaled.z) * 180.0f / PI;
  gTiltAngle = gComplementaryAlpha * (gTiltAngle + gGyroScaled.x * deltaTime)
             + (1.0f - gComplementaryAlpha) * accAngle;
}

/* ============================================================================
 * MT6701 Encoder Functions
 * ============================================================================ */

/**
 * @brief Initialize MT6701 encoder
 */
static void mt6701_init (void)
{
  mt6701_read();
  gEncoder.prevRawAngle   = gEncoder.rawAngle;
  gEncoder.ticks          = 0;
  gEncoder.position       = 0.0f;
  gEncoder.prevPosition   = 0.0f;
  gEncoder.velocity       = 0.0f;
  gEncoder.velocityFiltered = 0.0f;
}

/**
 * @brief Read angle from MT6701 encoder
 */
static void mt6701_read (void)
{
  Wire.beginTransmission(MT6701__I2C_ADDR);
  Wire.write(MT6701__ANGLE_REG);
  Wire.endTransmission(false);

  Wire.requestFrom(MT6701__I2C_ADDR, 2, true);

  if (2 <= Wire.available())
  {
    uint8_t highByte = Wire.read();
    uint8_t lowByte  = Wire.read();
    gEncoder.rawAngle = (highByte << 6) | (lowByte >> 2);
  }
}

/**
 * @brief Update encoder position handling wraparound
 */
static void mt6701_updatePosition (void)
{
  int16_t delta = gEncoder.rawAngle - gEncoder.prevRawAngle;

  //Handle wraparound (14-bit: 0-16383)
  if (8192 < delta)
  {
    delta -= 16384;
  }
  else if (delta < -8192)
  {
    delta += 16384;
  }

  gEncoder.ticks += delta;
  gEncoder.prevRawAngle = gEncoder.rawAngle;

  gEncoder.position = (gEncoder.ticks / MT6701__TICKS_PER_REV) * 360.0f;
}

/**
 * @brief Calculate wheel velocity with filtering
 * @param deltaTime Time since last update in seconds
 */
static void mt6701_calculateVelocity (float deltaTime)
{
  float rawVelocity = (gEncoder.position - gEncoder.prevPosition) / deltaTime;
  gEncoder.velocityFiltered = gVelocityFilterAlpha * gEncoder.velocityFiltered
                            + (1.0f - gVelocityFilterAlpha) * rawVelocity;
  gEncoder.velocity = gEncoder.velocityFiltered;
  gEncoder.prevPosition = gEncoder.position;
}

/* ============================================================================
 * PID Control Functions
 * ============================================================================ */

/**
 * @brief Compute balance PID output (inner loop)
 * @param deltaTime Time since last update in seconds
 * @return PID output value
 */
static float pid_computeBalance (float deltaTime)
{
  gSetpoint = gSetpointBase + gAngleOffset;

  gBalancePid.error = gSetpoint - gTiltAngle;

  //Integral term
  gBalancePid.integral += gBalancePid.error * deltaTime;

  //Clamp integral
  if (gBalancePid.integral > gBalancePid.integralLimit)
  {
    gBalancePid.integral = gBalancePid.integralLimit;
  }
  else if (gBalancePid.integral < -gBalancePid.integralLimit)
  {
    gBalancePid.integral = -gBalancePid.integralLimit;
  }

  //Reset integral near zero error
  if (0.3f > abs(gBalancePid.error))
  {
    gBalancePid.integral = 0.0f;
  }

  //Derivative: use gyro directly
  gBalancePid.derivativeFiltered = -gGyroScaled.x;

  //Compute PID output
  float pidOutput = gBalancePid.kp * gBalancePid.error
                  + gBalancePid.ki * gBalancePid.integral
                  + gBalancePid.kd * gBalancePid.derivativeFiltered;

  gBalancePid.prevError = gBalancePid.error;

  return pidOutput;
}

/**
 * @brief Compute position PID output (outer loop)
 * @param deltaTime Time since last update in seconds
 * @return Angle offset for balance setpoint
 */
static float pid_computePosition (float deltaTime)
{
  gPositionPid.error = gPositionSetpoint - gEncoder.position;

  //Integral term
  gPositionPid.integral += gPositionPid.error * deltaTime;

  //Clamp integral
  if (gPositionPid.integral > gPositionPid.integralLimit)
  {
    gPositionPid.integral = gPositionPid.integralLimit;
  }
  else if (gPositionPid.integral < -gPositionPid.integralLimit)
  {
    gPositionPid.integral = -gPositionPid.integralLimit;
  }

  //Derivative: negative velocity (position derivative)
  float positionDerivative = -gEncoder.velocity;

  //Compute PID output
  float pidOutput = gPositionPid.kp * gPositionPid.error
                  + gPositionPid.ki * gPositionPid.integral
                  + gPositionPid.kd * positionDerivative;

  //Clamp output to max angle offset
  if (pidOutput > SAFETY__MAX_ANGLE_OFFSET)
  {
    pidOutput = SAFETY__MAX_ANGLE_OFFSET;
  }
  else if (pidOutput < -SAFETY__MAX_ANGLE_OFFSET)
  {
    pidOutput = -SAFETY__MAX_ANGLE_OFFSET;
  }

  return pidOutput;
}

/* ============================================================================
 * Motor Control Functions
 * ============================================================================ */

/**
 * @brief Constrain output value to valid PWM range
 * @param value Input value to constrain
 * @return Constrained value between -255 and 255
 */
static float control_constrainOutput (float value)
{
  float result = value;

  if (255.0f < result)
  {
    result = 255.0f;
  }
  else if (result < -255.0f)
  {
    result = -255.0f;
  }

  return result;
}

/**
 * @brief Drive both motors at specified speed
 * @param speed Motor speed (-255 to 255)
 */
static void motor_drive (float speed)
{
  int motorSpeed = (int)speed;
  gMotor1.drive(motorSpeed);
  gMotor2.drive(motorSpeed);
}

/* ============================================================================
 * WiFi Functions
 * ============================================================================ */

/**
 * @brief Initialize WiFi access point
 */
static void wifi_init (void)
{
  WiFi.softAP(gApSsid, gApPassword);

  Serial.println("Access Point Started");
  Serial.print("IP: ");
  Serial.println(WiFi.softAPIP());
  Serial.println("=== CASCADED PID CONTROLLER ===");
  Serial.println("WiFi Commands:");
  Serial.println("  Balance: q/a=Kp, w/s=Kd, e/d=Ki, z/x=SP");
  Serial.println("  Position: r/f=Kp, t/g=Kd, y/h=Ki");
  Serial.println("  p=reset position, ?=show all gains");

  gServer.begin();
}

/**
 * @brief Handle incoming WiFi commands for PID tuning
 */
static void wifi_handleCommands (void)
{
  WiFiClient newClient = gServer.available();

  if (newClient)
  {
    if (gClient)
    {
      gClient.stop();
    }
    gClient = newClient;
  }

  if (gClient && gClient.connected() && gClient.available())
  {
    char cmd = gClient.read();

    if (('\n' == cmd) || ('\r' == cmd))
    {
      return;
    }

    switch (cmd)
    {
      //Inner loop: Balance PID
      case 'q':
        gBalancePid.kp += gStepKp;
        break;

      case 'a':
        gBalancePid.kp -= gStepKp;
        break;

      case 'w':
        gBalancePid.kd += gStepKd;
        break;

      case 's':
        gBalancePid.kd -= gStepKd;
        break;

      case 'e':
        gBalancePid.ki += gStepKi;
        break;

      case 'd':
        gBalancePid.ki -= gStepKi;
        break;

      case 'z':
        gSetpointBase += gStepSp;
        break;

      case 'x':
        gSetpointBase -= gStepSp;
        break;

      //Outer loop: Position PID
      case 'r':
        gPositionPid.kp += gStepKpPos;
        break;

      case 'f':
        gPositionPid.kp -= gStepKpPos;
        break;

      case 't':
        gPositionPid.kd += gStepKdPos;
        break;

      case 'g':
        gPositionPid.kd -= gStepKdPos;
        break;

      case 'y':
        gPositionPid.ki += gStepKiPos;
        break;

      case 'h':
        gPositionPid.ki -= gStepKiPos;
        break;

      //Reset encoder position
      case 'p':
      {
        gEncoder.ticks        = 0;
        gEncoder.position     = 0.0f;
        gEncoder.prevPosition = 0.0f;
        gPositionPid.integral = 0.0f;
        break;
      }

      //Print current state
      case '?':
      {
        gClient.println("=== CURRENT GAINS ===");
        gClient.print("Balance: Kp=");
        gClient.print(gBalancePid.kp);
        gClient.print(" Kd=");
        gClient.print(gBalancePid.kd);
        gClient.print(" Ki=");
        gClient.println(gBalancePid.ki);
        gClient.print("Position: Kp=");
        gClient.print(gPositionPid.kp);
        gClient.print(" Kd=");
        gClient.print(gPositionPid.kd);
        gClient.print(" Ki=");
        gClient.println(gPositionPid.ki);
        gClient.print("Base SP: ");
        gClient.println(gSetpointBase);
        return;
      }

      default:
        break;
    }

    //Prevent negative gains
    if (0.0f > gBalancePid.kp)
    {
      gBalancePid.kp = 0.0f;
    }
    if (0.0f > gBalancePid.kd)
    {
      gBalancePid.kd = 0.0f;
    }
    if (0.0f > gBalancePid.ki)
    {
      gBalancePid.ki = 0.0f;
    }
    if (0.0f > gPositionPid.kp)
    {
      gPositionPid.kp = 0.0f;
    }
    if (0.0f > gPositionPid.kd)
    {
      gPositionPid.kd = 0.0f;
    }
    if (0.0f > gPositionPid.ki)
    {
      gPositionPid.ki = 0.0f;
    }

    //Send back current values
    gClient.print("Bal[Kp:");
    gClient.print(gBalancePid.kp);
    gClient.print(" Kd:");
    gClient.print(gBalancePid.kd);
    gClient.print(" Ki:");
    gClient.print(gBalancePid.ki);
    gClient.print("] Pos[Kp:");
    gClient.print(gPositionPid.kp);
    gClient.print(" Kd:");
    gClient.print(gPositionPid.kd);
    gClient.print(" Ki:");
    gClient.print(gPositionPid.ki);
    gClient.print("] SP:");
    gClient.println(gSetpointBase);
  }
}

/* ============================================================================
 * Arduino Setup and Loop
 * ============================================================================ */

/**
 * @brief Arduino setup function
 */
void setup (void)
{
  Serial.begin(115200);

  Wire.begin(MPU6050__SDA_PIN, MPU6050__SCL_PIN);
  mpu6050_init();
  mt6701_init();

  delay(1000);

  unsigned long currentTime = millis();
  gLastInnerLoopTime = currentTime;
  gLastOuterLoopTime = currentTime;

  brake(gMotor1, gMotor2);

  wifi_init();
}

/**
 * @brief Arduino main loop
 */
void loop (void)
{
  wifi_handleCommands();

  unsigned long now = millis();

  /* === OUTER LOOP (Position Control - 50 Hz) === */
  if (now - gLastOuterLoopTime >= CONTROL__OUTER_LOOP_MS)
  {
    float deltaTimeOuter = CONTROL__OUTER_LOOP_MS / 1000.0f;
    gLastOuterLoopTime = now;

    mt6701_read();
    mt6701_updatePosition();
    mt6701_calculateVelocity(deltaTimeOuter);

    gAngleOffset = pid_computePosition(deltaTimeOuter);
  }

  /* === INNER LOOP (Balance Control - 100 Hz) === */
  if (now - gLastInnerLoopTime >= CONTROL__INNER_LOOP_MS)
  {
    float deltaTimeInner = CONTROL__INNER_LOOP_MS / 1000.0f;
    gLastInnerLoopTime = now;

    mpu6050_read();
    mpu6050_convertData();
    mpu6050_updateTiltAngle(deltaTimeInner);

    gBalanceOutput = pid_computeBalance(deltaTimeInner);
    gBalanceOutput = control_constrainOutput(gBalanceOutput);

    //Safety: stop if fallen
    if (SAFETY__MAX_TILT_ANGLE < abs(gTiltAngle - gSetpointBase))
    {
      gMotor1.brake();
      gMotor2.brake();
      gBalancePid.integral  = 0.0f;
      gBalancePid.prevError = 0.0f;
      gPositionPid.integral = 0.0f;
    }
    else
    {
      motor_drive(gBalanceOutput);
    }

    //Debug output
    Serial.print(deltaTimeInner);
    Serial.print(",");
    Serial.print(gTiltAngle);
    Serial.print(",");
    Serial.print(gBalanceOutput);
    Serial.print(",");
    Serial.print(gEncoder.position);
    Serial.print(",");
    Serial.print(gEncoder.velocity);
    Serial.print(",");
    Serial.println(gAngleOffset);
  }
}
