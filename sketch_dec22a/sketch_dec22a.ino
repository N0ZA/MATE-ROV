#include <Wire.h>

#define IMU_ADDR 0x6B

// Gyro registers
#define OUTX_L_G 0x22   // Gyro X (pitch rate)
#define OUTY_L_G 0x24   // Gyro Y (roll rate)

// Accel registers
#define OUTX_L_A 0x28

float gx_bias = 0;
float gy_bias = 0;

float pitch = 0;
float roll  = 0;

unsigned long lastTime = 0;

// ----------------------------------------------------
// Forward declarations
// ----------------------------------------------------
void getOrientation(float &outPitch, float &outRoll, float &outYaw);
String getPosition(float pitch, float roll);

// ----------------------------------------------------
// Read 16-bit signed
// ----------------------------------------------------
int16_t read16(uint8_t reg) {
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);

  Wire.requestFrom((uint8_t)IMU_ADDR, (uint8_t)2);
  uint8_t lo = Wire.read();
  uint8_t hi = Wire.read();
  return (int16_t)((hi << 8) | lo);
}

// ----------------------------------------------------
// Gyro bias calibration
// ----------------------------------------------------
void calibrateGyros() {
  long sumX = 0;
  long sumY = 0;

  Serial.println("Calibrating gyros... keep sensor still");

  for (int i = 0; i < 3000; i++) {
    sumX += read16(OUTX_L_G);
    sumY += read16(OUTY_L_G);
    delay(1);
  }

  gx_bias = (sumX / 3000.0f) * 0.070f;
  gy_bias = (sumY / 3000.0f) * 0.070f;

  Serial.println("Calibration complete");
}

// ----------------------------------------------------
// SETUP
// ----------------------------------------------------
void setup() {
  Serial.begin(115200);
  while (!Serial);

  Wire.setSDA(18);
  Wire.setSCL(19);
  Wire.begin();
  Wire.setClock(400000);

  // Gyro: 104 Hz, ±2000 dps
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(0x11);
  Wire.write(0x50);
  Wire.endTransmission();

  // Accel: 104 Hz, ±4g
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(0x10);
  Wire.write(0x50);
  Wire.endTransmission();

  calibrateGyros();
  lastTime = micros();
}

// ----------------------------------------------------
// LOOP
// ----------------------------------------------------
void loop() {
  float p, r, y;

  getOrientation(p, r, y);
  String pos = getPosition(p, r);

  Serial.print("Pitch: ");
  Serial.print(p);
  Serial.print("   Roll: ");
  Serial.print(r);
  Serial.print("   Yaw: ");
  Serial.print(y);
  Serial.print("   Position: ");
  Serial.println(pos);

  delay(50);
}

// ----------------------------------------------------
// ORIENTATION FUSION (gyro + accel)
// ----------------------------------------------------
void getOrientation(float &outPitch, float &outRoll, float &outYaw) {
  unsigned long now = micros();
  float dt = (now - lastTime) * 1e-6f;
  lastTime = now;

  if (dt <= 0 || dt > 0.1f) return;

  // Gyro rates (deg/sec)
  float gx = read16(OUTX_L_G) * 0.070f - gx_bias;  // pitch rate
  float gy = read16(OUTY_L_G) * 0.070f - gy_bias;  // roll rate

  // Accelerometer (g)
  float ax = read16(OUTX_L_A) * 0.000122f;
  float ay = read16(OUTX_L_A + 2) * 0.000122f;
  float az = read16(OUTX_L_A + 4) * 0.000122f;

  // Accel angles (deg)
  float accelPitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * 57.2958f;
  float accelRoll  = atan2f(ay, az) * 57.2958f;

  // Complementary filter
  const float alpha = 0.98f;

  pitch = alpha * (pitch + gx * dt) + (1.0f - alpha) * accelPitch;
  roll  = alpha * (roll  + gy * dt) + (1.0f - alpha) * accelRoll;

  outPitch = pitch;
  outRoll  = roll;
  outYaw   = 0.0f;  // frozen yaw
}

// ----------------------------------------------------
// HUMAN‑READABLE POSITION CLASSIFIER
// ----------------------------------------------------
String getPosition(float pitch, float roll) {

  const float LEVEL_TILT   = 12.0f;
  const float STRONG_TILT  = 25.0f;
  const float UPSIDE_LIMIT = 120.0f;

  if (fabs(pitch) > UPSIDE_LIMIT || fabs(roll) > UPSIDE_LIMIT)
    return "Upside Down";

  if (pitch > STRONG_TILT  && fabs(roll) < LEVEL_TILT) return "Tilted Forward";
  if (pitch < -STRONG_TILT && fabs(roll) < LEVEL_TILT) return "Tilted Backward";

  if (roll > STRONG_TILT  && fabs(pitch) < LEVEL_TILT) return "Tilted Right";
  if (roll < -STRONG_TILT && fabs(pitch) < LEVEL_TILT) return "Tilted Left";

  if (pitch > STRONG_TILT  && roll > STRONG_TILT)  return "Tilted Forward‑Right";
  if (pitch > STRONG_TILT  && roll < -STRONG_TILT) return "Tilted Forward‑Left";
  if (pitch < -STRONG_TILT && roll > STRONG_TILT)  return "Tilted Backward‑Right";
  if (pitch < -STRONG_TILT && roll < -STRONG_TILT) return "Tilted Backward‑Left";

  return "Flat / Level";
}