#include <Adafruit_ISM330DHCX.h>
#include <MadgwickAHRS.h>
#include <Wire.h>

// Objects
Adafruit_ISM330DHCX ism;
Madgwick filter;

// --- State Variables ---
float velX = 0, velY = 0, velZ = 0;
float roll = 0, pitch = 0, heading = 0;
float offAX = 0, offAY = 0, offAZ = 0;
unsigned long lastUpdate = 0;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  
  // Initialize Serial
  Serial.begin(115200);
  uint32_t startWait = millis();
  while (!Serial && (millis() - startWait < 3000));
  
  Serial.println("--- ROV MISSION COMMAND: START ---");

  // Physical Layer (Teensy 4.1 Optimized)
  Wire.begin();
  Wire.setClock(400000); 
  Wire.setTimeout(3000); // Prevent I2C hardware hangs

  if (!ism.begin_I2C(0x6B)) {
    Serial.println("FATAL: Sensor Not Found");
    while (1) {
      digitalToggle(LED_BUILTIN); // Rapid blink = Hardware Error
      delay(50);
    }
  }

  // Sensor Configuration
  ism.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
  ism.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS);
  filter.begin(100); 

  Serial.println("STATUS: Calibrating Bias... Do not move.");
  calibrateSensors();
  Serial.println("STATUS: System Online.");
  
  lastUpdate = micros();
}

void loop() {
  sensors_event_t accel, gyro, temp;
  
  // Check if sensor is alive
  if (!ism.getEvent(&accel, &gyro, &temp)) return;

  // 1. Precise Delta Time
  unsigned long now = micros();
  float dt = (now - lastUpdate) / 1000000.0;
  lastUpdate = now;
  if (dt <= 0 || dt > 0.1) dt = 0.01;

  // 2. Orientation Fusion (AHRS)
  filter.updateIMU(gyro.gyro.x * 57.2958, gyro.gyro.y * 57.2958, gyro.gyro.z * 57.2958, 
                   accel.acceleration.x, accel.acceleration.y, accel.acceleration.z);

  roll    = filter.getRoll();
  pitch   = filter.getPitch();
  heading = filter.getYaw();

  // 3. Convert to Radians for Math
  float r = roll    * 0.0174533;
  float p = pitch   * 0.0174533;
  float y = heading * 0.0174533;

  // 4. Gravity Compensation (Body Frame)
  // Isolating linear motion from the 9.8m/s^2 constant pull of Earth
  float linAccX_body = (accel.acceleration.x - offAX) - (sin(p) * 9.80665);
  float linAccY_body = (accel.acceleration.y - offAY) - (-sin(r) * cos(p) * 9.80665);
  float linAccZ_body = (accel.acceleration.z - offAZ) - (cos(r) * cos(p) * 9.80665);

  // 5. World Coordinate Transformation (Yaw Rotation)
  // This ensures velX and velY stay fixed to the world, even if the ROV turns
  float worldAccX = linAccX_body * cos(y) - linAccY_body * sin(y);
  float worldAccY = linAccX_body * sin(y) + linAccY_body * cos(y);

  // 6. Deadzone Filtering (Noise Gate)
  if (abs(worldAccX) < 0.2) worldAccX = 0;
  if (abs(worldAccY) < 0.2) worldAccY = 0;
  if (abs(linAccZ_body) < 0.2) linAccZ_body = 0;

  // 7. Velocity Integration & Drag Simulation
  // Multiplying by 0.98 simulates water resistance/damping
  velX = (velX + worldAccX * dt) * 0.98;
  velY = (velY + worldAccY * dt) * 0.98;
  velZ = (velZ + linAccZ_body * dt) * 0.98;

  // 8. Serial Output (USB Buffer Protected)
  if (Serial.availableForWrite() > 100) {
    Serial.printf("H:%.1f,R:%.1f,P:%.1f,VX:%.3f,VY:%.3f,VZ:%.3f\n", 
                  heading, roll, pitch, velX, velY, velZ);
  }

  // Heartbeat LED
  static uint32_t ledT = 0;
  if (millis() - ledT > 500) {
    digitalToggle(LED_BUILTIN);
    ledT = millis();
  }

  delay(10); // Maintain ~100Hz frequency
}

void calibrateSensors() {
  float sx = 0, sy = 0, sz = 0;
  const int samples = 500;
  for (int i = 0; i < samples; i++) {
    sensors_event_t a, g, t;
    ism.getEvent(&a, &g, &t);
    sx += a.acceleration.x; 
    sy += a.acceleration.y; 
    sz += a.acceleration.z;
    delay(2);
  }
  offAX = sx / samples;
  offAY = sy / samples;
  offAZ = (sz / samples) - 9.80665; // Expected gravity on Z is 9.8
}