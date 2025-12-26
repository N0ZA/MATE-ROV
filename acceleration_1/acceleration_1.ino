#include <Adafruit_ISM330DHCX.h>
#include <MadgwickAHRS.h>
#include <Wire.h>

Adafruit_ISM330DHCX ism;
Madgwick filter;

// Velocity State
float velX = 0, velY = 0;
float posX = 0, posY = 0; // Displacement (optional)

// Calibration Offsets
float offAX = 0, offAY = 0, offAZ = 0;

unsigned long lastUpdate = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 5000);

  Wire.begin();
  Wire.setClock(400000);

  // Initialize ISM330DHCX at your scanned address 0x6B
  if (!ism.begin_I2C(0x6B)) {
    Serial.println("ISM330DHCX not found at 0x6B!");
    while (1) yield();
  }

  // Set sensor ranges for ROV movement
  ism.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
  ism.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS);

  filter.begin(100); // 100 Hz filter
  
  Serial.println("Calibrating... Keep ROV perfectly still.");
  calibrateSensors();
  
  lastUpdate = micros();
}

void loop() {
  sensors_event_t accel, gyro, temp;
  ism.getEvent(&accel, &gyro, &temp);

  // 1. Calculate Delta Time
  unsigned long now = micros();
  float dt = (now - lastUpdate) / 1000000.0;
  lastUpdate = now;
  if (dt <= 0 || dt > 0.1) dt = 0.01;

  // 2. Update Orientation (6-DOF)
  // Converting Gyro Rad/s to Deg/s for Madgwick
  filter.updateIMU(gyro.gyro.x * 57.2958, gyro.gyro.y * 57.2958, gyro.gyro.z * 57.2958, 
                   accel.acceleration.x, accel.acceleration.y, accel.acceleration.z);

  // 3. Get Pitch and Roll (in Radians)
  float roll  = filter.getRoll()  * 0.0174533;
  float pitch = filter.getPitch() * 0.0174533;

  // 4. Gravity Compensation
  // We remove the static 9.81 m/s^2 based on current tilt
  float linAccX = (accel.acceleration.x - offAX) - (sin(pitch) * 9.806);
  float linAccY = (accel.acceleration.y - offAY) - (-sin(roll) * cos(pitch) * 9.806);

  // 5. Deadzone & Noise Filter
  // If acceleration is tiny, assume it's sensor noise or vibration
  if (abs(linAccX) < 0.25) linAccX = 0;
  if (abs(linAccY) < 0.25) linAccY = 0;

  // 6. Integrate Acceleration into Velocity (v = v + a*t)
  velX += linAccX * dt;
  velY += linAccY * dt;

  // 7. Friction/Drag Simulation (THE MOST IMPORTANT STEP)
  // Without this, velocity drifts to infinity. 
  // 0.98 simulates water slowing the ROV down.
  velX *= 0.98;
  velY *= 0.98;

  // 8. Output to Serial Plotter
  // Format: Name:Value for easy reading in Arduino Plotter
  Serial.print("AccX_m/s2:"); Serial.print(linAccX);
  Serial.print(",AccY_m/s2:"); Serial.print(linAccY);
  Serial.print(",VelX_m/s:"); Serial.print(velX);
  Serial.print(",VelY_m/s:"); Serial.println(velY);

  delay(10); // Run at approx 100Hz
}

void calibrateSensors() {
  float sumX = 0, sumY = 0, sumZ = 0;
  int samples = 200;
  for (int i = 0; i < samples; i++) {
    sensors_event_t accel, gyro, temp;
    ism.getEvent(&accel, &gyro, &temp);
    sumX += accel.acceleration.x;
    sumY += accel.acceleration.y;
    sumZ += accel.acceleration.z;
    delay(5);
  }
  // We expect Z to be 9.8, but X and Y should be 0.
  offAX = sumX / samples;
  offAY = sumY / samples;
  // We don't zero Z because it measures gravity.
  Serial.println("Calibration Complete.");
}