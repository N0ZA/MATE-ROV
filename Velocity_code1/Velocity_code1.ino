#include <SPI.h>
#include <Wire.h>

// RM3100 SPI Pins & Registers
#define RM3100_CS 10
#define RM3100_CMM 0x01
#define RM3100_CCX 0x04
#define RM3100_MEAS 0x24

// SmartElex (ISM330DHCX) I2C Address & Registers
#define ISM_ADDR 0x6B
#define ISM_CTRL1_XL 0x10 

// --- CALIBRATION SECTION ---
// Run the code, rotate the sensor 360 degrees, find the min/max for X and Y.
// Offset = (Max + Min) / 2
float magOffX = 0; 
float magOffY = 0;
float magOffZ = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial);

  // 1. Initialize SPI for RM3100
  pinMode(RM3100_CS, OUTPUT);
  digitalWrite(RM3100_CS, HIGH);
  SPI.begin();

  // 2. Initialize I2C for SmartElex
  Wire.begin();
  Wire.setClock(400000);

  // 3. Setup RM3100
  // Set Cycle Counts (200 is a good balance of speed/noise)
  writeRMRegister(RM3100_CCX, 0x00); writeRMRegister(RM3100_CCX + 1, 0xC8); 
  writeRMRegister(0x06, 0x00); writeRMRegister(0x07, 0xC8); 
  writeRMRegister(0x08, 0x00); writeRMRegister(0x09, 0xC8); 
  writeRMRegister(RM3100_CMM, 0x79); // Continuous Measurement Mode

  // 4. Setup SmartElex Accelerometer
  Wire.beginTransmission(ISM_ADDR);
  Wire.write(ISM_CTRL1_XL);
  Wire.write(0x40); // 104Hz, +/- 2g
  Wire.endTransmission();

  Serial.println("System Initialized. Starting Data Stream...");
}

void loop() {
  // --- 1. READ ACCELEROMETER ---
  float ax, ay, az;
  readAccel(ax, ay, az);

  // Calculate Pitch and Roll (in Radians)
  // 
  float roll = atan2(ay, az);
  float pitch = atan2(-ax, sqrt(ay * ay + az * az));

  // --- 2. READ MAGNETOMETER ---
  int32_t rawMx = read24Bit(0x24);
  int32_t rawMy = read24Bit(0x27);
  int32_t rawMz = read24Bit(0x2A);

  // Apply Calibration Offsets
  float mx = (float)rawMx - magOffX;
  float my = (float)rawMy - magOffY;
  float mz = (float)rawMz - magOffZ;

  // --- 3. TILT COMPENSATION ---
  // Transforms 3D magnetic readings into a 2D horizontal plane
  // 
  float Xh = mx * cos(pitch) + my * sin(roll) * sin(pitch) + mz * cos(roll) * sin(pitch);
  float Yh = my * cos(roll) - mz * sin(roll);

  // Calculate Heading (Yaw)
  float yaw = atan2(-Yh, Xh) * 180.0 / PI;

  // Normalize to 0-360 degrees
  if (yaw < 0) yaw += 360;

  // --- 4. OUTPUT ---
  Serial.print("P: "); Serial.print(pitch * 180/PI, 1);
  Serial.print("\tR: "); Serial.print(roll * 180/PI, 1);
  Serial.print("\tYAW: "); Serial.println(yaw, 1);

  delay(50); // 20Hz update rate
}

// --- HELPER FUNCTIONS ---

void readAccel(float &ax, float &ay, float &az) {
  Wire.beginTransmission(ISM_ADDR);
  Wire.write(0x28); 
  Wire.endTransmission(false);
  Wire.requestFrom(ISM_ADDR, 6);
  
  if (Wire.available() == 6) {
    int16_t rawX = Wire.read() | (Wire.read() << 8);
    int16_t rawY = Wire.read() | (Wire.read() << 8);
    int16_t rawZ = Wire.read() | (Wire.read() << 8);

    ax = rawX * 0.061 / 1000.0; 
    ay = rawY * 0.061 / 1000.0;
    az = rawZ * 0.061 / 1000.0;
  }
}

void writeRMRegister(uint8_t reg, uint8_t val) {
  digitalWrite(RM3100_CS, LOW);
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  SPI.transfer(reg & ~0x80);
  SPI.transfer(val);
  SPI.endTransaction();
  digitalWrite(RM3100_CS, HIGH);
}

int32_t read24Bit(uint8_t reg) {
  uint32_t val;
  digitalWrite(RM3100_CS, LOW);
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  SPI.transfer(reg | 0x80);
  val = (uint32_t)SPI.transfer(0x00) << 16;
  val |= (uint32_t)SPI.transfer(0x00) << 8;
  val |= (uint32_t)SPI.transfer(0x00);
  SPI.endTransaction();
  digitalWrite(RM3100_CS, HIGH);

  if (val & 0x800000) val |= 0xFF000000; // Sign extension
  return (int32_t)val;
}