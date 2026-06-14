#include <Servo.h>
#include <NativeEthernet.h>
#include <NativeEthernetUdp.h>
#include <Wire.h>
#include <Adafruit_ISM330DHCX.h>
#include <MadgwickAHRS.h>
#include <MS5837.h>

// ========================================f=============
// ETHERNET CONFIG
// =====================================================

byte mac[] = { 0x04, 0xE9, 0xE5, 0x12, 0x34, 0x56 };

IPAddress teensyIP(192, 168, 2, 177);

// Control: surface -> Teensy
unsigned int controlLocalPort = 5000;

// Telemetry: Teensy -> surface
unsigned int telemetryLocalPort = 5001;

NativeEthernetUDP UdpControl;
NativeEthernetUDP UdpTelem;

char udpBuf[512];

IPAddress surfaceIP(192, 168, 2, 1);
unsigned int surfaceControlPort = 5000;
unsigned int surfaceTelemetryPort = 5001;

// =====================================================
// IMU CONFIG
// =====================================================
// =====================================================
// RM3100 CONFIG
// =====================================================

#define RM3100_ADDR   0x22
#define RM3100_CCX    0x04
#define RM3100_CMM    0x01
#define RM3100_TMRC   0x0B
#define RM3100_MX     0x24
#define RM3100_STATUS 0x34
#define RM3100_REVID  0x36
#define RM3100_CC     200

const float magGain = (0.3671f * RM3100_CC) + 1.5f;
float mx = 0, my = 0, mz = 0;
bool magOk = false;

void rm3100WriteReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(RM3100_ADDR);
  Wire.write(reg); Wire.write(val);
  Wire.endTransmission();
}

void rm3100WriteReg16(uint8_t reg, uint16_t val) {
  Wire.beginTransmission(RM3100_ADDR);
  Wire.write(reg);
  Wire.write((val >> 8) & 0xFF);
  Wire.write(val & 0xFF);
  Wire.endTransmission();
}

uint8_t rm3100ReadReg(uint8_t reg) {
  Wire.beginTransmission(RM3100_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)RM3100_ADDR, (uint8_t)1);
  return Wire.available() ? Wire.read() : 0xFF;
}

bool rm3100DataReady() {
  return (rm3100ReadReg(RM3100_STATUS) & 0x80);
}

void rm3100ReadMag(float &ox, float &oy, float &oz) {
  Wire.beginTransmission(RM3100_ADDR);
  Wire.write(RM3100_MX);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)RM3100_ADDR, (uint8_t)9);
  uint8_t buf[9];
  for (int i = 0; i < 9; i++)
    buf[i] = Wire.available() ? Wire.read() : 0;
  int32_t rx = ((int32_t)buf[0] << 16) | ((uint32_t)buf[1] << 8) | buf[2];
  int32_t ry = ((int32_t)buf[3] << 16) | ((uint32_t)buf[4] << 8) | buf[5];
  int32_t rz = ((int32_t)buf[6] << 16) | ((uint32_t)buf[7] << 8) | buf[8];
  if (rx & 0x800000) rx |= 0xFF000000;
  if (ry & 0x800000) ry |= 0xFF000000;
  if (rz & 0x800000) rz |= 0xFF000000;
  ox = (float)rx / magGain;
  oy = (float)ry / magGain;
  oz = (float)rz / magGain;
}
Adafruit_ISM330DHCX ism;
Madgwick filter;

float roll = 0;
float pitch = 0;
float yaw = 0;
float offAX = 0, offAY = 0, offAZ = 0;
unsigned long lastUpdate = 0;
bool imuOk = false;

// =====================================================
// BAR30 CONFIG
// =====================================================

MS5837 bar30;
float depth = 0;
bool barOk = false;

// =====================================================
// HARDWARE CONFIG
// =====================================================

const uint8_t RELAY_PINS[] = {15, 20, 21, 22};

const uint8_t NUM_THRUSTERS = 6;
const uint8_t NUM_RELAYS = 4;
const uint8_t TOTAL = 12;

// =====================================================
// ESC CHANNELS
// =====================================================

Servo ch[TOTAL];
int target[TOTAL];
unsigned long lastPacket = 0;

// =====================================================
// RELAY STATE
// =====================================================

bool relayState[NUM_RELAYS] = {0, 0, 0, 0};

// =====================================================
// LOG BUFFER
// =====================================================

const int LOG_LINES = 48;
String logQ[LOG_LINES];
volatile uint8_t logHead = 0;
volatile uint8_t logTail = 0;

void logMsg(const String &s) {
  uint8_t next = (logHead + 1) % LOG_LINES;
  if (next == logTail) return;
  logQ[logHead] = s;
  logHead = next;
}

void flushLogs() {
  int sent = 0;
  while (logTail != logHead && Serial.availableForWrite() > 48 && sent < 4) {
    Serial.println(logQ[logTail]);
    logTail = (logTail + 1) % LOG_LINES;
    sent++;
  }
}

// =====================================================
// SAFETY
// =====================================================

int clampUS(int us) {
  if (us < 1000) return 1000;
  if (us > 2000) return 2000;
  return us;
}

// =====================================================
// ESC CONTROL
// =====================================================

void writeCH(int i, int us) {
  if (i < 0 || i >= TOTAL) return;
  target[i] = clampUS(us);
}

void refreshESCs() {
  for (int i = 0; i < TOTAL; i++) {
    if (ch[i].attached()) ch[i].writeMicroseconds(target[i]);
  }
}

void setAll(int us) {
  for (int i = 0; i < TOTAL; i++) writeCH(i, us);
}

// =====================================================
// RELAY CONTROL
// =====================================================

void writeRelay(int i, bool state) {
  if (i < 0 || i >= NUM_RELAYS) return;
  relayState[i] = state;
  digitalWrite(RELAY_PINS[i], state ? LOW : HIGH);   // active-low
}

void allRelaysOff() {
  for (int i = 0; i < NUM_RELAYS; i++) writeRelay(i, false);
}

// =====================================================
// SERIAL PARSER
// =====================================================

String serialBuf = "";

void processSerial(String s) {
  s.trim();
  if (s.length() == 0) return;

  lastPacket = millis();

  if (s.indexOf(',') >= 0) {
    int v[TOTAL];
    int n = 0;
    char c[s.length() + 1];
    s.toCharArray(c, sizeof(c));
    char *t = strtok(c, ",");
    while (t && n < TOTAL) {
      v[n++] = atoi(t);
      t = strtok(NULL, ",");
    }
    if (n == TOTAL) {
      for (int i = 0; i < TOTAL; i++) writeCH(i, v[i]);
      logMsg("[RX SERIAL] 12 PWM values applied");
    } else {
      logMsg(String("[RX SERIAL] invalid PWM count = ") + n);
    }
    return;
  }

  char cmd[10];
  int val;
  if (sscanf(s.c_str(), "%9s %d", cmd, &val) >= 2) {
    if (cmd[0] == 'T') writeCH(atoi(&cmd[1]) - 1, val);
    else if (cmd[0] == 'M' && cmd[1] == 'D') writeCH(atoi(&cmd[2]) - 1 + NUM_THRUSTERS, val);
    else if (strcmp(cmd, "ALL") == 0) setAll(val);
    else if (cmd[0] == 'R') writeRelay(atoi(&cmd[1]) - 1, val);

    logMsg(String("[RX SERIAL] ") + s);
  }
}

void readSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      processSerial(serialBuf);
      serialBuf = "";
    } else {
      serialBuf += c;
      if (serialBuf.length() > 200) serialBuf = "";
    }
  }
}

// =====================================================
// UDP CONTROL PARSER
// Expected: {"pwms":[t1,t2,t3,t4,t5,t6,md1,md2,md3,r1,r2,md4,md5,md6,r3,r4]}
// =====================================================

void processControlUDP(char* msg) {
  char* p = strstr(msg, "\"pwms\":[");
  if (!p) return;

  p += strlen("\"pwms\":[");

  int values[16];
  int count = 0;

  while (*p && *p != ']' && count < 16) {
    values[count++] = atoi(p);
    while (*p && *p != ',' && *p != ']') p++;
    if (*p == ',') p++;
  }

  if (count != 16) {
    logMsg(String("[RX CTRL] invalid PWM count: ") + count);
    return;
  }

  lastPacket = millis();

  for (int i = 0; i < 6; i++) writeCH(i, values[i]);
  for (int i = 0; i < 3; i++) writeCH(6 + i, values[6 + i]);
  for (int i = 0; i < 2; i++) writeRelay(i, values[9 + i]);
  for (int i = 0; i < 3; i++) writeCH(9 + i, values[11 + i]);
  for (int i = 0; i < 2; i++) writeRelay(2 + i, values[14 + i]);

  char ipBuf[24];
  snprintf(ipBuf, sizeof(ipBuf), "%u.%u.%u.%u",
           surfaceIP[0], surfaceIP[1], surfaceIP[2], surfaceIP[3]);
  logMsg(String("[RX CTRL] PWM applied from ") + ipBuf);
}

// =====================================================
// UDP RECEIVE CONTROL
// =====================================================

void readControlUDP() {
  int processed = 0;
  const int maxPacketsPerLoop = 2;

  while (processed < maxPacketsPerLoop) {
    int packetSize = UdpControl.parsePacket();
    if (packetSize <= 0) break;

    surfaceIP = UdpControl.remoteIP();
    surfaceControlPort = UdpControl.remotePort();

    int len = UdpControl.read(udpBuf, sizeof(udpBuf) - 1);
    if (len > 0) {
      udpBuf[len] = 0;
      processControlUDP(udpBuf);
    }

    processed++;
  }
}

// =====================================================
// FAILSAFE
// =====================================================

void failsafe() {
  if (millis() - lastPacket > 500) {
    setAll(1500);
    allRelaysOff();
  }
}

// =====================================================
// IMU CALIBRATION
// =====================================================

void calibrateSensors() {
  float sx = 0, sy = 0, sz = 0;
  const int samples = 500;

  for (int i = 0; i < samples; i++) {
    sensors_event_t a, g, t;
    ism.getEvent(&a, &g, &t);
    sx += a.acceleration.x;
    sy += a.acceleration.y;
    sz += a.acceleration.z;
    Ethernet.maintain();
    delay(2);
  }

  offAX = sx / samples;
  offAY = sy / samples;
  offAZ = (sz / samples) - 9.80665f;
}

// =====================================================
// IMU READ
// =====================================================
void readIMU() {
  if (!imuOk) return;

  sensors_event_t accel, gyro, temp;
  if (!ism.getEvent(&accel, &gyro, &temp)) return;

  unsigned long now = micros();
  float dt = (now - lastUpdate) / 1000000.0f;
  lastUpdate = now;
  if (dt <= 0 || dt > 0.1f) dt = 0.01f;

  float gx = gyro.gyro.x * 57.2958f;
  float gy = gyro.gyro.y * 57.2958f;
  float gz = gyro.gyro.z * 57.2958f;
  float ax = accel.acceleration.x;
  float ay = accel.acceleration.y;
  float az = accel.acceleration.z;

  if (magOk && rm3100DataReady()) {
    rm3100ReadMag(mx, my, mz);
    filter.update(gx, gy, gz, ax, ay, az, mx, my, mz);  // 9-DOF fusion
  } else {
    filter.updateIMU(gx, gy, gz, ax, ay, az);            // fallback
  }

  roll  = filter.getRoll();
  pitch = filter.getPitch();
  yaw   = filter.getYaw();
}
// =====================================================
// BAR30 READ
// =====================================================

void readDepth() {
  if (!barOk) return;
  bar30.read();
  depth = bar30.depth();
}

// =====================================================
// SEND TELEMETRY
// Format: [ISM_ROLL,ISM_PITCH,RM3100_YAW,BAR30_DEPTH,imuOk,magOk]
// =====================================================

void sendTelemetry() {
  char outBuf[96];
  snprintf(outBuf, sizeof(outBuf),
           "[%.2f,%.2f,%.2f,%.3f,%d,%d]\n",
           imuOk ? roll  : 0.0f,
           imuOk ? pitch : 0.0f,
           magOk ? yaw   : 0.0f,
           barOk ? depth : 0.0f,
           (int)imuOk,
           (int)magOk);

  UdpTelem.beginPacket(surfaceIP, surfaceTelemetryPort);
  UdpTelem.write((uint8_t*)outBuf, strlen(outBuf));
  UdpTelem.endPacket();

  logMsg(String("[TX TELEM] ") + outBuf);
}

// =====================================================
// SETUP
// =====================================================

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 4000) {}
  Serial.println("Serial OK");

  Serial.println("Starting Ethernet...");
  Ethernet.begin(mac, teensyIP);
  delay(500);

  UdpControl.begin(controlLocalPort);
  UdpTelem.begin(telemetryLocalPort);

  Serial.print("Teensy IP: ");
  Serial.println(Ethernet.localIP());
  Serial.print("Listening control UDP port: ");
  Serial.println(controlLocalPort);
  Serial.print("Telemetry UDP local port: ");
  Serial.println(telemetryLocalPort);
  Serial.println("Ethernet OK.");

  Wire.begin();
  Wire.setClock(400000);

  if (!ism.begin_I2C(0x6B)) {
    Serial.println("WARNING: ISM330DHCX not found — roll/pitch/yaw will send 0.00");
    imuOk = false;
  } else {
    ism.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
    ism.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS);
    filter.begin(100);
    Serial.println("Calibrating IMU... do not move.");
    calibrateSensors();
    imuOk = true;
    lastUpdate = micros();
    Serial.println("IMU online.");
    logMsg("IMU online");
  }
  
  // RM3100 init 
  uint8_t revid = rm3100ReadReg(RM3100_REVID);
  if (revid != 0x22) {
    Serial.println("WARNING: RM3100 not found — yaw will drift");
    magOk = false;
  } else {
    rm3100WriteReg16(RM3100_CCX,     RM3100_CC);
    rm3100WriteReg16(RM3100_CCX + 2, RM3100_CC);
    rm3100WriteReg16(RM3100_CCX + 4, RM3100_CC);
    rm3100WriteReg(RM3100_TMRC, 0x96);
    rm3100WriteReg(RM3100_CMM,  0x71);
    magOk = true;
    Serial.println("RM3100 online.");
    logMsg("RM3100 online");
  }

  if (!bar30.init()) {
    Serial.println("WARNING: Bar30 not found — depth will send 0.000");
    barOk = false;
  } else {
    bar30.setModel(MS5837::MS5837_30BA);
    bar30.setFluidDensity(997.0f);
    barOk = true;
    Serial.println("Bar30 online.");
    logMsg("Bar30 online");
  }

  uint8_t pins[TOTAL] = {
    8, 7, 6, 5, 4, 3,
    26, 27, 31,
    32, 34, 35
  };

  for (int i = 0; i < TOTAL; i++) {
    ch[i].attach(pins[i], 1000, 2000);
    ch[i].writeMicroseconds(1500);
    target[i] = 1500;
    delay(20);
  }

  for (int i = 0; i < NUM_RELAYS; i++) {
    pinMode(RELAY_PINS[i], OUTPUT);
    digitalWrite(RELAY_PINS[i], HIGH);
  }

  delay(2000);
  setAll(1500);
  allRelaysOff();
  refreshESCs();
  lastPacket = millis();

  Serial.println("READY");
  logMsg("READY");
}

// =====================================================
// LOOP
// =====================================================

void loop() {
  Ethernet.maintain();

  // Highest priority: incoming control
  readSerial();
  readControlUDP();
  failsafe();

  unsigned long now = millis();

  // Keep PWM smooth and steady
  static unsigned long escT = 0;
  if (now - escT >= 20) {
    escT = now;
    refreshESCs();
  }

  // Sensor acquisition
  static unsigned long sensorT = 0;
  if (now - sensorT >= 20) {
    sensorT = now;
    readIMU();
    readDepth();
  }

  // Telemetry TX on separate socket/port
  static unsigned long telemT = 0;
  if (now - telemT >= 100) {
    telemT = now;
    sendTelemetry();
  }

  // Serial logs, but bounded so they don't stall control
  static unsigned long logT = 0;
  if (now - logT >= 10) {
    logT = now;
    flushLogs();
  }
}