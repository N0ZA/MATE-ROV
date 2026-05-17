#include <Servo.h>
#include <NativeEthernet.h>
#include <NativeEthernetUdp.h>

// =====================================================
// ETHERNET CONFIG
// =====================================================

byte mac[] = { 0x04, 0xE9, 0xE5, 0x12, 0x34, 0x56 };

IPAddress teensyIP(192,168,2,177);
unsigned int localPort = 5000;

EthernetUDP Udp;
char udpBuf[512];

// =====================================================
// CONFIG
// =====================================================

// Thrusters
const uint8_t THRUSTER_PINS[] = {8, 7, 6, 5, 4, 3};

// Motor drivers actually USED
const uint8_t MOTOR_DRIVER_PINS[] = {
    26,27,31,
    32,34,35
};

// Relays used
const uint8_t RELAY_PINS[] = {15,20,21,22};

const uint8_t NUM_THRUSTERS = 6;
const uint8_t NUM_MOTORS = 6;
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

bool relayState[NUM_RELAYS] = {0,0,0,0};

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

    us = clampUS(us);

    target[i] = us;

    if (ch[i].attached()) {
        ch[i].writeMicroseconds(us);
    }
}

void setAll(int us) {

    for (int i = 0; i < TOTAL; i++) {
        writeCH(i, us);
    }
}

// =====================================================
// RELAY CONTROL
// =====================================================

void writeRelay(int i, bool state) {

    if (i < 0 || i >= NUM_RELAYS) return;

    relayState[i] = state;

    digitalWrite(RELAY_PINS[i], state ? LOW : HIGH);
}

void allRelaysOff() {

    for (int i = 0; i < NUM_RELAYS; i++) {
        writeRelay(i, LOW);
    }
}

// =====================================================
// SERIAL PARSER (OLD SUPPORT KEPT)
// =====================================================

String serialBuf = "";

void processSerial(String s) {

    s.trim();

    if (s.length() == 0) return;

    lastPacket = millis();

    // =============================================
    // ARRAY MODE
    // =============================================

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

            for (int i = 0; i < TOTAL; i++) {
                writeCH(i, v[i]);
            }
        }

        return;
    }

    // =============================================
    // COMMAND MODE
    // =============================================

    char cmd[10];
    int val;

    if (sscanf(s.c_str(), "%9s %d", cmd, &val) >= 2) {

        // Thrusters
        if (cmd[0] == 'T') {
            writeCH(atoi(&cmd[1]) - 1, val);
        }

        // Motor Drivers
        else if (cmd[0] == 'M' && cmd[1] == 'D') {
            writeCH(atoi(&cmd[2]) - 1 + NUM_THRUSTERS, val);
        }

        // All ESCs
        else if (strcmp(cmd, "ALL") == 0) {
            setAll(val);
        }

        // Relays
        else if (cmd[0] == 'R') {
            int idx = atoi(&cmd[1]) - 1;
            writeRelay(idx, val);
        }
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

            if (serialBuf.length() > 200)
                serialBuf = "";
        }
    }
}

// =====================================================
// UDP JSON PARSER
// =====================================================

void processUDP(char* msg) {

    char* p = strstr(msg, "\"pwms\":[");

    if (!p) {
        Serial.println("No pwms field");
        return;
    }

    p += strlen("\"pwms\":[");

    int values[16];
    int count = 0;

    while (*p && *p != ']' && count < 16) {

        values[count++] = atoi(p);

        while (*p && *p != ',' && *p != ']') {
            p++;
        }

        if (*p == ',') p++;
    }

    if (count != 16) {
        Serial.print("Invalid PWM packet count: ");
        Serial.println(count);
        return;
    }

    lastPacket = millis();

    // ============================================
    // THRUSTERS (0-5)
    // ============================================
    Serial.println("=== THRUSTERS ===");
    for (int i = 0; i < 6; i++) {
        writeCH(i, values[i]);
        Serial.print("T"); Serial.print(i + 1); Serial.print(": "); Serial.println(values[i]);
    }

    // ============================================
    // MOTOR DRIVERS SET 1 (6-8)
    // ============================================
    Serial.println("=== MOTOR DRIVERS 1 ===");
    for (int i = 0; i < 3; i++) {
        writeCH(6 + i, values[6 + i]);
        Serial.print("MD"); Serial.print(i + 1); Serial.print(": "); Serial.println(values[6 + i]);
    }

    // ============================================
    // RELAYS SET 1 (9-10)
    // ============================================
    Serial.println("=== RELAYS 1 ===");
    for (int i = 0; i < 2; i++) {
        writeRelay(i, values[9 + i]);
        Serial.print("R"); Serial.print(i + 1); Serial.print(": "); Serial.println(values[9 + i]);
    }

    // ============================================
    // MOTOR DRIVERS SET 2 (11-13)
    // ============================================
    Serial.println("=== MOTOR DRIVERS 2 ===");
    for (int i = 0; i < 3; i++) {
        writeCH(9 + i, values[11 + i]);
        Serial.print("MD"); Serial.print(4 + i); Serial.print(": "); Serial.println(values[11 + i]);
    }

    // ============================================
    // RELAYS SET 2 (14-15)
    // ============================================
    Serial.println("=== RELAYS 2 ===");
    for (int i = 0; i < 2; i++) {
        writeRelay(2 + i, values[14 + i]);
        Serial.print("R"); Serial.print(3 + i); Serial.print(": "); Serial.println(values[14 + i]);
    }

    Serial.println("=== PWM packet applied ===\n");
}
// =====================================================
// UDP RECEIVE
// =====================================================

void readUDP() {

    int packetSize = Udp.parsePacket();

    if (packetSize > 0) {

        int len = Udp.read(udpBuf, sizeof(udpBuf)-1);

        if (len > 0) {

            udpBuf[len] = 0;

            Serial.print("Received: ");
            Serial.println(udpBuf);

            processUDP(udpBuf);
        }
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
// SETUP
// =====================================================

void setup() {

    Serial.begin(115200);

    while (!Serial && millis() < 4000) {}

    Serial.println("BOOTING ESC + RELAY + ETHERNET SYSTEM");

    // =================================================
    // ETHERNET INIT
    // =================================================

    Ethernet.begin(mac, teensyIP);

    delay(1000);

    Serial.print("Teensy IP: ");
    Serial.println(Ethernet.localIP());

    Udp.begin(localPort);

    Serial.print("Listening UDP port: ");
    Serial.println(localPort);

    // =================================================
    // ESC INIT
    // =================================================

    uint8_t pins[TOTAL] = {
        8,7,6,5,4,3,
        26,27,31,
        32,34,35
    };

    for (int i = 0; i < TOTAL; i++) {

        ch[i].attach(pins[i], 1000, 2000);

        ch[i].writeMicroseconds(1500);

        target[i] = 1500;

        delay(20);
    }

    // =================================================
    // RELAY INIT
    // =================================================

    for (int i = 0; i < NUM_RELAYS; i++) {

        pinMode(RELAY_PINS[i], OUTPUT);

        digitalWrite(RELAY_PINS[i], LOW);
    }

    delay(2000);

    setAll(1500);

    allRelaysOff();

    lastPacket = millis();

    Serial.println("READY");
}

// =====================================================
// LOOP
// =====================================================

void loop() {

    readSerial();

    readUDP();

    failsafe();

    static unsigned long t = 0;

    if (millis() - t > 20) {

        t = millis();

        for (int i = 0; i < TOTAL; i++) {
            ch[i].writeMicroseconds(target[i]);
        }
    }
}