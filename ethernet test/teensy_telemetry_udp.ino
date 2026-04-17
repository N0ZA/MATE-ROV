#include <QNEthernet.h>

QNEthernet ethernet;
UDP udp;

void setup() {
    ethernet.begin();
    udp.begin(1234); // Port to listen on
}

void loop() {
    // Prepare telemetry data
    String telemetryData = "Telemetry data";

    // Send telemetry data
    udp.beginPacket("255.255.255.255", 1234); // Broadcast address
    udp.write(telemetryData.c_str());
    udp.endPacket();

    delay(1000); // Send every second
}