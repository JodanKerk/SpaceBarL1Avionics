#include <SPI.h>
#include <LoRa.h>

#define LoRa_SS 10
#define LoRa_RESET 9
#define LoRa_DIO0 2

void setup() {
  Serial.begin(74880);
  while (!Serial);

  Serial.println("LoRa Receiver Serial Started");
  LoRa.setPins(LoRa_SS, LoRa_RESET, LoRa_SS);
  if (!LoRa.begin(920E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
  LoRa.setSpreadingFactor(10);
  LoRa.enableCrc();
}


void loop() {
  // try to parse packet
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    // print RSSI of packet
    Serial.print("Received packet with RSSI ");
    Serial.println(LoRa.packetRssi());

    // read packet
    while (LoRa.available()) {
      Serial.print((char)LoRa.read());
    }
    Serial.println();
  }
}
