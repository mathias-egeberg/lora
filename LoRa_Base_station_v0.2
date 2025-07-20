#include <SPI.h>
#include <RH_RF95.h>

#define RFM95_CS  5
#define RFM95_RST 4
#define RFM95_INT 3
#define RF95_FREQ 868.0

RH_RF95 rf95(RFM95_CS, RFM95_INT);

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("🔋 LoRa Base Station");

  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, LOW);
  delay(100);
  digitalWrite(RFM95_RST, HIGH);
  delay(100);

  SPI.begin(18, 19, 23);

  if (!rf95.init()) {
    Serial.println("❌ LoRa init failed");
    while (1);
  }

  rf95.setFrequency(RF95_FREQ);
  rf95.setModemConfig(RH_RF95::Bw125Cr48Sf4096);  // Long range config
  rf95.setTxPower(15, false);
  Serial.println("✅ LoRa Ready");
}

void loop() {
  if (rf95.available()) {
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
    if (rf95.recv(buf, &len)) {
      buf[len] = '\0';
      int rssi = rf95.lastRssi();

      // Get timestamp
      unsigned long now = millis() / 1000;
      int hours = (now / 3600) % 24;
      int minutes = (now / 60) % 60;
      int seconds = now % 60;

      char timestamp[9]; // hh:mm:ss
      sprintf(timestamp, "%02d:%02d:%02d", hours, minutes, seconds);

      // Build reply
      char reply[64];
      snprintf(reply, sizeof(reply), "Time: %s | RSSI: %d", timestamp, rssi);

      Serial.print("📥 Received: ");
      Serial.println((char*)buf);
      Serial.print("📡 Sending reply: ");
      Serial.println(reply);

      rf95.send((uint8_t*)reply, strlen(reply));
      rf95.waitPacketSent();
      Serial.println("✅ Sent reply");
    }
  }

  // Manual serial send
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    rf95.send((uint8_t*)line.c_str(), line.length());
    rf95.waitPacketSent();
    Serial.print("📤 Manual reply sent: ");
    Serial.println(line);
  }
}
