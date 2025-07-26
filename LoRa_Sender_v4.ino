#include <SPI.h>
#include <RH_RF95.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include <TinyGPSPlus.h>
#include <SD.h>

// ==== LoRa Configuration ====
#define RFM95_CS   5
#define RFM95_RST  4
#define RFM95_INT  3
#define RF95_FREQ  868.0

// ==== SPI (Shared for LoRa and SD) ====
#define SD_CS   7
#define SD_SCK  18
#define SD_MISO 19
#define SD_MOSI 23

// ==== GPS (UART1) ====
#define GPS_RX 20
#define GPS_TX 21
HardwareSerial GPSSerial(1);
TinyGPSPlus gps;

// ==== BLE UUIDs ====
#define SERVICE_UUID        "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
#define CHARACTERISTIC_RX   "6e400002-b5a3-f393-e0a9-e50e24dcca9e"
#define CHARACTERISTIC_TX   "6e400003-b5a3-f393-e0a9-e50e24dcca9e"

// ==== Globals ====
RH_RF95 rf95(RFM95_CS, RFM95_INT);
BLECharacteristic *pTxCharacteristic;
bool deviceConnected = false;
File logFile;
String lastMessage = "";  // For avoiding duplicate LoRa sends

void logToSD(const String& msg) {
  Serial.println(msg);
  if (logFile) {
    logFile.println(msg);
    logFile.flush();
  }
}

// ==== BLE Callbacks ====
class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
    Serial.println("✅ BLE connected");
  }
  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
    Serial.println("❌ BLE disconnected");
  }
};

class CharacteristicCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    String rxValue = pCharacteristic->getValue();

    // Skip if it's the same message as before
    if (rxValue == lastMessage) return;
    lastMessage = rxValue;

    if (rxValue.length() > 0) {
      Serial.print("📥 BLE Received: ");
      Serial.println(rxValue);

      // ⛔ Make sure SD card is not active
      digitalWrite(SD_CS, HIGH);

      // ✅ LoRa Send
      rf95.send((uint8_t*)rxValue.c_str(), rxValue.length());
      rf95.waitPacketSent();

      rf95.setModeRx();  // Go into RX mode immediately
      Serial.println("📡 LoRa Sent!");
      Serial.println("👂 Waiting for reply...");

      unsigned long start = millis();
      bool gotReply = false;

      while (millis() - start < 5000) {
        if (rf95.available()) {
          uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
          uint8_t len = sizeof(buf);
          if (rf95.recv(buf, &len)) {
            buf[len] = '\0';
            String reply = String((char*)buf);
            int rssi = rf95.lastRssi();

            Serial.print("📨 Got reply: ");
            Serial.println(reply);

            // ✅ BLE Notify
            if (deviceConnected) {
              pTxCharacteristic->setValue(reply.c_str());
              pTxCharacteristic->notify();
              Serial.println("📤 Notified BLE device");
            }

            // ✅ Log with GPS
            String logEntry = "Reply: " + reply + " | RSSI: " + String(rssi);
            if (gps.location.isValid()) {
              logEntry += " | Lat: " + String(gps.location.lat(), 6);
              logEntry += ", Lon: " + String(gps.location.lng(), 6);
            } else {
              logEntry += " | No GPS fix";
            }

            digitalWrite(RFM95_CS, HIGH);  // Deselect LoRa before SD
            logToSD(logEntry);
            gotReply = true;
            break;
          }
        }
        delay(10);
      }

      if (!gotReply) {
        Serial.println("⚠️ No reply from base station");
        digitalWrite(RFM95_CS, HIGH);
        logToSD("⚠️ No reply from base station");
      }

      digitalWrite(RFM95_CS, HIGH);  // Re-enable SD next time
    }
  }
};

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("🚀 Starting BLE + LoRa + GPS + SD");

  pinMode(SD_CS, OUTPUT); digitalWrite(SD_CS, HIGH);
  pinMode(RFM95_CS, OUTPUT); digitalWrite(RFM95_CS, HIGH);

  // Reset LoRa
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, LOW); delay(100);
  digitalWrite(RFM95_RST, HIGH); delay(100);

  SPI.begin(SD_SCK, SD_MISO, SD_MOSI);

  if (!rf95.init()) {
    Serial.println("❌ LoRa init failed");
    while (1);
  }
  rf95.setFrequency(RF95_FREQ);
  rf95.setModemConfig(RH_RF95::Bw125Cr48Sf4096);
  rf95.setTxPower(15, false);
  Serial.println("✅ LoRa Ready");

  // GPS
  GPSSerial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
  Serial.println("🛰️ GPS Ready");

  // SD Card
  if (SD.begin(SD_CS)) {
    logFile = SD.open("/mobile_log.txt", FILE_APPEND);
    if (logFile) {
      logToSD("===== BOOT =====");
      logToSD("📦 SD card OK");
    }
  } else {
    Serial.println("❌ SD card failed");
  }

  // BLE Setup
  BLEDevice::init("LoRaMobile");
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);
  BLECharacteristic *pRxCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_RX, BLECharacteristic::PROPERTY_WRITE
  );
  pRxCharacteristic->setCallbacks(new CharacteristicCallbacks());

  pTxCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_TX, BLECharacteristic::PROPERTY_NOTIFY
  );
  pTxCharacteristic->addDescriptor(new BLE2902());

  pService->start();
  pServer->getAdvertising()->start();
  Serial.println("📶 BLE advertising");
}

void loop() {
  while (GPSSerial.available()) {
    gps.encode(GPSSerial.read());
  }
  delay(10);  // Keep loop light
}
