#include <SPI.h>
#include <RH_RF95.h>             // LoRa RadioHead Library
#include <BLEDevice.h>           // Built-in BLE library
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// LoRa pin configuration
#define RFM95_CS   5
#define RFM95_RST  4
#define RFM95_INT  3
#define RF95_FREQ  868.0

RH_RF95 rf95(RFM95_CS, RFM95_INT);

LiquidCrystal_I2C lcd(0x27, 16, 2);  // 0x27 is the common I2C address

// BLE UUIDs
#define SERVICE_UUID        "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
#define CHARACTERISTIC_RX   "6e400002-b5a3-f393-e0a9-e50e24dcca9e"
#define CHARACTERISTIC_TX   "6e400003-b5a3-f393-e0a9-e50e24dcca9e"

BLECharacteristic *pTxCharacteristic;
bool deviceConnected = false;

class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
    Serial.println("✅ BLE device connected");
  }
  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
    Serial.println("❌ BLE device disconnected");
  }
};

class CharacteristicCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
  String rxValue = pCharacteristic->getValue();
  if (rxValue.length() > 0) {
    Serial.print("📥 BLE Received: ");
    Serial.println(rxValue);
    // Send over LoRa
    rf95.send((uint8_t*)rxValue.c_str(), rxValue.length());
    rf95.waitPacketSent();
    Serial.println("📡 LoRa Sent!");
  }
}

};

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("🚀 Starting BLE + LoRa Node");

  Wire.begin(8, 22);  // SDA, SCL
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("LoRaMobile Ready");

  // LoRa Reset
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, LOW);
  delay(100);
  digitalWrite(RFM95_RST, HIGH);
  delay(100);

  // SPI and LoRa Init
  SPI.begin(18, 19, 23);
  if (!rf95.init()) {
    Serial.println("❌ LoRa init failed");
    while (1);
  }
  rf95.setFrequency(RF95_FREQ);
   rf95.setModemConfig(RH_RF95::Bw125Cr48Sf4096);  // Long range config
  rf95.setTxPower(15, false);
  Serial.println("✅ LoRa Ready");

  // BLE Init
  BLEDevice::init("LoRaMobile");
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);
  BLECharacteristic *pRxCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_RX,
    BLECharacteristic::PROPERTY_WRITE
  );
  pRxCharacteristic->setCallbacks(new CharacteristicCallbacks());

  pTxCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_TX,
    BLECharacteristic::PROPERTY_NOTIFY
  );
  pTxCharacteristic->addDescriptor(new BLE2902());

  pService->start();
  pServer->getAdvertising()->start();
  Serial.println("📶 BLE Service started and advertising");
}

unsigned long lastMessageTime = 0;
bool messageDisplayed = false;

void loop() {
  if (rf95.available()) {
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
    if (rf95.recv(buf, &len)) {
      buf[len] = '\0';
      int rssi = rf95.lastRssi();
      Serial.print("📡 LoRa Received: ");
      Serial.println((char*)buf);

      // Update LCD
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("From Base: ");
      lcd.print(rssi);  // Show RSSI
      lcd.setCursor(0, 1);
      lcd.print((char*)buf);  // Message

      // Update BLE
      if (deviceConnected) {
        pTxCharacteristic->setValue((char*)buf);
        pTxCharacteristic->notify();
        Serial.println("📤 Notified BLE device");
      }

      // Set timer
      lastMessageTime = millis();
      messageDisplayed = true;
    }
  }

  // Clear LCD if 10 seconds passed since last message
  if (messageDisplayed && millis() - lastMessageTime > 10000) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Waiting for msg");
    messageDisplayed = false;
  }

  delay(10);
}
