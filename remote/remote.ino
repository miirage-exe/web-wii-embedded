#include <Adafruit_NeoPixel.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#include <cstdlib> // For using rand()

// Not used here
// #define CONSOLE_SERVICE_UUID "641f9229-8a89-4ec4-9588-afca9e9e724b"
// #define CONSOLE_CHARACTERISTIC_UUID "5039e8f7-c80e-40da-8d9c-fd2213bf3a41"

#define REMOTE_SERVICE_UUID "baf9bb4c-1e25-4318-9863-e7fa3e048351"
#define REMOTE_CHARACTERISTIC_UUID "d94ceacc-33c8-43e6-8b38-f126f7c00b6d"

#define BUTTON_A_PIN 7

// --- Board state ---------------------------------------------

struct BoardState {
  bool consoleConnected = false;
} boardState;

// --- Payloads ------------------------------------------------

struct RemotePayload {
  int32_t pitch = 0; // 65.58° -> 6558
  int32_t roll = 0; // 65.58° -> 6558
  int32_t yaw = 0; // 65.58° -> 6558
  uint8_t buttonMask = 0; // lowest bit (1) = button A, second bit (2) = button B 
} remotePayload; 

// Calculate the remote payload from the sensor data
// This function is called each time before sending the remote payload to the console
void getSensorData() {
  remotePayload.pitch += (rand()%3-1);
  remotePayload.yaw += (rand()%3-1);
  remotePayload.roll += (rand()%3-1)*100;

  // Button mask
  uint8_t mask = 0;
  if (digitalRead(BUTTON_A_PIN)) mask = mask | (1 << 0);
  remotePayload.buttonMask = mask;
}

// --- Neopixel ------------------------------------------------

Adafruit_NeoPixel pixels(1, PIN_NEOPIXEL); 

void initLed() {
  pixels.begin(); 
  updateLed();
}

void updateLed() {
  pixels.setPixelColor(0, pixels.Color(
    boardState.consoleConnected ? 255 : 0, // Red + Green: console is connected ?
    boardState.consoleConnected ? 255 : 0,
    255
  ));
  pixels.show();
}

// --- Captors ------------------------------------------------

void initPins() {
  pinMode(BUTTON_A_PIN, INPUT);
}

// --- Console Connection -------------------------------------

BLEServer* pServer = nullptr;
BLEService* pRemoteService = nullptr;
BLECharacteristic* pRemoteCharacteristic = nullptr;
BLEAdvertising* pRemoteAdvertising = nullptr;

class ConsoleCallback: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    Serial.printf("[Remote]: Console connected\n");
    boardState.consoleConnected = true;
    updateLed();
  }
  
  void onDisconnect(BLEServer* pServer) {
    boardState.consoleConnected = false;
    updateLed();
    delay(1000);
    BLEDevice::startAdvertising();
  }
};

void initRemote() {
  // Initialise the bluetooth
  BLEDevice::init("WiiRemote");

  // Create server
  pServer = BLEDevice::createServer();

  // Create remote service
  pRemoteService = pServer->createService(REMOTE_SERVICE_UUID);

  //Create remote characteristic
  pRemoteCharacteristic = pRemoteService->createCharacteristic(
    REMOTE_CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ |
    BLECharacteristic::PROPERTY_WRITE |
    BLECharacteristic::PROPERTY_NOTIFY
  );
  
  // Inform the remote characteristic that we have a callback on console connect/disconnect
  pServer->setCallbacks(new ConsoleCallback());  

  // Setup advertising
  pRemoteAdvertising = BLEDevice::getAdvertising();
  pRemoteAdvertising->addServiceUUID(REMOTE_SERVICE_UUID);
  pRemoteAdvertising->setScanResponse(true);
  pRemoteAdvertising->setMinPreferred(0x06);
  pRemoteAdvertising->setMinPreferred(0x12);  
  
  // Start the service
  pRemoteService->start();
}

void notifyConsole() {
  if (pRemoteCharacteristic != nullptr) {
    getSensorData();
    pRemoteCharacteristic->setValue((uint8_t*)&remotePayload, sizeof(remotePayload));
    pRemoteCharacteristic->notify();
    Serial.printf("[Remote]: Notified console\n");
  }
}

// -------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  
  delay(5000);
  initLed();
  initRemote();
  initPins();
  BLEDevice::startAdvertising(); // Make the remote visible to the console
  
  updateLed();
}

void loop() {
  static unsigned long lastTick = 0;

  if (boardState.consoleConnected && (millis() - lastTick > 20)) { // We send data at 50Hz, so every 20ms
    notifyConsole();
    lastTick = millis();
  }
}
