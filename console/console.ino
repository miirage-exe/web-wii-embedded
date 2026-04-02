#include <Adafruit_NeoPixel.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#include <cstdlib> // For using rand()

#define CONSOLE_SERVICE_UUID "641f9229-8a89-4ec4-9588-afca9e9e724b"
#define CONSOLE_CHARACTERISTIC_UUID "5039e8f7-c80e-40da-8d9c-fd2213bf3a41"

#define REMOTE_SERVICE_UUID "baf9bb4c-1e25-4318-9863-e7fa3e048351"
#define REMOTE_CHARACTERISTIC_UUID "d94ceacc-33c8-43e6-8b38-f126f7c00b6d"

// --- Board state ---------------------------------------------

struct BoardState {
  bool computerConnected = false;
  bool remoteConnected = false;
  bool ready = false;
} boardState;

// --- Payloads ------------------------------------------------

struct RemotePayload {
  int32_t pitch = 0;
  int32_t roll = 0;
  int32_t yaw = 0;
  uint8_t buttonMask = 0; // lowest bit (1) = button A, second bit (2) = button B 
} remotePayload; 

struct IRCameraPayload {
  int32_t x = 64;
  int32_t y = 48;
  // Constants
  int32_t W = 128; // Width (in pixels) of the IR Camera (cf. documentation)
  int32_t H = 96; // Height (in pixels) of the IR Camera (cf. documentation)
} irCameraPayload;

struct ConsolePayload {
  int32_t x = 0 * 100;
  int32_t y = 0 * 100;
  int32_t roll = 0 * 100;
  uint8_t buttonMask = 0; // lowest bit (1) = button A, second bit (2) = button B 
} consolePayload;

const float Kw = 2;
const float D = 2;
const float SCREEN_RATIO = 16.0f/9.0f;
const float Kh = Kw*(float)irCameraPayload.W/((float)irCameraPayload.H*SCREEN_RATIO); // K_height is deduced from K_width and the ratio of screen_ratio and sensor_ratio

// Calculate the console payload from the remote payload data
// This function is called each time before sending the console payload to the computer
void calculateCursorState() {
  float x_f = (1 - 1/Kw)/2 + (float)irCameraPayload.x/(Kw*(float)irCameraPayload.W) + (float)tan(remotePayload.yaw/100.f*PI/180.0f)*D/Kw;
  float y_f = (1 - 1/Kh)/2 + (float)irCameraPayload.y/(Kh*(float)irCameraPayload.H) + (float)tan(-remotePayload.pitch/100.f*PI/180.0f)*D/Kh;
  consolePayload.x = (int32_t)(x_f*100*100); // 0.5486 (54.86%) -> 54.86 -> 5486
  consolePayload.y = (int32_t)(y_f*100*100); // 0.5486 (54.86%) -> 54.86 -> 5486
  consolePayload.roll = remotePayload.roll;
  consolePayload.buttonMask = remotePayload.buttonMask;
}

// --- Neopixel ------------------------------------------------

Adafruit_NeoPixel pixels(1, PIN_NEOPIXEL); 

void initLed() {
  pixels.begin(); 
  updateLed();
}

void updateLed() {
  pixels.setPixelColor(0, pixels.Color(
    boardState.computerConnected ? 255 : 0, // Red: computer is connected ?
    boardState.remoteConnected ? 255 : 0, // Green: Remote is connected ?
    boardState.ready ? 255 : 0 // Green: Board is ready ?
  ));
  pixels.show();
}

// --- Computer Connection -------------------------------------

BLEServer* pServer = nullptr;
BLEService* pConsoleService = nullptr;
BLECharacteristic* pConsoleCharacteristic = nullptr;
BLEAdvertising* pConsoleAdvertising = nullptr;

class ComputerCallback: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    boardState.computerConnected = true;
    updateLed();
  }
  
  void onDisconnect(BLEServer* pServer) {
    boardState.computerConnected = false;
    updateLed();
    delay(1000);
    BLEDevice::startAdvertising();
  }
};

void initConsole() {
  // Initialise the bluetooth
  BLEDevice::init("WiiConsole");

  // Create server
  pServer = BLEDevice::createServer();

  // Create console service
  pConsoleService = pServer->createService(CONSOLE_SERVICE_UUID);

  //Create console characteristic
  pConsoleCharacteristic = pConsoleService->createCharacteristic(
    CONSOLE_CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ |
    BLECharacteristic::PROPERTY_WRITE |
    BLECharacteristic::PROPERTY_NOTIFY
  );
  
  // Inform the console characteristic that we have a callback on computer connect/disconnect
  pServer->setCallbacks(new ComputerCallback());  

  // Setup advertising
  pConsoleAdvertising = BLEDevice::getAdvertising();
  pConsoleAdvertising->addServiceUUID(CONSOLE_SERVICE_UUID);
  pConsoleAdvertising->setScanResponse(true);
  pConsoleAdvertising->setMinPreferred(0x06);
  pConsoleAdvertising->setMinPreferred(0x12);  
  
  // Start the service
  pConsoleService->start();
}

void notifyComputer() {
  if (pConsoleCharacteristic != nullptr) {
    consolePayload.roll = (consolePayload.roll + (rand()%3 - 1))%360;
    calculateCursorState();
    pConsoleCharacteristic->setValue((uint8_t*)&consolePayload, sizeof(consolePayload));
    pConsoleCharacteristic->notify();
  }
}

// -------------------------------------------------------------

// --- Remote connection ---------------------------------------

BLEScan* pRemoteScan = nullptr;
BLEClient* pRemoteClient = nullptr;
BLERemoteService* pRemoteService = nullptr;
BLERemoteCharacteristic* pRemoteCharacteristic = nullptr;

void connectToRemote(BLEAddress remoteAddress) {
  pRemoteClient = BLEDevice::createClient();
  pRemoteClient->connect(remoteAddress);
  
  // --- Github (https://github.com/nkolban/ESP32_BLE_Arduino/blob/master/examples/BLE_client/BLE_client.ino)

  // Obtain a reference to the service we are after in the remote BLE server.
  pRemoteService = pRemoteClient->getService(BLEUUID(REMOTE_SERVICE_UUID));
  if (pRemoteService == nullptr) {
    Serial.print("Failed to find our service UUID: ");
    pRemoteClient->disconnect();
    return;
  }
  Serial.println(" - Found our service");


  // Obtain a reference to the characteristic in the service of the remote BLE server.
  pRemoteCharacteristic = pRemoteService->getCharacteristic(BLEUUID(REMOTE_CHARACTERISTIC_UUID));
  if (pRemoteCharacteristic == nullptr) {
    Serial.print("Failed to find our characteristic UUID: ");
    pRemoteClient->disconnect();
    return;
  }
  Serial.println(" - Found our characteristic");

  pRemoteCharacteristic->registerForNotify(remoteNotifyCallback);

  // --- End of Github
}

// Flags for scan
bool doConnect = false;
BLEAddress* pRemoteAddress = nullptr;

class RemoteCallback: public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    Serial.printf("[Console]: Device found\n");
    if (advertisedDevice.haveServiceUUID()) {
      BLEUUID service = advertisedDevice.getServiceUUID();
      // BLEUUID(REMOTE_SERVICE_UUID) builds a BLEUUID class from the uuid string
      if (service.equals(BLEUUID(REMOTE_SERVICE_UUID))) {
        Serial.printf("[Console]: Service uuid match remote. Flagging for connection...\n");
        pRemoteAddress = new BLEAddress(advertisedDevice.getAddress());
        doConnect = true;
        pRemoteScan->stop();
      }
    }
  }
};

void remoteNotifyCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* data, size_t length, bool isNotify) {
  if (length == sizeof(RemotePayload)) {
    memcpy(&remotePayload, data, sizeof(RemotePayload));
  }
}

void initRemote() {
  pRemoteScan = BLEDevice::getScan();
  pRemoteScan->setAdvertisedDeviceCallbacks(new RemoteCallback());
}

void scan() {
  if (pRemoteScan != nullptr) {
    while (pRemoteClient == nullptr) {
      Serial.printf("[Console]: Starting remote scan for 5 seconds...\n");
      pRemoteScan->start(5, false); // false = blocking
      pRemoteScan->stop();
      if (doConnect && pRemoteAddress != nullptr) {
        connectToRemote(*pRemoteAddress);
        Serial.printf("[Console]: Remote connected\n");
      }
    }
    Serial.printf("[Console]: Finished scan\n");
  }
}

void setup() {
  Serial.begin(115200);
  
  delay(5000);
  initLed();
  initConsole();
  BLEDevice::startAdvertising(); // Make the console visible to the computer
  
  boardState.ready = true;
  updateLed();

  initRemote();
  scan();
}

void loop() {

  static unsigned long lastTick = 0;

  if (boardState.computerConnected && (millis() - lastTick > 20)) { // We send data at 50Hz, so every 20ms
    notifyComputer();
    lastTick = millis();
  }
}
