#include <Adafruit_NeoPixel.h>
#include <Wire.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "Adafruit_BNO08x_RVC.h" // For using gyroscope
#include <cstdlib> // For using rand()

#define REMOTE_SERVICE_UUID "c0990295-2b98-4df2-9338-6caf6bd6acce"
#define REMOTE_CHARACTERISTIC_UUID "7bf3223d-0687-479f-80b0-5b4a4c6df558"

#define LED_DISTANCE_MM 200.0f
#define NOTIFY_INTERVAL_MS 10 // 10ms interval -> 100Hz

#define BUTTON_A_PIN 7
#define BUTTON_B_PIN 9

// === Mathematics types =====================================================

typedef float vec3[3];
typedef float mat3[3][3];

bool PRINT_FLAG = false;

// === Board state =============================================================

struct BoardState {
  bool computerConnected = false;
  vec3 pos = { 0,0,0 };
  vec3 ray = { 0,0,0 };
  float pitch = 0;
} boardState;


// === Payloads =============================================================

struct GyroscopePayload {
  float pitch = 0;
  float roll = 0;
  float yaw = 0;
} gyroscopePayload;

struct GyroscopeOffset {
  float pitch = 0;
  float roll  = 0;
  float yaw   = 0;
} gyroscopeOffset;

struct IRCameraPayload {
  int x1 = 0;
  int y1 = 0;
  int x2 = 0;
  int y2 = 0;
  // Constants (Pixel resolution and angles W/H switched due to the IRL position orientation the camera)
  float H = 1024; // Width (in pixels) of the IR Camera (cf. documentation)
  float W = 768; // Height (in pixels) of the IR Camera (cf. documentation)
  float fov_h = 33.0f*3.14159/180.0f;
  float fov_w = 23.0f*3.14159/180.0f;
} irCameraPayload;

struct CursorPayload {
  int32_t x = 0 * 100;
  int32_t y = 0 * 100;
  int32_t roll = 0 * 100;
  uint8_t buttonMask = 0; // lowest bit (1) = button A, second bit (2) = button B 
  uint8_t sensorMask = 0; //0b[0][0][0][0][0][valid computation values ?][valid IR camera measurement ?][valid gyroscope measurement ?]
} cursorPayload;

// === Mathematics =============================================================

void mat3_mul_vec3(const mat3 M, const vec3 v, vec3 out) {
  for (int i = 0; i < 3; i++) {
    out[i] = M[i][0]*v[0] + M[i][1]*v[1] + M[i][2]*v[2];
  }
}

float dot3(const vec3 a, const vec3 b) {
  return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}

float norm2_3(const vec3 a) {
  return dot3(a, a);
}

void mat3_mul_mat3(const mat3 A, const mat3 B, mat3 out) { // A*B
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      out[i][j] = A[i][0]*B[0][j] + A[i][1]*B[1][j] + A[i][2]*B[2][j];
    }
  }
}

// Builds the rotation matrix following yaw->pitch->roll convention
void build_rotation(float psi, float theta, float phi, mat3 R) {
  float cp = cosf(phi),   sp = sinf(phi);
  float ct = cosf(theta), st = sinf(theta);
  float cy = cosf(psi),   sy = sinf(psi);

  R[0][0] =  cy*ct;
  R[0][1] =  cy*st*sp - sy*cp;
  R[0][2] =  cy*st*cp + sy*sp;

  R[1][0] =  sy*ct;
  R[1][1] =  sy*st*sp + cy*cp;
  R[1][2] =  sy*st*cp - cy*sp;

  R[2][0] = -st;
  R[2][1] =  ct*sp;
  R[2][2] =  ct*cp;
}

void mat3T_mul_vec3(const mat3 R, const vec3 v, vec3 out) {
  for (int i = 0; i < 3; i++) {
    out[i] = R[0][i]*v[0] + R[1][i]*v[1] + R[2][i]*v[2];
  }
}

void pixel_to_world_ray(
  float u_raw, float v_raw,
  float fx, float fy, float cx, float cy,
  const mat3 R,
  vec3 r_out)
{
  vec3 d = {
    (u_raw - cx) / fx, // eq.(12)
    -1.0f,
    (v_raw - cy) / fy
  };
  mat3T_mul_vec3(R, d, r_out); // eq.(16)
}

bool localise_remote(
  float psi, float theta, float phi, // Euler angles (rad) (yaw/pitch/roll)
  float u1, float v1, float u2, float v2, // raw camera pixels values
  float d, // distance between two leds
  const IRCameraPayload& ir,
  vec3 P_out,
  vec3 ray_out
  )
{  
  // camera intrinsics
  float fx = ir.W / (2.0f * tanf(ir.fov_w / 2.0f));
  float fy = ir.H / (2.0f * tanf(ir.fov_h / 2.0f));
  float cx = ir.W / 2.0f, cy = ir.H / 2.0f;

  // rotation matrix
  mat3 R;
  build_rotation(psi, theta, phi, R);

  // world-space rays r1, r2
  vec3 r1, r2;
  pixel_to_world_ray(u1, v1, fx, fy, cx, cy, R, r1);
  pixel_to_world_ray(u2, v2, fx, fy, cx, cy, R, r2);

  if (PRINT_FLAG) {
    Serial.printf("Ray1 in O space: (%f, %f, %f)\n", r1[0], r1[1], r1[2]);
    Serial.printf("Ray2 in O space: (%f, %f, %f)\n", r2[0], r2[1], r2[2]);
  }

  // closed-form solve, eq.(26)
  float alpha = norm2_3(r1); //  |r1|^2
  float beta  = dot3(r1, r2); // r1.r2
  float gamma = norm2_3(r2); //  |r2|^2
  float delta = alpha*gamma - beta*beta; // det(A.T * A)

  if (fabsf(delta) < 1e-9f) return false; // degenerate: rays are parallel

  float lambda1 = (2.0f*d / delta) * (gamma*r1[0] - beta*r2[0]); // eq.(26)
  float lambda2 = (2.0f*d / delta) * (beta*r1[0] - alpha*r2[0]);

  if (PRINT_FLAG) {
    Serial.printf("lambda1: %f | lambda2: %f\n", lambda1, lambda2);
  }

  if (lambda1 < 0 || lambda2 < 0) return false; // leds behind camera (impossible)

  // recover P, eq.(27-29)
  // P1 = L1 - λ1.r1,  P2 = L2 - λ2.r2,  P = (P1+P2)/2
  vec3 L1 = { d, 0, 0}, L2 = {-d, 0, 0};
  for (int i = 0; i < 3; i++) {
    P_out[i] = 0.5f * ((L1[i] - lambda1*r1[i]) + (L2[i] - lambda2*r2[i]));
  }

  // calculate the ray vector
  mat3_mul_vec3(R, vec3{0,-1,0}, ray_out);

  if (PRINT_FLAG) {
    vec3 test = {0, 0, 0};
    mat3_mul_vec3(R, vec3{1,0,0}, test);
    Serial.printf("X axis: O -> R : (%f, %f, %f)\n", test[0], test[1], test[2]);
    mat3_mul_vec3(R, vec3{0,1,0}, test);
    Serial.printf("Y axis: O -> R : (%f, %f, %f)\n", test[0], test[1], test[2]);
    mat3_mul_vec3(R, vec3{0,0,1}, test);
    Serial.printf("Z axis: O -> R : (%f, %f, %f)\n", test[0], test[1], test[2]);
  }

  return true;
}

// === Neopixel =============================================================

Adafruit_NeoPixel pixels(1, PIN_NEOPIXEL); 

void setupLED() {
  pixels.begin(); 
  updateLed();
}

void updateLed() {
  pixels.setPixelColor(0, pixels.Color(
    boardState.computerConnected ? 255 : 0, // Red + Green: computer is connected ?
    boardState.computerConnected ? 255 : 0,
    255
  ));
  pixels.show();
}

void turnLedOff() {
  pixels.setPixelColor(0, pixels.Color(0,0,0));
  pixels.show();
}

// === Buttons =============================================================

void setupButtons() {
  pinMode(BUTTON_A_PIN, INPUT);
  pinMode(BUTTON_B_PIN, INPUT);  // reset button
}

// === IR Camera =======================================================

int IRsensorAddress = 0xB0;
//int IRsensorAddress = 0x58;
int slaveAddress = IRsensorAddress >> 1;
byte data_buf[16];

int Ix[4];
int Iy[4];
int s;

void Write_2bytes(byte d1, byte d2) {
  Wire.beginTransmission(slaveAddress);
  Wire.write(d1); Wire.write(d2);
  Wire.endTransmission();
}

void setupIRCamera() {
  slaveAddress = IRsensorAddress >> 1; // 0xB0 >> 1 = 0x58
  Wire.begin();
  // IR sensor initialize
  Write_2bytes(0x30,0x01); delay(10);
  Write_2bytes(0x30,0x08); delay(10);
  Write_2bytes(0x06,0x90); delay(10);
  Write_2bytes(0x08,0xC0); delay(10);
  Write_2bytes(0x1A,0x40); delay(10);
  Write_2bytes(0x33,0x33); delay(10); 
}

bool readIRCamera(IRCameraPayload& out) {
  //IR sensor read
  Wire.beginTransmission(slaveAddress);
  Wire.write(0x36);
  Wire.endTransmission();
  Wire.requestFrom(slaveAddress, 16); // Request the 2 byte heading (MSB comes first)
  for (int i=0;i<16;i++) { data_buf[i]=0; }
  int i=0;
  while(Wire.available() && i < 16) {
    data_buf[i] = Wire.read();
    i++;
  }
  Ix[0] = data_buf[1];
  Iy[0] = data_buf[2];
  s = data_buf[3];
  Ix[0] += (s & 0x30) <<4;
  Iy[0] += (s & 0xC0) <<2;
  Ix[1] = data_buf[4];
  Iy[1] = data_buf[5];
  s = data_buf[6];
  Ix[1] += (s & 0x30) <<4;
  Iy[1] += (s & 0xC0) <<2;
  Ix[2] = data_buf[7];
  Iy[2] = data_buf[8];
  s = data_buf[9];
  Ix[2] += (s & 0x30) <<4;
  Iy[2] += (s & 0xC0) <<2;
  Ix[3] = data_buf[10];
  Iy[3] = data_buf[11];
  s = data_buf[12];
  Ix[3] += (s & 0x30) <<4;
  Iy[3] += (s & 0xC0) <<2;
  // for(i=0; i<4; i++)
  // {
  //   if (Ix[i] < 1000)
  //     Serial.print("");
  //   if (Ix[i] < 100)
  //     Serial.print("");
  //   if (Ix[i] < 10)
  //     Serial.print("");
    
  //   Serial.print( int(Ix[i]) );
  //   Serial.print(",");
  //   if (Iy[i] < 1000)
  //     Serial.print("");
  //   if (Iy[i] < 100)
  //     Serial.print("");
  //   if (Iy[i] < 10)
  //     Serial.print("");
    
  //   Serial.print( int(Iy[i]) );
  //   if (i<3)
  //     Serial.print(",");
  // }
  // Serial.println("");
  if (Iy[0] == 1023 || Ix[0] == 1023 || Iy[1] == 1023 || Ix[1] == 1023) return false;
  
  int smallest_x = Iy[0] < Iy[1] ? 0 : 1;
  out.x1 = Iy[1-smallest_x];
  out.y1 = Ix[1-smallest_x];
  out.x2 = Iy[smallest_x];
  out.y2 = Ix[smallest_x];

  return true;
}

// === Gyroscope (BNO085x) =============================================================

#define BNO_RX 4  // ESP32-S3 pin connected to BNO08x TX
#define BNO_TX 5  // ESP32-S3 pin connected to BNO08x RX (optional, RVC is one direction only)

Adafruit_BNO08x_RVC rvc = Adafruit_BNO08x_RVC();

void setupGyroscope() {
    // Wait for serial monitor to open
    // while (!Serial)
    //     delay(10);

    Serial.println("Adafruit BNO08x IMU - UART-RVC mode");

    Serial1.begin(115200, SERIAL_8N1, BNO_RX, BNO_TX); // Define the pins we are using with the sensor. 115200 is the baud rate specified by the datasheet
    while (!Serial1)
        delay(10);


    if (!rvc.begin(&Serial1)) { // connect to the sensor over hardware serial
        Serial.println("Could not find BNO08x!");
        while (1)
            delay(10);
    }

    Serial.println("BNO08x found!");
}

// Calculate the remote payload from the sensor data
// This function is called each time before sending the remote payload to the console
bool readGyroscope(GyroscopePayload& out) {
  BNO08x_RVC_Data heading;
  
  // To understand why this double-check: https://claude.ai/share/e2369bef-e1bb-41e0-90b9-f1987f1cefd1
  if (!rvc.read(&heading)) { // If the sensor is still transmitting data to the ESP32 buffer, we may have to wait for the end of the transmission.
    delay(3); // wait for frame to finish transmitting (~1.65ms)
    if (!rvc.read(&heading)) return false; // Here, we are almost sure that the buffer is just empty/not at the right format.
  }

  out.pitch = heading.pitch - gyroscopeOffset.pitch; // Rotation around the 2 white connectors
  out.yaw = heading.yaw - gyroscopeOffset.yaw; // Rotation around sensor plane normal (horizontal)
  out.roll = heading.roll - gyroscopeOffset.roll; // Rotation around the 2 pin edges
  // Serial.printf("Gyro: %f, %f, %f\n", heading.pitch, heading.yaw, heading.roll);
  return true;
}

// Captures the current orientation as the new zero reference
void resetGyroscope(const GyroscopePayload& current, GyroscopeOffset& offset) {
  offset.pitch += current.pitch; // raw = offset + current, so new offset = raw
  offset.roll  += current.roll;
  offset.yaw   += current.yaw;
  Serial.println("[Gyro]: Origin reset.");
}


// === Bluetooth ====================================================

BLEServer* pServer = nullptr;
BLEService* pRemoteService = nullptr;
BLECharacteristic* pRemoteCharacteristic = nullptr;
BLEAdvertising* pRemoteAdvertising = nullptr;

class ComputerCallback: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    Serial.printf("[Remote]: Computer connected\n");
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

void setupRemoteBluetooth() {
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
  
  // Inform the remote characteristic that we have a callback on computer connect/disconnect
  pServer->setCallbacks(new ComputerCallback());  

  // Setup advertising
  pRemoteAdvertising = BLEDevice::getAdvertising();
  pRemoteAdvertising->addServiceUUID(REMOTE_SERVICE_UUID);
  pRemoteAdvertising->setScanResponse(true);
  pRemoteAdvertising->setMinPreferred(0x06);
  pRemoteAdvertising->setMinPreferred(0x12);  
  
  // Start the service
  pRemoteService->start();
}

// === Cursor calculations =================================================

bool updateBoardState(
  const GyroscopePayload& gyro,
  const IRCameraPayload& ir,
  BoardState& state)
{
  if (!localise_remote(
    gyro.yaw*3.1415/180.0f, gyro.pitch*3.1415/180.0f, gyro.roll*3.1415/180.0f,
    ir.x1, ir.y1, ir.x2, ir.y2,
    LED_DISTANCE_MM/2,
    irCameraPayload,
    state.pos,
    state.ray
  )) return false;

  state.pitch = gyro.pitch;

  return true;
}

void assembleCursorPayload(const BoardState& state, const uint8_t sensorMask, CursorPayload& out) {
  float t = -state.pos[1] / state.ray[1];
  out.x = (int32_t)((state.pos[0] + t * state.ray[0]) * 100);
  out.y = (int32_t)((state.pos[2] + t * state.ray[2]) * 100);
  out.roll = (int32_t)(state.pitch * 100);

  uint8_t mask = 0;
  if (digitalRead(BUTTON_A_PIN)) mask |= (1 << 0);
  out.buttonMask = mask;

  out.sensorMask = sensorMask;

}

void notifyComputer() {
  if (pRemoteCharacteristic != nullptr) {
    pRemoteCharacteristic->setValue((uint8_t*)&cursorPayload, sizeof(cursorPayload));
    pRemoteCharacteristic->notify();
    // Serial.printf("[Remote]: Notified console\n");
  }
}

// === Orchestration =============================================

int did_init = 0;

void setup() {
  Serial.begin(115200);
  delay(5000);

  // Setup different components
  setupLED();
  setupButtons();
  setupIRCamera();
  setupGyroscope();
  setupRemoteBluetooth();

  BLEDevice::startAdvertising(); // Make the remote visible to the console
  updateLed();
}

bool MISS_FLAG = false;

void loop() {
  static unsigned long lastTick = 0;
  static unsigned long lastPrint = 0;

  if ((millis() - lastTick > NOTIFY_INTERVAL_MS)) { // (boardState.computerConnected && ) We send data at a defined frequency
    if (PRINT_FLAG) {
      Serial.printf("=== Missed cursor calculation ===.\n");
      PRINT_FLAG = 0;
    }
    lastTick = millis();

    if (MISS_FLAG) {
      turnLedOff();
    } else {
      updateLed();
    }
    MISS_FLAG = true;


    if (millis() - lastPrint > 1000) {
      lastPrint = millis();
      PRINT_FLAG = true;
      Serial.printf("======== DEBUG ========\n");
    }

    if(!readGyroscope(gyroscopePayload)) return;
    
    // Reset gyroscope origin when button B is pressed
    if (digitalRead(BUTTON_B_PIN) || did_init < 10) {
      did_init++;
      resetGyroscope(gyroscopePayload, gyroscopeOffset);
    }
    
    if(!readIRCamera(irCameraPayload)) return; // not ready yet (TODO: completer la fonction)

    if (!updateBoardState(gyroscopePayload, irCameraPayload, boardState)) return;

    if (PRINT_FLAG) {
      Serial.printf("Gyro: pitch %f,  roll %f, yaw %f\n", gyroscopePayload.pitch, gyroscopePayload.roll, gyroscopePayload.yaw);
      Serial.printf("Camera: (%d, %d), (%d, %d)\n", irCameraPayload.x1, irCameraPayload.y1, irCameraPayload.x2, irCameraPayload.y2);
      Serial.printf("Pos: %f, %f, %f\n", boardState.pos[0], boardState.pos[1], boardState.pos[2]);
      Serial.printf("Ray: %f, %f, %f\n", boardState.ray[0], boardState.ray[1], boardState.ray[2]);
    }

    assembleCursorPayload(boardState, 0b00000111, cursorPayload);
    notifyComputer();

    PRINT_FLAG = false;
    MISS_FLAG = false;
  }
}
