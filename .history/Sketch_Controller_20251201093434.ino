#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// ----- LCD -----
LiquidCrystal_I2C lcd(0x27, 16, 2);

// ----- I2C Pins -----
#define SDA_PIN 21
#define SCL_PIN 22

// ----- Pin Assignments -----
#define RED_LED     25
#define BLUE_LED    26
#define YELLOW_LED  27

#define BTN_CONV1   32
#define BTN_CONV2   33
#define BTN_BLADE   23

#define BTN_SPEED50 19
#define BTN_SPEED75 18
#define BTN_SPEED100 5

#define JOY_VRX     34
#define JOY_VRY     35
#define JOY_SW      14

// ----- Device MAC Address -----
uint8_t deviceMAC[] = {0x5C, 0x01, 0x3B, 0x47, 0x3E, 0x24};

// ----- Data Structures -----
typedef struct {
  int joyX;
  int joyY;
  bool joyPressed;
  bool conv1;
  bool conv2;
  bool blade;
  int wheelSpeed;
  float battery;
} ControllerData;

typedef struct {
  int palayCount;
  bool sackFull;
  bool sackEmpty;
} DeviceData;

ControllerData dataToSend;
DeviceData dataReceived;

// ----- Button States -----
bool bladeState = false;
bool conv1State = false;
bool conv2State = false;

bool lastBladeBtn = HIGH;
bool lastConv1Btn = HIGH;
bool lastConv2Btn = HIGH;

// Debounce timing
unsigned long lastBladeTime = 0;
unsigned long lastConv1Time = 0;
unsigned long lastConv2Time = 0;
const unsigned long debounceDelay = 200; // 200ms debounce

// ----- Send Callback (fixed for ESP32 v5+) -----
void OnDataSent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status) {
  Serial.print("Send Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "✅ Success" : "❌ Fail");
}

// ----- Receive Callback (fixed for ESP32 v5+) -----
void OnDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *incomingData, int len) {
  memcpy(&dataReceived, incomingData, sizeof(dataReceived));

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("SackFull:");
  lcd.print(dataReceived.sackFull ? "YES" : "NO ");
  lcd.setCursor(0, 1);
  lcd.print("Empty:");
  lcd.print(dataReceived.sackEmpty ? "YES" : "NO ");
}

// ----- Battery Placeholder -----
float readBatteryVoltage() {
  return 12.0;
}

// ----- Setup -----
void setup() {
  Serial.begin(115200);

  pinMode(RED_LED, OUTPUT);
  pinMode(BLUE_LED, OUTPUT);
  pinMode(YELLOW_LED, OUTPUT);

  pinMode(BTN_CONV1, INPUT_PULLUP);
  pinMode(BTN_CONV2, INPUT_PULLUP);
  pinMode(BTN_BLADE, INPUT_PULLUP);
  pinMode(BTN_SPEED50, INPUT_PULLUP);
  pinMode(BTN_SPEED75, INPUT_PULLUP);
  pinMode(BTN_SPEED100, INPUT_PULLUP);

  pinMode(JOY_SW, INPUT_PULLUP);

  // Initialize I2C with explicit pins
  Wire.begin(SDA_PIN, SCL_PIN);
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.print("Controller Ready");

  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("❌ ESP-NOW Init Failed!");
    while (true) delay(1000);
  }

  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, deviceMAC, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("❌ Failed to add peer");
    while (true) delay(1000);
  }

  Serial.println("📡 Controller Ready");
}

// ----- Loop -----
void loop() {
  int joyX = analogRead(JOY_VRX);
  int joyY = analogRead(JOY_VRY);
  bool joyPressed = (digitalRead(JOY_SW) == LOW);

  // Toggle blade
  bool bladeBtn = digitalRead(BTN_BLADE);
  if (bladeBtn == LOW && lastBladeBtn == HIGH) bladeState = !bladeState;
  lastBladeBtn = bladeBtn;

  // Toggle conveyors
  bool conv1Btn = digitalRead(BTN_CONV1);
  if (conv1Btn == LOW && lastConv1Btn == HIGH) conv1State = !conv1State;
  lastConv1Btn = conv1Btn;

  bool conv2Btn = digitalRead(BTN_CONV2);
  if (conv2Btn == LOW && lastConv2Btn == HIGH) conv2State = !conv2State;
  lastConv2Btn = conv2Btn;

  // Speed levels
  int wheelSpeed = 0;
  if (digitalRead(BTN_SPEED50) == LOW) wheelSpeed = 50;
  if (digitalRead(BTN_SPEED75) == LOW) wheelSpeed = 75;
  if (digitalRead(BTN_SPEED100) == LOW) wheelSpeed = 100;

  dataToSend.joyX = joyX;
  dataToSend.joyY = joyY;
  dataToSend.joyPressed = joyPressed;
  dataToSend.conv1 = conv1State;
  dataToSend.conv2 = conv2State;
  dataToSend.blade = bladeState;
  dataToSend.wheelSpeed = wheelSpeed;
  dataToSend.battery = readBatteryVoltage();

  esp_now_send(deviceMAC, (uint8_t *)&dataToSend, sizeof(dataToSend));

  Serial.print("JoyX: "); Serial.print(joyX);
  Serial.print(" | JoyY: "); Serial.print(joyY);
  Serial.print(" | Speed: "); Serial.print(wheelSpeed);
  Serial.print(" | C1: "); Serial.print(conv1State);
  Serial.print(" | C2: "); Serial.print(conv2State);
  Serial.print(" | Blade: "); Serial.println(bladeState);

  delay(100);
}