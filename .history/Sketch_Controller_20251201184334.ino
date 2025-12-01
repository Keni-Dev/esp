#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// ==================== LCD ====================
LiquidCrystal_I2C lcd(0x27, 16, 2);

// ==================== PIN ASSIGNMENTS ====================
// ----- I2C Pins -----
#define SDA_PIN 21
#define SCL_PIN 22

// ----- LED Pins -----
#define LED_RED     25   // Low battery indicator (future)
#define LED_BLUE    26   // Sack detected indicator (future)
#define LED_YELLOW  27   // Sack full indicator (future)

// ----- Button Pins -----
#define BTN_CONV1    32  // Conveyor 1 toggle
#define BTN_CONV2    33  // Conveyor 2 toggle
#define BTN_BLADE    23  // Blade toggle
#define BTN_SPEED50  19  // Speed 50%
#define BTN_SPEED75  18  // Speed 75%
#define BTN_SPEED100 5   // Speed 100%

// ----- Joystick Pins -----
#define JOY_VRX     34   // X axis (steering)
#define JOY_VRY     35   // Y axis (forward/backward)
#define JOY_SW      14   // Joystick button

// ==================== APPARATUS MAC ADDRESS ====================
// ╔════════════════════════════════════════════════════════════════╗
// ║  HOW TO GET MAC ADDRESS:                                       ║
// ║  1. Upload this code to Controller ESP32                       ║
// ║  2. Open Serial Monitor (115200 baud)                          ║
// ║  3. You will see: "Controller MAC: XX:XX:XX:XX:XX:XX"          ║
// ║  4. Upload Apparatus.ino to the other ESP32                    ║
// ║  5. Open Serial Monitor, you will see "Apparatus MAC: ..."     ║
// ║  6. Copy the Apparatus MAC address and paste it below          ║
// ║                                                                ║
// ║  EXAMPLE: If Apparatus shows MAC: 5C:01:3B:47:3E:24            ║
// ║  Then write: {0x5C, 0x01, 0x3B, 0x47, 0x3E, 0x24}              ║
// ╚════════════════════════════════════════════════════════════════╝
//
// PASTE YOUR APPARATUS MAC ADDRESS HERE (replace the values below):
uint8_t deviceMAC[] = {0x5C, 0x01, 0x3B, 0x47, 0x3E, 0x24};

// ==================== DATA STRUCTURES ====================
typedef struct {
  int joyX;           // Joystick X (0-4095)
  int joyY;           // Joystick Y (0-4095)
  bool joyPressed;    // Joystick button
  bool conv1;         // Conveyor 1 state
  bool conv2;         // Conveyor 2 state
  bool blade;         // Blade state
  int wheelSpeed;     // Speed: 0, 50, 75, or 100
  float battery;      // Battery voltage
} ControllerData;

typedef struct {
  bool lowBattery;
  bool sackDetected;
  bool sackFull;
} DeviceData;

ControllerData dataToSend;
DeviceData dataReceived;

// ==================== TOGGLE STATES ====================
bool bladeState = false;
bool conv1State = false;
bool conv2State = false;
int speedState = 0;  // 0, 50, 75, or 100

// ==================== BUTTON DEBOUNCE ====================
bool lastBladeBtn = HIGH;
bool lastConv1Btn = HIGH;
bool lastConv2Btn = HIGH;
bool lastSpeed50Btn = HIGH;
bool lastSpeed75Btn = HIGH;
bool lastSpeed100Btn = HIGH;

unsigned long lastBladeTime = 0;
unsigned long lastConv1Time = 0;
unsigned long lastConv2Time = 0;
unsigned long lastSpeed50Time = 0;
unsigned long lastSpeed75Time = 0;
unsigned long lastSpeed100Time = 0;
const unsigned long DEBOUNCE_DELAY = 200;

// ==================== LCD MESSAGE ====================
unsigned long lcdMessageTime = 0;
const unsigned long LCD_MESSAGE_DURATION = 1500;  // Show message for 1.5 seconds
bool showingMessage = false;

void showLCDMessage(const char* line1, const char* line2) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(line1);
  lcd.setCursor(0, 1);
  lcd.print(line2);
  lcdMessageTime = millis();
  showingMessage = true;
}

void updateLCDStatus() {
  // Only update if not showing a temporary message
  if (showingMessage && (millis() - lcdMessageTime < LCD_MESSAGE_DURATION)) {
    return;
  }
  showingMessage = false;
  
  // Show current status
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("B:");
  lcd.print(bladeState ? "ON " : "OFF");
  lcd.print(" C1:");
  lcd.print(conv1State ? "ON" : "OF");
  lcd.print(" C2:");
  lcd.print(conv2State ? "ON" : "OF");
  
  lcd.setCursor(0, 1);
  lcd.print("Speed: ");
  if (speedState == 0) {
    lcd.print("0%  ");
  } else {
    lcd.print(speedState);
    lcd.print("%");
  }
}

// ==================== ESP-NOW CALLBACKS ====================
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  // Can use this to show connection status
}

void OnDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *incomingData, int len) {
  memcpy(&dataReceived, incomingData, sizeof(dataReceived));
  
  // Update LEDs based on received data (for future use)
  digitalWrite(LED_RED, dataReceived.lowBattery ? HIGH : LOW);
  digitalWrite(LED_BLUE, dataReceived.sackDetected ? HIGH : LOW);
  digitalWrite(LED_YELLOW, dataReceived.sackFull ? HIGH : LOW);
}

// ==================== SETUP ====================
void setup() {
  Serial.begin(115200);
  Serial.println("\n=== Controller Starting ===");

  // ----- Initialize LED Pins -----
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
  pinMode(LED_YELLOW, OUTPUT);
  digitalWrite(LED_RED, LOW);
  digitalWrite(LED_BLUE, LOW);
  digitalWrite(LED_YELLOW, LOW);

  // ----- Initialize Button Pins (with internal pull-up) -----
  pinMode(BTN_CONV1, INPUT_PULLUP);
  pinMode(BTN_CONV2, INPUT_PULLUP);
  pinMode(BTN_BLADE, INPUT_PULLUP);
  pinMode(BTN_SPEED50, INPUT_PULLUP);
  pinMode(BTN_SPEED75, INPUT_PULLUP);
  pinMode(BTN_SPEED100, INPUT_PULLUP);
  pinMode(JOY_SW, INPUT_PULLUP);

  // ----- Initialize LCD -----
  Wire.begin(SDA_PIN, SCL_PIN);
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Controller");
  lcd.setCursor(0, 1);
  lcd.print("Starting...");

  // ----- Initialize ESP-NOW -----
  WiFi.mode(WIFI_STA);
  
  // Print MAC Address in easy-to-copy format
  Serial.println("\n╔══════════════════════════════════════════════════════════╗");
  Serial.println("║           CONTROLLER ESP32 - MAC ADDRESS                 ║");
  Serial.println("╠══════════════════════════════════════════════════════════╣");
  Serial.print("║  MAC Address: ");
  Serial.print(WiFi.macAddress());
  Serial.println("                       ║");
  Serial.println("║                                                          ║");
  Serial.println("║  Copy this MAC and paste it in Apparatus.ino             ║");
  Serial.println("║  at line: uint8_t controllerMAC[] = {...}                 ║");
  Serial.println("╚══════════════════════════════════════════════════════════╝\n");
  
  // Also print in code format for easy copy-paste
  String mac = WiFi.macAddress();
  Serial.print("Code format: {0x");
  mac.replace(":", ", 0x");
  Serial.print(mac);
  Serial.println("}\n");

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW Init Failed!");
    lcd.clear();
    lcd.print("ESP-NOW FAILED!");
    while (true) { delay(1000); }
  }
  Serial.println("ESP-NOW initialized");

  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);

  // Add apparatus as peer
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, deviceMAC, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    lcd.clear();
    lcd.print("Peer FAILED!");
    while (true) { delay(1000); }
  }
  Serial.println("Apparatus peer added");

  delay(1000);
  lcd.clear();
  lcd.print("Ready!");
  delay(500);
  
  Serial.println("\n=== Controller Ready! ===\n");
}

// ==================== MAIN LOOP ====================
void loop() {
  unsigned long currentTime = millis();
  bool stateChanged = false;

  // ----- Read Joystick -----
  int joyX = analogRead(JOY_VRX);
  int joyY = analogRead(JOY_VRY);
  bool joyPressed = (digitalRead(JOY_SW) == LOW);

  // ----- Handle Blade Button (Toggle) -----
  bool bladeBtn = digitalRead(BTN_BLADE);
  if (bladeBtn == LOW && lastBladeBtn == HIGH && (currentTime - lastBladeTime > DEBOUNCE_DELAY)) {
    bladeState = !bladeState;
    lastBladeTime = currentTime;
    stateChanged = true;
    showLCDMessage("BLADE", bladeState ? ">> ON <<" : ">> OFF <<");
    Serial.println(bladeState ? "Blade: ON" : "Blade: OFF");
  }
  lastBladeBtn = bladeBtn;

  // ----- Handle Conveyor 1 Button (Toggle) -----
  bool conv1Btn = digitalRead(BTN_CONV1);
  if (conv1Btn == LOW && lastConv1Btn == HIGH && (currentTime - lastConv1Time > DEBOUNCE_DELAY)) {
    conv1State = !conv1State;
    lastConv1Time = currentTime;
    stateChanged = true;
    showLCDMessage("CONVEYOR 1", conv1State ? ">> ON <<" : ">> OFF <<");
    Serial.println(conv1State ? "Conv1: ON" : "Conv1: OFF");
  }
  lastConv1Btn = conv1Btn;

  // ----- Handle Conveyor 2 Button (Toggle) -----
  bool conv2Btn = digitalRead(BTN_CONV2);
  if (conv2Btn == LOW && lastConv2Btn == HIGH && (currentTime - lastConv2Time > DEBOUNCE_DELAY)) {
    conv2State = !conv2State;
    lastConv2Time = currentTime;
    stateChanged = true;
    showLCDMessage("CONVEYOR 2", conv2State ? ">> ON <<" : ">> OFF <<");
    Serial.println(conv2State ? "Conv2: ON" : "Conv2: OFF");
  }
  lastConv2Btn = conv2Btn;

  // ----- Handle Speed Buttons (Select, not toggle) -----
  bool speed50Btn = digitalRead(BTN_SPEED50);
  if (speed50Btn == LOW && lastSpeed50Btn == HIGH && (currentTime - lastSpeed50Time > DEBOUNCE_DELAY)) {
    speedState = (speedState == 50) ? 0 : 50;  // Toggle 50% or OFF
    lastSpeed50Time = currentTime;
    stateChanged = true;
    showLCDMessage("WHEEL SPEED", speedState == 50 ? ">> 50% <<" : ">> 0% <<");
    Serial.print("Speed: "); Serial.println(speedState);
  }
  lastSpeed50Btn = speed50Btn;

  bool speed75Btn = digitalRead(BTN_SPEED75);
  if (speed75Btn == LOW && lastSpeed75Btn == HIGH && (currentTime - lastSpeed75Time > DEBOUNCE_DELAY)) {
    speedState = (speedState == 75) ? 0 : 75;  // Toggle 75% or OFF
    lastSpeed75Time = currentTime;
    stateChanged = true;
    showLCDMessage("WHEEL SPEED", speedState == 75 ? ">> 75% <<" : ">> 0% <<");
    Serial.print("Speed: "); Serial.println(speedState);
  }
  lastSpeed75Btn = speed75Btn;

  bool speed100Btn = digitalRead(BTN_SPEED100);
  if (speed100Btn == LOW && lastSpeed100Btn == HIGH && (currentTime - lastSpeed100Time > DEBOUNCE_DELAY)) {
    speedState = (speedState == 100) ? 0 : 100;  // Toggle 100% or OFF
    lastSpeed100Time = currentTime;
    stateChanged = true;
    showLCDMessage("WHEEL SPEED", speedState == 100 ? ">> 100% <<" : ">> 0% <<");
    Serial.print("Speed: "); Serial.println(speedState);
  }
  lastSpeed100Btn = speed100Btn;

  // ----- Prepare Data to Send -----
  dataToSend.joyX = joyX;
  dataToSend.joyY = joyY;
  dataToSend.joyPressed = joyPressed;
  dataToSend.conv1 = conv1State;
  dataToSend.conv2 = conv2State;
  dataToSend.blade = bladeState;
  dataToSend.wheelSpeed = speedState;
  dataToSend.battery = 12.0;  // Placeholder

  // ----- Send Data via ESP-NOW -----
  esp_now_send(deviceMAC, (uint8_t *)&dataToSend, sizeof(dataToSend));

  // ----- Update LCD Status -----
  updateLCDStatus();

  // ----- Debug Output -----
  Serial.print("X:"); Serial.print(joyX);
  Serial.print(" Y:"); Serial.print(joyY);
  Serial.print(" Spd:"); Serial.print(speedState);
  Serial.print(" B:"); Serial.print(bladeState);
  Serial.print(" C1:"); Serial.print(conv1State);
  Serial.print(" C2:"); Serial.println(conv2State);

  delay(50);  // Send data 20 times per second
}
