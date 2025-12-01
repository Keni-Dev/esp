#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <ESP32Servo.h>

// ==================== PIN ASSIGNMENTS ====================
// ----- Relay Pins (ACTIVE LOW) -----
#define BLADE_PIN     23   // Blade relay
#define CONVEYOR1_PIN 27   // Conveyor 1 relay  
#define CONVEYOR2_PIN 26   // Conveyor 2 relay

// ----- BTS7960 Motor Driver (Wheels) -----
#define MOTOR_RPWM    25   // Forward PWM
#define MOTOR_LPWM    19   // Backward PWM
#define MOTOR_R_EN    4    // Right Enable
#define MOTOR_L_EN    5    // Left Enable

// ----- Servo (Steering) -----
#define SERVO_PIN     18

// ----- Ultrasonic Sensors (disabled for now) -----
// #define SENSOR1_TRIG 12
// #define SENSOR1_ECHO 14
// #define SENSOR2_TRIG 33
// #define SENSOR2_ECHO 32

// ==================== CONTROLLER MAC ADDRESS ====================
// IMPORTANT: Change this to YOUR controller's MAC address!
// Run WiFi.macAddress() on controller to get it
uint8_t controllerMAC[] = {0x6C, 0xC8, 0x40, 0x45, 0x00, 0xB8};

// ==================== SERVO ====================
Servo steeringServo;
int servoAngle = 90;        // Center position
const int SERVO_MIN = 45;   // Max left
const int SERVO_MAX = 135;  // Max right
const int SERVO_CENTER = 90;

// ==================== DATA STRUCTURES ====================
// Data received from controller
typedef struct {
  int joyX;           // Joystick X (0-4095) - steering
  int joyY;           // Joystick Y (0-4095) - forward/backward
  bool joyPressed;    // Joystick button
  bool conv1;         // Conveyor 1 state
  bool conv2;         // Conveyor 2 state
  bool blade;         // Blade state
  int wheelSpeed;     // Speed: 0, 50, 75, or 100
  float battery;      // Battery voltage
} ControllerData;

// Data sent back to controller (for future use)
typedef struct {
  bool lowBattery;
  bool sackDetected;
  bool sackFull;
} DeviceData;

ControllerData receivedData;
DeviceData dataToSend;

// ==================== STATE VARIABLES ====================
bool bladeState = false;
bool conv1State = false;
bool conv2State = false;
int currentSpeed = 0;

// Connection status
unsigned long lastReceiveTime = 0;
bool isConnected = false;

// ==================== MOTOR CONTROL ====================
void controlWheelMotor(int speedPercent, int joyY) {
  // Convert speed percent to PWM (0-255)
  int pwmValue = map(speedPercent, 0, 100, 0, 255);
  
  // Joystick Y: ~0 = forward, ~4095 = backward, ~2048 = center
  // Adjust these thresholds based on your joystick
  const int DEAD_ZONE_LOW = 1800;   // Below this = forward
  const int DEAD_ZONE_HIGH = 2200;  // Above this = backward
  
  if (joyY < DEAD_ZONE_LOW && speedPercent > 0) {
    // FORWARD
    analogWrite(MOTOR_RPWM, pwmValue);
    analogWrite(MOTOR_LPWM, 0);
  } 
  else if (joyY > DEAD_ZONE_HIGH && speedPercent > 0) {
    // BACKWARD
    analogWrite(MOTOR_RPWM, 0);
    analogWrite(MOTOR_LPWM, pwmValue);
  } 
  else {
    // STOP
    analogWrite(MOTOR_RPWM, 0);
    analogWrite(MOTOR_LPWM, 0);
  }
}

// ==================== SERVO CONTROL ====================
void controlSteering(int joyX) {
  // Joystick X: ~0 = left, ~4095 = right, ~2048 = center
  // Map joystick to servo angle
  servoAngle = map(joyX, 0, 4095, SERVO_MIN, SERVO_MAX);
  servoAngle = constrain(servoAngle, SERVO_MIN, SERVO_MAX);
  steeringServo.write(servoAngle);
}

// ==================== RELAY CONTROL ====================
void controlRelays() {
  // ACTIVE LOW relays: LOW = ON, HIGH = OFF
  digitalWrite(BLADE_PIN, bladeState ? LOW : HIGH);
  digitalWrite(CONVEYOR1_PIN, conv1State ? LOW : HIGH);
  digitalWrite(CONVEYOR2_PIN, conv2State ? LOW : HIGH);
}

// ==================== ESP-NOW CALLBACKS ====================
void OnDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *incomingData, int len) {
  memcpy(&receivedData, incomingData, sizeof(receivedData));
  
  // Update states from controller
  bladeState = receivedData.blade;
  conv1State = receivedData.conv1;
  conv2State = receivedData.conv2;
  currentSpeed = receivedData.wheelSpeed;
  
  // Mark as connected
  lastReceiveTime = millis();
  isConnected = true;
  
  // Debug print
  Serial.print("JoyX:"); Serial.print(receivedData.joyX);
  Serial.print(" JoyY:"); Serial.print(receivedData.joyY);
  Serial.print(" Spd:"); Serial.print(currentSpeed);
  Serial.print("% B:"); Serial.print(bladeState);
  Serial.print(" C1:"); Serial.print(conv1State);
  Serial.print(" C2:"); Serial.println(conv2State);
}

// ==================== SETUP ====================
void setup() {
  Serial.begin(115200);
  Serial.println("\n=== Apparatus Starting ===");
  
  // ----- Initialize Relay Pins (OFF at startup) -----
  pinMode(BLADE_PIN, OUTPUT);
  pinMode(CONVEYOR1_PIN, OUTPUT);
  pinMode(CONVEYOR2_PIN, OUTPUT);
  digitalWrite(BLADE_PIN, HIGH);      // Relay OFF
  digitalWrite(CONVEYOR1_PIN, HIGH);  // Relay OFF
  digitalWrite(CONVEYOR2_PIN, HIGH);  // Relay OFF
  Serial.println("Relays initialized (all OFF)");
  
  // ----- Initialize Motor Driver -----
  pinMode(MOTOR_RPWM, OUTPUT);
  pinMode(MOTOR_LPWM, OUTPUT);
  pinMode(MOTOR_R_EN, OUTPUT);
  pinMode(MOTOR_L_EN, OUTPUT);
  
  // Enable motor driver
  digitalWrite(MOTOR_R_EN, HIGH);
  digitalWrite(MOTOR_L_EN, HIGH);
  
  // Stop motors
  analogWrite(MOTOR_RPWM, 0);
  analogWrite(MOTOR_LPWM, 0);
  Serial.println("Motor driver initialized");
  
  // ----- Initialize Servo -----
  steeringServo.attach(SERVO_PIN);
  steeringServo.write(SERVO_CENTER);
  Serial.println("Servo initialized (center)");
  
  // ----- Initialize ESP-NOW -----
  WiFi.mode(WIFI_STA);
  Serial.print("Apparatus MAC: ");
  Serial.println(WiFi.macAddress());
  
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW Init Failed!");
    while (true) { delay(1000); }
  }
  Serial.println("ESP-NOW initialized");
  
  // Register callbacks
  esp_now_register_recv_cb(OnDataRecv);
  
  // Add controller as peer
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, controllerMAC, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    while (true) { delay(1000); }
  }
  Serial.println("Controller peer added");
  
  // Initialize received data with safe defaults
  receivedData.joyX = 2048;  // Center
  receivedData.joyY = 2048;  // Center
  receivedData.wheelSpeed = 0;
  receivedData.blade = false;
  receivedData.conv1 = false;
  receivedData.conv2 = false;
  
  Serial.println("\n=== Apparatus Ready! ===");
  Serial.println("Waiting for controller...\n");
}

// ==================== MAIN LOOP ====================
void loop() {
  // Check connection timeout (no data for 500ms = disconnected)
  if (millis() - lastReceiveTime > 500) {
    if (isConnected) {
      Serial.println("Controller disconnected!");
      isConnected = false;
    }
    // Safety: stop wheel motor when disconnected
    analogWrite(MOTOR_RPWM, 0);
    analogWrite(MOTOR_LPWM, 0);
    steeringServo.write(SERVO_CENTER);
  }
  
  if (isConnected) {
    // Control steering with joystick X
    controlSteering(receivedData.joyX);
    
    // Control wheel motor with joystick Y and speed setting
    controlWheelMotor(currentSpeed, receivedData.joyY);
    
    // Control relays (blade and conveyors)
    controlRelays();
  }
  
  // Small delay
  delay(20);
}
