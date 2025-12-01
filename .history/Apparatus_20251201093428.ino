#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <ESP32Servo.h>

// ----- Pin Assignments -----
#define SENSOR1_TRIG 12
#define SENSOR1_ECHO 14
#define SENSOR2_TRIG 33
#define SENSOR2_ECHO 32

#define CONVEYOR1_PIN 26
#define CONVEYOR2_PIN 27
#define BLADE_PIN     23

#define MOTOR_LPWM_PIN 25  // Forward
#define MOTOR_RPWM_PIN 19  // Backward
#define MOTOR_L_EN     4   // Left Enable
#define MOTOR_R_EN     5   // Right Enable

#define SERVO_PIN      18  // Steering

// ----- Controller MAC -----
uint8_t controllerMAC[] = {0x6C, 0xC8, 0x40, 0x45, 0x00, 0xB8};

// ----- Servo Object -----
Servo steeringServo;

// ----- Data Structures -----
typedef struct {
  int joyX;
  int joyY;
  bool joyPressed;
  bool conv1;
  bool conv2;
  bool blade;
  int wheelSpeed; // 0-100
  float battery;
} ControllerData;

typedef struct {
  int palayCount;
  bool sackFull;
  bool sackEmpty;
} DeviceData;

ControllerData receivedData;
DeviceData dataToSend;

// ----- Variables -----
int servoAngle = 90; // current servo angle
const int servoMin = 45;
const int servoMax = 135;

// States received from controller (no need to toggle here!)
bool bladeState = false;
bool conv1State = false;
bool conv2State = false;
int wheelSpeedState = 0;

// ----- Motor Control -----
void setMotorSpeed(int speedPercent, int joyY) {
  int pwmVal = map(speedPercent, 0, 100, 0, 255);

  if (joyY < 1800) {          // Forward
    analogWrite(MOTOR_LPWM_PIN, pwmVal);
    analogWrite(MOTOR_RPWM_PIN, 0);
  } else if (joyY > 3000) {   // Backward
    analogWrite(MOTOR_LPWM_PIN, 0);
    analogWrite(MOTOR_RPWM_PIN, pwmVal);
  } else {                    // Stop
    analogWrite(MOTOR_LPWM_PIN, 0);
    analogWrite(MOTOR_RPWM_PIN, 0);
  }
}

// ----- Ultrasonic -----
long readUltrasonicCM(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, 20000);
  long distance = duration * 0.034 / 2;
  return distance;
}

// ----- ESP-NOW Receive Callback -----
void OnDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *incomingData, int len) {
  memcpy(&receivedData, incomingData, sizeof(receivedData));

  // IMPORTANT: Controller already toggles the states!
  // Just copy the states directly - NO TOGGLE HERE!
  bladeState = receivedData.blade;
  conv1State = receivedData.conv1;
  conv2State = receivedData.conv2;
  wheelSpeedState = receivedData.wheelSpeed;

  Serial.print("Received -> Blade: "); Serial.print(bladeState);
  Serial.print(" | Conv1: "); Serial.print(conv1State);
  Serial.print(" | Conv2: "); Serial.print(conv2State);
  Serial.print(" | Speed: "); Serial.print(wheelSpeedState);
  Serial.print(" | JoyX: "); Serial.print(receivedData.joyX);
  Serial.print(" | JoyY: "); Serial.println(receivedData.joyY);
}

// ----- Send Data to Controller -----
void sendDeviceData() {
  long dist1 = readUltrasonicCM(SENSOR1_TRIG, SENSOR1_ECHO);
  long dist2 = readUltrasonicCM(SENSOR2_TRIG, SENSOR2_ECHO);

  dataToSend.palayCount = dist1 < 20 ? 1 : 0;
  dataToSend.sackFull = dist1 < 20 && dist2 < 20;
  dataToSend.sackEmpty = dist2 > 50;

  esp_now_send(controllerMAC, (uint8_t *)&dataToSend, sizeof(dataToSend));
}

// ----- Setup -----
void setup() {
  Serial.begin(115200);

  // Pins
  pinMode(CONVEYOR1_PIN, OUTPUT);
  pinMode(CONVEYOR2_PIN, OUTPUT);
  pinMode(BLADE_PIN, OUTPUT);

  pinMode(SENSOR1_TRIG, OUTPUT);
  pinMode(SENSOR1_ECHO, INPUT);
  pinMode(SENSOR2_TRIG, OUTPUT);
  pinMode(SENSOR2_ECHO, INPUT);

  // Motor pins
  pinMode(MOTOR_LPWM_PIN, OUTPUT);
  pinMode(MOTOR_RPWM_PIN, OUTPUT);
  pinMode(MOTOR_L_EN, OUTPUT);
  pinMode(MOTOR_R_EN, OUTPUT);
  digitalWrite(MOTOR_L_EN, HIGH);
  digitalWrite(MOTOR_R_EN, HIGH);

  // Servo
  steeringServo.attach(SERVO_PIN);
  steeringServo.write(servoAngle);

  // Wi-Fi and ESP-NOW
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("❌ ESP-NOW Init Failed!");
    while (true) delay(1000);
  }

  esp_now_register_recv_cb(OnDataRecv);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, controllerMAC, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("❌ Failed to add peer");
    while (true) delay(1000);
  }

  Serial.println("✅ Device Ready and Connected");
}

// ----- Loop -----
void loop() {
  // ---- Servo Control (Joystick X for steering) ----
  if (receivedData.joyX < 1500) {
    servoAngle -= 2; // turn left
  } else if (receivedData.joyX > 2500) {
    servoAngle += 2; // turn right
  }
  // Constrain servo angle
  servoAngle = constrain(servoAngle, servoMin, servoMax);
  steeringServo.write(servoAngle);

  // ---- Motor Control (Joystick Y + Speed) ----
  setMotorSpeed(wheelSpeedState, receivedData.joyY);

  // ---- Conveyor & Blade Control ----
  digitalWrite(CONVEYOR1_PIN, conv1State ? HIGH : LOW);
  digitalWrite(CONVEYOR2_PIN, conv2State ? HIGH : LOW);
  digitalWrite(BLADE_PIN, bladeState ? HIGH : LOW);

  // ---- Send sensor data back to controller ----
  sendDeviceData();

  // Debug output
  Serial.print("Servo: "); Serial.print(servoAngle);
  Serial.print(" | Motor Speed: "); Serial.print(wheelSpeedState);
  Serial.print(" | Conv1: "); Serial.print(conv1State);
  Serial.print(" | Conv2: "); Serial.print(conv2State);
  Serial.print(" | Blade: "); Serial.println(bladeState);

  delay(50);
}