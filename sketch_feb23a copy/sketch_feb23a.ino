// esp32_websocket_scan.cpp – always emitting frames (distance = -1 when no echo)
// -----------------------------------------------------------------------------
// Libraries: ESP32Servo (John Bennett), WebSockets (Links2004)
// -----------------------------------------------------------------------------
#include <WiFi.h>
#include <WebSocketsServer.h>
#include <ESP32Servo.h>

// ===== Wi‑Fi credentials =====
const char* WIFI_SSID     = "MosheHWN1";
const char* WIFI_PASSWORD = "TowelSponge@";

// ===== Pins =====
static const uint8_t SERVO_PIN = 18;  // PWM
static const uint8_t TRIG_PIN  = 32;
static const uint8_t ECHO_PIN  = 33;

// ===== Scan parameters =====
static const uint16_t STEP_DEG         = 5;
static const uint32_t STEP_INTERVAL_MS = 300;
static const uint16_t PULSE_TIMEOUT_US = 25'000; // 4 m

WebSocketsServer ws(8080);
Servo servo;

uint8_t  curAngle = 0;
bool     forward  = true;
uint32_t lastStep = 0;

// -----------------------------------------------------------------------------
float measureCM() {
  digitalWrite(TRIG_PIN, LOW); delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long dur = pulseIn(ECHO_PIN, HIGH, PULSE_TIMEOUT_US);
  if (dur == 0) return -1.0f;                 // timeout → no echo
  return dur * 0.0343f / 2.0f;                // cm
}

void sendFrame(uint8_t angle, float distance) {
  String msg = "{\"angle\":" + String(angle) + ",\"distance\":" + String(distance, 1) + "}";
  ws.broadcastTXT(msg);
}

void onWs(uint8_t num, WStype_t type, uint8_t*, size_t) {
  if (type == WStype_CONNECTED) {
    Serial.printf("[WS] Client %u connected\n", num);
  }
}

void connectWiFi() {
  Serial.printf("[WiFi] Connecting to %s", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print('.'); }
  Serial.printf("\n[WiFi] %s\n", WiFi.localIP().toString().c_str());
}

void setup() {
  Serial.begin(115200);
  pinMode(TRIG_PIN, OUTPUT); pinMode(ECHO_PIN, INPUT);
  servo.attach(SERVO_PIN);
  connectWiFi();
  ws.begin(); ws.onEvent(onWs);
  servo.write(curAngle);
  lastStep = millis();
}

void loop() {
  ws.loop(); yield();
  uint32_t now = millis();
  if (now - lastStep >= STEP_INTERVAL_MS) {
    // update angle
    if (forward) {
      curAngle += STEP_DEG; if (curAngle >= 180) { curAngle = 180; forward = false; }
    } else {
      if (curAngle >= STEP_DEG) curAngle -= STEP_DEG; else curAngle = 0;
      if (curAngle == 0) forward = true;
    }
    servo.write(curAngle);

    // measure distance (may be -1)
    float d = measureCM();
    sendFrame(curAngle, d);    // ALWAYS send, even when d == -1

    lastStep = now;
  }
}
