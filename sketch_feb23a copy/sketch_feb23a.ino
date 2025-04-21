// esp32_websocket_scan.cpp
// ESP32 ultrasonic scanner + WebSocket broadcaster (Servo control with ESP32Servo)
// -----------------------------------------------------------------------------
// Libraries required (Arduino Library Manager):
//   • ESP32Servo (by John Bennett)
//   • WebSockets by Markus Sattler (Links2004)
// -----------------------------------------------------------------------------
// Compatible with ESP32 Arduino Core 3.x

#include <WiFi.h>
#include <WebSocketsServer.h>
#include <ESP32Servo.h>

// ======= USER CONFIG (kept as requested) =======
const char* WIFI_SSID     = "MosheHWN1";
const char* WIFI_PASSWORD = "TowelSponge@";

// ======= GPIO PINS (kept as requested) =======
static const uint8_t SERVO_PIN = 18;   // PWM‑capable pin for SG90
static const uint8_t TRIG_PIN  = 32;   // HC‑SR04 TRIG
static const uint8_t ECHO_PIN  = 33;   // HC‑SR04 ECHO

// ======= Scanner Behaviour =======
static const uint16_t STEP_DEG          = 5;          // servo increment (°)
static const uint32_t STEP_INTERVAL_MS  = 300;        // time between steps (ms)
static const uint16_t PULSE_TIMEOUT_US  = 25'000;     // echo timeout (µs) ≈4 m

// ======= Globals =======
WebSocketsServer webSocket(8080);
Servo myServo;

uint8_t  currentAngle = 0;
bool     sweepForward = true;
uint32_t lastStepTime = 0;

// -----------------------------------------------------------------------------
// Measure distance (cm) with HC‑SR04 (returns −1 when timed‑out)
// -----------------------------------------------------------------------------
float measureDistanceCM() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, PULSE_TIMEOUT_US);
  if (duration == 0) return -1.0f;                     // no echo
  return duration * 0.0343f / 2.0f;                    // cm
}

// -----------------------------------------------------------------------------
// WebSocket helpers
// -----------------------------------------------------------------------------
void sendFrame(uint8_t angle, float distance) {
  String msg = "{\"angle\":" + String(angle) + ",\"distance\":" + String(distance, 1) + "}";
  webSocket.broadcastTXT(msg);
}

void onWsEvent(uint8_t client, WStype_t type, uint8_t *payload, size_t len) {
  switch (type) {
    case WStype_CONNECTED:
      Serial.printf("[WS] Client %u connected: %s\n", client, webSocket.remoteIP(client).toString().c_str());
      break;
    case WStype_DISCONNECTED:
      Serial.printf("[WS] Client %u disconnected\n", client);
      break;
    default:
      break;
  }
}

// -----------------------------------------------------------------------------
void connectWiFi() {
  Serial.printf("[WiFi] Connecting to %s", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print('.');
  }
  Serial.printf("\n[WiFi] Connected → %s\n", WiFi.localIP().toString().c_str());
}

// -----------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  myServo.attach(SERVO_PIN);           // attach once – ESP32Servo style
  myServo.write(currentAngle);         // initial position

  connectWiFi();

  webSocket.begin();
  webSocket.onEvent(onWsEvent);

  lastStepTime = millis();
}

// -----------------------------------------------------------------------------
void loop() {
  // keep network alive
  webSocket.loop();
  yield();

  uint32_t now = millis();
  if (now - lastStepTime >= STEP_INTERVAL_MS) {
    // ---- update angle ----
    if (sweepForward) {
      currentAngle += STEP_DEG;
      if (currentAngle >= 180) { currentAngle = 180; sweepForward = false; }
    } else {
      if (currentAngle >= STEP_DEG) currentAngle -= STEP_DEG; else currentAngle = 0;
      if (currentAngle == 0) sweepForward = true;
    }
    myServo.write(currentAngle);

    // ---- measure & transmit ----
    float d = measureDistanceCM();
    if (d >= 0) sendFrame(currentAngle, d);

    lastStepTime = now;
  }
}