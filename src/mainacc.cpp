#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <UniversalTelegramBot.h>
#include <Adafruit_ADXL345_U.h>

/* ---------------- PIN DEFINITIONS ---------------- */
#define TOUCH_PIN   4
#define BUZZER_PIN  15

/* ---------------- THRESHOLDS ---------------- */
#define ON_THRESHOLD   30.0
#define OFF_THRESHOLD  25.0

/* ---------------- WIFI ---------------- */
const char* ssid = "...";
const char* password = "..";

/* ---------------- TELEGRAM ---------------- */
#define BOT_TOKEN "..."
#define CHAT_ID  "..."

WiFiClientSecure client;
UniversalTelegramBot bot(BOT_TOKEN, client);

/* ---------------- ACCELEROMETER ---------------- */
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

/* ---------------- VARIABLES ---------------- */
bool systemOn = false;
bool drowsy = false;
bool alertSent = false;

float ax, ay, az;
float pitch, roll;

unsigned long drowsyStartTime = 0;
unsigned long lastSensorRead = 0;
unsigned long lastSerialPrint = 0;

/* ---------------- SETUP ---------------- */
void setup() {
  Serial.begin(115200);

  pinMode(TOUCH_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  Wire.begin(21, 22);

  if (!accel.begin()) {
    Serial.println("❌ ADXL345 not detected");
    while (1);
  }

  accel.setRange(ADXL345_RANGE_4_G);

  WiFi.begin(ssid, password);
  Serial.print("Connecting WiFi");

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\n✅ WiFi Connected");
  client.setInsecure();
}

/* ---------------- SENSOR READ ---------------- */
void readAccelerometer() {
  sensors_event_t event;
  accel.getEvent(&event);

  ax = event.acceleration.x / 9.81;
  ay = event.acceleration.y / 9.81;
  az = event.acceleration.z / 9.81;
}

/* ---------------- ANGLE CALCULATION ---------------- */
void calculateAngles() {
  pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI;
  roll  = atan2(ay, az) * 180.0 / PI;
}

/* ---------------- TELEGRAM ALERT ---------------- */
void sendTelegramAlert(unsigned long duration) {
  String msg = "🚨 Drowsiness Alert 🚨\n";
  msg += "Duration: " + String(duration) + " sec";
  bot.sendMessage(CHAT_ID, msg, "");
}

/* ---------------- MAIN LOOP ---------------- */
void loop() {

  /* ---- TOUCH TOGGLE ---- */
  if (digitalRead(TOUCH_PIN) == HIGH) {
    delay(200);
    systemOn = !systemOn;
    Serial.println(systemOn ? "🟢 System ON" : "🔴 System OFF");
  }

  if (!systemOn) {
    digitalWrite(BUZZER_PIN, LOW);
    return;
  }

  /* ---- SENSOR UPDATE ---- */
  if (millis() - lastSensorRead >= 50) {
    lastSensorRead = millis();
    readAccelerometer();
    calculateAngles();
  }

  /* ---- DROWSINESS STATE MACHINE ---- */

  // TURN ON
  if (!drowsy &&
      (abs(pitch) > ON_THRESHOLD || abs(roll) > ON_THRESHOLD)) {
    drowsy = true;
    alertSent = false;
    drowsyStartTime = millis();
    Serial.println("⚠ Drowsiness Detected");
  }

  // TURN OFF
  if (drowsy &&
      (abs(pitch) < OFF_THRESHOLD && abs(roll) < OFF_THRESHOLD)) {
    drowsy = false;
    digitalWrite(BUZZER_PIN, LOW);

    unsigned long duration =
        (millis() - drowsyStartTime) / 1000;

    if (!alertSent) {
      sendTelegramAlert(duration);
      alertSent = true;
    }

    Serial.println("✅ Drowsiness Ended");
  }

  /* ---- BUZZER CONTROL ---- */
  digitalWrite(BUZZER_PIN, drowsy ? HIGH : LOW);

  /* ---- SERIAL PRINT (50ms sec) ---- */
  if (millis() - lastSerialPrint >= 50) {
    lastSerialPrint = millis();
    Serial.print("Pitch: ");
    Serial.print(pitch, 1);
    Serial.print(" | Roll: ");
    Serial.println(roll, 1);
  }
}
