#define REMOTEXY_MODE__WIFI
#include <WiFi.h>
#include <RemoteXY.h>
#include <Wire.h>
#include <MPU6050.h>
#include <WebServer.h>
#include <TinyGPS++.h>

// --- [1] إعدادات RemoteXY WiFi ---
#define REMOTEXY_WIFI_SSID "Redmi Note 14"
#define REMOTEXY_WIFI_PASSWORD "1234567890"
#define REMOTEXY_SERVER_PORT 6377

// --- [2] تعريف الأرجل ---
const int IN1 = 27, IN2 = 26, IN3 = 25, IN4 = 33;
const int TRIG_PIN = 5, ECHO_PIN = 18, BUZZER = 23;
#define RXD2 16
#define TXD2 17

// --- [3] واجهة RemoteXY ---
#pragma pack(push, 1)
uint8_t const PROGMEM RemoteXY_CONF_PROGMEM[] =   
  { 255,5,0,0,0,71,0,19,0,0,0,101,115,112,51,50,115,116,97,116,
  105,111,110,32,0,31,1,106,200,1,1,5,0,1,41,6,24,24,0,121,
  31,0,1,15,40,24,24,0,121,31,0,1,66,40,24,24,0,121,31,0,
  1,40,75,24,24,0,121,31,0,1,36,140,37,37,0,1,31,0 };

struct {
  uint8_t Front, Left, Right, Down, btn_reset, connect_flag;
} RemoteXY;   
#pragma pack(pop)

// --- [4] الكائنات ---
MPU6050 mpu;
WebServer server(80);
TinyGPSPlus gps;

float currentDistance = 0, currentAngleX = 0, currentAngleY = 0, impactForce = 0;
float impactForceNet = 0;
String systemStatus = "SAFE";
unsigned long lastSensorTime = 0, lastBeep = 0, lastUltrasonic = 0;
unsigned long tiltStartTime = 0; 
bool potentialTilt = false;
bool accidentLocked = false, buzzerState = false;

#define NUM_READINGS 8
float distanceReadings[NUM_READINGS] = {0};
int readIndex = 0;
bool firstRun = true;

#define MAX_PATH_POINTS 10
struct GPSPoint {
  float lat;
  float lng;
};
GPSPoint routePath[MAX_PATH_POINTS];
int pathIndex = 0, pathCount = 0;
unsigned long lastPathUpdate = 0;

// --- ثوابت النظام ---
#define IMPACT_WARN_THRESHOLD 0.8     // WARNING: > 0.8G
#define IMPACT_CRASH_THRESHOLD 0.8     // CRASH: > 0.8G + جسم قريب
#define IMPACT_ACCIDENT_THRESHOLD 1.0  // ACCIDENT: > 1.0G (مع انقلاب أو مسافة قريبة)
#define TILT_ANGLE_THRESHOLD 45.0      // زاوية الميل
#define TILT_CONFIRM_TIME 1000         // وقت تأكيد الميل (ms)
#define ULTRASONIC_CLOSE 0.10          // مسافة قريبة (10 سم)
#define ACCIDENT_LOCK_DURATION 5000

unsigned long accidentLockTime = 0;
int totalAccidents = 0;

int getSeverity(String status) {
  if (status == "SAFE") return 0;
  if (status == "WARNING") return 1;
  if (status == "CRASH DETECTED") return 2;
  if (status == "TILT DETECTED") return 3;
  if (status == "ACCIDENT DETECTED") return 4;
  return 0;
}

// ============== JSON Data ==============
void handleData() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.sendHeader("Access-Control-Allow-Methods", "GET,POST,OPTIONS");
  server.sendHeader("Access-Control-Allow-Headers", "*");

  float lat = gps.location.isValid() ? gps.location.lat() : 0.0;
  float lng = gps.location.isValid() ? gps.location.lng() : 0.0;
  float speed = gps.location.isValid() ? gps.speed.kmph() : 0.0;
  int sats = gps.satellites.isValid() ? gps.satellites.value() : 0;
  float hdop = gps.hdop.isValid() ? gps.hdop.hdop() : 0.0;
  unsigned long age = gps.location.isValid() ? gps.location.age() : 0;

  String routeJson = "[";
  for (int i = 0; i < pathCount; i++) {
    int idx = (pathIndex - pathCount + i + MAX_PATH_POINTS) % MAX_PATH_POINTS;
    if (i > 0) routeJson += ",";
    routeJson += "{\"lat\":" + String(routePath[idx].lat, 6) + ",\"lng\":" + String(routePath[idx].lng, 6) + "}";
  }
  routeJson += "]";

  String json = "{\"dist\":" + String(currentDistance, 2) + 
                ",\"force\":" + String(impactForceNet, 2) + 
                ",\"angX\":" + String(currentAngleX, 1) + 
                ",\"angY\":" + String(currentAngleY, 1) + 
                ",\"status\":\"" + systemStatus + "\"" +
                ",\"lat\":" + String(lat, 6) + 
                ",\"lng\":" + String(lng, 6) + 
                ",\"speed\":" + String(speed, 1) +
                ",\"sats\":" + String(sats) +
                ",\"hdop\":" + String(hdop, 1) +
                ",\"age\":" + String(age) +
                ",\"totalAccidents\":" + String(totalAccidents) +
                ",\"route\":" + routeJson + "}";
  server.send(200, "application/json", json);
}

void handleOptions() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.sendHeader("Access-Control-Allow-Methods", "GET,POST,OPTIONS");
  server.sendHeader("Access-Control-Allow-Headers", "*");
  server.send(204);
}

// ============== بقية الدوال ==============
void move(int s1, int s2, int s3, int s4) {
  if (accidentLocked && systemStatus != "WARNING") { s1=s2=s3=s4=LOW; }
  digitalWrite(IN1, s1); digitalWrite(IN2, s2);
  digitalWrite(IN3, s3); digitalWrite(IN4, s4);
}

void resetSystem() {
  accidentLocked = false; accidentLockTime = 0; systemStatus = "SAFE";
  potentialTilt = false; noTone(BUZZER); firstRun = true; readIndex = 0;
  pathIndex = 0; pathCount = 0;
  for (int i = 0; i < NUM_READINGS; i++) distanceReadings[i] = 0;
  Serial.println("\n>>> SYSTEM RESET <<<");
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n========================================");
  Serial.println("🚀 FUTURE MAKERS ESP32");
  Serial.println("========================================");

  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2); 

  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT); 
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT); pinMode(ECHO_PIN, INPUT); 
  pinMode(BUZZER, OUTPUT);

  Wire.begin(21, 22);
  mpu.initialize();

  Serial.println("📡 Connecting to WiFi via RemoteXY...");
  RemoteXY_Init();

  Serial.print("⏳ Waiting for WiFi");
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 30) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("✅ WiFi Connected!");
    Serial.print("📡 ESP32 IP Address: ");
    Serial.println(WiFi.localIP());
    Serial.println("👉 اكتب الـ IP ده في كود الـ HTML!");
  } else {
    Serial.println("❌ WiFi Failed! Check SSID/Password");
  }

  server.on("/data", HTTP_GET, handleData);
  server.on("/data", HTTP_OPTIONS, handleOptions);
  server.begin();
  
  Serial.println("✅ WebServer started on port 80");
  Serial.println("========================================\n");
}

void loop() {
  RemoteXY_Handler();
  server.handleClient();

  while (Serial2.available() > 0) gps.encode(Serial2.read());

  if (millis() - lastPathUpdate > 3000 && gps.location.isValid()) {
    lastPathUpdate = millis();
    routePath[pathIndex].lat = gps.location.lat();
    routePath[pathIndex].lng = gps.location.lng();
    pathIndex = (pathIndex + 1) % MAX_PATH_POINTS;
    if (pathCount < MAX_PATH_POINTS) pathCount++;
  }

  if (RemoteXY.btn_reset) { resetSystem(); RemoteXY.btn_reset = 0; }
  if (RemoteXY.Front) move(HIGH, LOW, HIGH, LOW);
  else if (RemoteXY.Down) move(LOW, HIGH, LOW, HIGH);
  else if (RemoteXY.Right) move(HIGH, LOW, LOW, HIGH);
  else if (RemoteXY.Left) move(LOW, HIGH, HIGH, LOW);
  else move(LOW, LOW, LOW, LOW);

  if (millis() - lastUltrasonic > 60) {
    lastUltrasonic = millis();
    digitalWrite(TRIG_PIN, LOW); delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    long dur = pulseIn(ECHO_PIN, HIGH, 25000);
    float raw = (dur == 0) ? 4.0 : (dur * 0.034 / 2) / 100.0;
    if (firstRun) { for (int i=0; i<NUM_READINGS; i++) distanceReadings[i]=raw; firstRun=false; }
    distanceReadings[readIndex] = raw;
    readIndex = (readIndex + 1) % NUM_READINGS;
    float sum = 0; for (int i=0; i<NUM_READINGS; i++) sum += distanceReadings[i];
    currentDistance = sum / NUM_READINGS;
  }

  if (millis() - lastSensorTime > 200) {
    lastSensorTime = millis();
    int16_t ax, ay, az;
    mpu.getAcceleration(&ax, &ay, &az);
    float ax_g = ax / 16384.0, ay_g = ay / 16384.0, az_g = az / 16384.0;
    impactForce = sqrt(ax_g*ax_g + ay_g*ay_g + az_g*az_g);
    impactForceNet = impactForce - 1.0; if (impactForceNet < 0) impactForceNet = 0;

    currentAngleX = atan2(ax_g, sqrt(ay_g*ay_g + az_g*az_g)) * 180 / PI;
    currentAngleY = atan2(ay_g, sqrt(ax_g*ax_g + az_g*az_g)) * 180 / PI;

    bool flipped = (az_g < -0.6);
    bool isTurning = (RemoteXY.Right != 0 || RemoteXY.Left != 0);
    bool checkTilt = (abs(currentAngleX) > TILT_ANGLE_THRESHOLD || abs(currentAngleY) > TILT_ANGLE_THRESHOLD || flipped) && !isTurning;

    if (checkTilt) { if (!potentialTilt) { tiltStartTime = millis(); potentialTilt = true; } }
    else potentialTilt = false;

    bool isTilted = potentialTilt && (millis() - tiltStartTime > TILT_CONFIRM_TIME);
    bool isWarn = impactForceNet > IMPACT_WARN_THRESHOLD;
    bool isCrash = impactForceNet > IMPACT_CRASH_THRESHOLD;
    bool isAccident = impactForceNet > IMPACT_ACCIDENT_THRESHOLD;  // > 1.0G
    bool isVeryClose = (currentDistance < ULTRASONIC_CLOSE);

    String prevStatus = systemStatus;
    String newStatus = systemStatus;

    // ✅✅✅ المنطق الجديد: ACCIDENT في حالتين ✓✓✓
    // الحالة 1: انقلاب + impact > 1.0G (بدون شرط المسافة)
    // الحالة 2: impact > 1.0G + مسافة قريبة (بدون شرط الانقلاب)
    if ((isTilted && isAccident) || (isAccident && isVeryClose)) {
      newStatus = "ACCIDENT DETECTED";
    }
    // TILT: انقلاب بس (من غير impact قوي)
    else if (isTilted) {
      newStatus = "TILT DETECTED";
    }
    // CRASH: impact > 0.8G + جسم قريب
    else if (isCrash && isVeryClose) {
      newStatus = "CRASH DETECTED";
    }
    // WARNING: impact > 0.3G
    else if (isWarn) {
      newStatus = "WARNING";
    }
    else if (!accidentLocked) {
      newStatus = "SAFE";
    }

    // عد الحوادث (ACCIDENT, CRASH, TILT)
    if (getSeverity(newStatus) > getSeverity(prevStatus) && getSeverity(newStatus) >= 2) {
      totalAccidents++;
    }

    if (getSeverity(newStatus) > getSeverity(systemStatus)) {
      systemStatus = newStatus; accidentLocked = true; accidentLockTime = millis();
    }
    if (accidentLocked && (millis() - accidentLockTime > ACCIDENT_LOCK_DURATION) && newStatus == "SAFE") {
      accidentLocked = false; systemStatus = "SAFE"; accidentLockTime = 0;
    }
  }

  if (accidentLocked) {
    if (systemStatus == "ACCIDENT DETECTED") tone(BUZZER, 3000);
    else {
      int interval = (systemStatus == "WARNING") ? 400 : (systemStatus == "CRASH DETECTED" ? 200 : 100);
      if (millis() - lastBeep > interval) {
        lastBeep = millis(); buzzerState = !buzzerState;
        buzzerState ? tone(BUZZER, 1500) : noTone(BUZZER);
      }
    }
  } else noTone(BUZZER);
}