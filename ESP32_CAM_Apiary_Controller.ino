/*
  ============================================================
  ESP32-CAM (AI Thinker) + OV2640 Camera + DHT11 + Dual SG90
  ============================================================
  Description:
    - Initializes OV2640 camera on AI Thinker ESP32-CAM pinout
    - Connects to WiFi and starts a standard WebServer
    - Reads DHT11 temperature/humidity every 2 seconds
    - Controls two SG90 servos based on temperature threshold:
        Temp < 30C  -> Servos move to OPEN position
        Temp >= 30C -> Servos move to CLOSED position
    - OPEN/CLOSED angles are stored as editable variables
      (tune later for the 2 cm mechanical travel requirement)

  Wiring (finalized safe pinout - avoids all strapping pins):
    Servo 1 : Brown -> GND | Red -> 5V (Power Bank) | Orange -> GPIO14
    Servo 2 : Brown -> GND | Red -> 5V (Power Bank) | Orange -> GPIO4
    DHT11   : VCC -> 3.3V  | GND -> GND              | DATA   -> GPIO13

    Note: GPIO4 also drives the onboard flash LED, so it will
    flicker faintly with servo pulses. This is cosmetic only.

  This sketch is Part 1A. It works together with two additional
  tabs saved in the SAME sketch folder:
    - dashboard_html.h   (Part 1B - web dashboard)
    - NeonTracking.ino   (Part 1C - color tracking + MJPEG stream)

  Libraries required (install via Arduino Library Manager):
    - ESP32 Board Package (esp32 by Espressif Systems)
    - DHT sensor library (by Adafruit)
    - Adafruit Unified Sensor (dependency of DHT library)
    - ESP32Servo (by Kevin Harrington / madhephaestus)

  Board: AI Thinker ESP32-CAM
  ============================================================
*/

#include <WiFi.h>
#include <WebServer.h>
#include <esp_camera.h>
#include <DHT.h>
#include <ESP32Servo.h>
#include "dashboard_html.h"   // Part 1B - dashboard page
#include "soc/soc.h"          // Diagnostic only: brownout detector control
#include "soc/rtc_cntl_reg.h" // Diagnostic only: brownout detector control

// ============================================================
// WiFi Credentials (User-defined)
// ============================================================
const char* WIFI_SSID     = "ruben";
const char* WIFI_PASSWORD = "12345678";

// ============================================================
// AI Thinker ESP32-CAM Pin Definitions (OV2640)
// ============================================================
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

// ============================================================
// DHT11 Configuration
// ============================================================
#define DHT_PIN   13        // GPIO connected to DHT11 data pin
#define DHT_TYPE  DHT11
DHT dht(DHT_PIN, DHT_TYPE);

// Timing for DHT reads (non-blocking, every 2 seconds)
unsigned long lastDHTReadTime = 0;
const unsigned long DHT_READ_INTERVAL = 2000; // 2 seconds

// Latest sensor readings (global so other functions can use them)
float currentTemperature = 0.0;
float currentHumidity    = 0.0;
bool  dhtReadValid        = false;

// ============================================================
// Servo Configuration (finalized safe pinout)
// ============================================================
#define SERVO1_PIN  14   // Safe: not a strapping pin, not camera-related
#define SERVO2_PIN   4   // Safe: not a strapping pin (shares pin with flash LED)

Servo servo1;
Servo servo2;

// Editable angle variables — tune these to achieve the required
// 2 cm mechanical movement of the queen excluder mechanism.
// Editable angle variables — tune these to achieve the required
// 2 cm mechanical movement of the queen excluder mechanism.
// Servo 2 is MIRRORED because it's mounted facing the opposite
// direction on the other side of the box — using swapped angles
// makes both racks move the same physical direction together.
int SERVO1_OPEN_ANGLE   = 100;  // Servo 1: OPEN position
int SERVO1_CLOSED_ANGLE = 0;    // Servo 1: CLOSED position
int SERVO2_OPEN_ANGLE   = 0;    // Servo 2: OPEN position (mirrored)
int SERVO2_CLOSED_ANGLE = 100;  // Servo 2: CLOSED position (mirrored)

// Temperature threshold (in Celsius) that decides servo state
const float TEMP_THRESHOLD_C = 35.0;

// Track current servo state to avoid redundant writes (optional but clean)
bool servosCurrentlyOpen = true;

// ============================================================
// Combined Servo Control State (Temperature + Neon Proximity)
// ============================================================
// Both conditions act on the SAME pair of servos, simultaneously:
//   - tempWantsClose      : true when temperature >= 30C
//   - proximityWantsClose : true when the neon-marked bee is
//                           close to the camera (set in
//                           NeonTracking.ino)
// Final rule (safety-first OR logic): the gate CLOSES if EITHER
// condition calls for closing, and only OPENS when BOTH agree
// it's safe to open (temp below 30C AND bee far from camera).
bool tempWantsClose      = false;
bool proximityWantsClose = false;

// ============================================================
// Simulated / Derived Dashboard Metrics
// (placeholders until real sensors are added for these)
// ============================================================
float simulatedHiveWeight = 20.0; // kg, simulated
int   cleanlinessPercent  = 90;   // %, simulated
int   honeyFillPercent    = 40;   // %, simulated

// ============================================================
// Web Server
// ============================================================
WebServer server(80);

// ============================================================
// Function Prototypes
// ============================================================
void setupCamera();
void setupWiFi();
void setupWebServer();
void readDHT();
void updateServos();
void applyServoDecision();
void handleRoot();
void handleStatus();

// Provided by NeonTracking.ino (Part 1C)
void setupColorTracking();
void trackNeonColor();
void handleStream();

// ============================================================
// SETUP
// ============================================================
void setup() {
  // DIAGNOSTIC ONLY: disables the brownout reset so you can see what's
  // really happening instead of a silent reboot loop. This does NOT fix
  // the underlying power problem — it just stops the chip from hiding it.
  // Remove this line once you're running on a proper external 5V/2A supply.
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

  Serial.begin(115200);
  Serial.println();
  Serial.println("=== ESP32-CAM AIoT Controller Booting ===");

  // Initialize DHT11 sensor
  dht.begin();

  // Explicitly allocate separate PWM timers for the servos so they
  // don't collide with the camera's LEDC timer/channel 0 (used for
  // XCLK generation). Without this, .write() can execute correctly
  // in software while the actual PWM pulse to the servo never comes
  // out cleanly, so the servo appears to not move at all.
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  servo1.setPeriodHertz(50); // Standard 50Hz servo pulse rate
  servo2.setPeriodHertz(50);

  // Attach and initialize servos
  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);
  // Set initial safe position (OPEN by default at startup)
  servo1.write(SERVO1_OPEN_ANGLE);
  servo2.write(SERVO2_OPEN_ANGLE);
  servosCurrentlyOpen = true;

  // Initialize camera
  setupCamera();

  // Connect to WiFi
  setupWiFi();

  // Start web server routes
  setupWebServer();

  // Initialize neon color tracking (Part 1C)
  setupColorTracking();

  Serial.println("=== Setup Complete ===");
}

// ============================================================
// MAIN LOOP
// ============================================================
void loop() {
  // Handle incoming web client requests
  server.handleClient();

  // Read DHT11 every 2 seconds (non-blocking)
  unsigned long now = millis();
  if (now - lastDHTReadTime >= DHT_READ_INTERVAL) {
    lastDHTReadTime = now;
    readDHT();

    // Update servo positions based on latest valid reading
    if (dhtReadValid) {
      updateServos();
    }
  }

  // Run neon color tracking (internally rate-limited, Part 1C)
  trackNeonColor();
}

// ============================================================
// setupCamera()
// Initializes the OV2640 camera using AI Thinker pin mapping
// ============================================================
void setupCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0       = Y2_GPIO_NUM;
  config.pin_d1       = Y3_GPIO_NUM;
  config.pin_d2       = Y4_GPIO_NUM;
  config.pin_d3       = Y5_GPIO_NUM;
  config.pin_d4       = Y6_GPIO_NUM;
  config.pin_d5       = Y7_GPIO_NUM;
  config.pin_d6       = Y8_GPIO_NUM;
  config.pin_d7       = Y9_GPIO_NUM;
  config.pin_xclk     = XCLK_GPIO_NUM;
  config.pin_pclk     = PCLK_GPIO_NUM;
  config.pin_vsync    = VSYNC_GPIO_NUM;
  config.pin_href     = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn     = PWDN_GPIO_NUM;
  config.pin_reset    = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  // Use higher resolution/quality if PSRAM is available
  if (psramFound()) {
    config.frame_size   = FRAMESIZE_SVGA;
    config.jpeg_quality  = 10;
    config.fb_count      = 2;
  } else {
    config.frame_size   = FRAMESIZE_VGA;
    config.jpeg_quality  = 12;
    config.fb_count      = 1;
  }

  // Initialize the camera driver
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init FAILED with error 0x%x\n", err);
    return;
  }

  Serial.println("Camera initialized successfully.");
}

// ============================================================
// setupWiFi()
// Connects the ESP32-CAM to the configured WiFi network
// ============================================================
void setupWiFi() {
  Serial.print("Connecting to WiFi: ");
  Serial.println(WIFI_SSID);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  int retryCount = 0;
  while (WiFi.status() != WL_CONNECTED && retryCount < 40) {
    delay(500);
    Serial.print(".");
    retryCount++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println();
    Serial.print("WiFi connected. IP address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println();
    Serial.println("WiFi connection FAILED. Check credentials/signal.");
  }
}

// ============================================================
// setupWebServer()
// Configures and starts the standard WebServer instance
// ============================================================
void setupWebServer() {
  server.on("/", HTTP_GET, handleRoot);
  server.on("/status", HTTP_GET, handleStatus);
  server.on("/stream", HTTP_GET, handleStream); // Part 1C - MJPEG stream

  server.begin();
  Serial.println("Web server started on port 80.");
}

// ============================================================
// handleRoot()
// Serves the IntelliHive dashboard (Part 1B)
// ============================================================
void handleRoot() {
  server.send_P(200, "text/html", DASHBOARD_HTML);
}

// ============================================================
// handleStatus()
// Returns current sensor + servo status as JSON for the
// dashboard's fetch() polling (Part 1B). Note: neon tracking
// data is intentionally NOT included here, per requirements.
// ============================================================
void handleStatus() {
  String json = "{";
  json += "\"temperature\":" + String(currentTemperature, 1) + ",";
  json += "\"humidity\":" + String(currentHumidity, 1) + ",";
  json += "\"servoStatus\":\"" + String(servosCurrentlyOpen ? "OPEN" : "CLOSED") + "\",";
  json += "\"wifiStatus\":\"" + String(WiFi.status() == WL_CONNECTED ? "Connected" : "Disconnected") + "\",";
  json += "\"cleanliness\":" + String(cleanlinessPercent) + ",";
  json += "\"hiveWeight\":" + String(simulatedHiveWeight, 1) + ",";
  json += "\"honeyFill\":" + String(honeyFillPercent);
  json += "}";

  server.send(200, "application/json", json);
}

// ============================================================
// readDHT()
// Reads temperature and humidity from the DHT11 sensor
// and prints the result to the Serial Monitor
// ============================================================
void readDHT() {
  float humidity    = dht.readHumidity();
  float temperature = dht.readTemperature(); // Celsius

  // Validate readings
  if (isnan(humidity) || isnan(temperature)) {
    Serial.println("DHT11 read failed. Check wiring.");
    dhtReadValid = false;
    return;
  }

  currentTemperature = temperature;
  currentHumidity    = humidity;
  dhtReadValid        = true;

  // Print readings to Serial Monitor
  Serial.print("Temperature: ");
  Serial.print(currentTemperature);
  Serial.print(" C | Humidity: ");
  Serial.print(currentHumidity);
  Serial.println(" %");
}

// ============================================================
// updateServos()
// Evaluates the TEMPERATURE half of the servo decision.
// Sets tempWantsClose, then hands off to applyServoDecision()
// so temperature and neon proximity are combined consistently.
// ============================================================
void updateServos() {
  tempWantsClose = (currentTemperature >= TEMP_THRESHOLD_C);
  applyServoDecision();
}

// ============================================================
// applyServoDecision()
// Combines temperature and neon-proximity signals to drive
// BOTH servos simultaneously. Called any time either signal
// updates (from updateServos() here, and from
// updateServoByProximity() in NeonTracking.ino).
//
// Rule: CLOSE if temperature >= 30C OR bee is close to camera.
//       OPEN only if temperature < 30C AND bee is far/absent.
// This favors closing (protecting the queen) whenever either
// condition says to close.
// ============================================================
void applyServoDecision() {
  bool shouldClose = tempWantsClose || proximityWantsClose;

  if (shouldClose && servosCurrentlyOpen) {
    servo1.write(SERVO1_CLOSED_ANGLE);
    servo2.write(SERVO2_CLOSED_ANGLE);
    servosCurrentlyOpen = false;
    Serial.print("Servos CLOSED (Temp>=30C: ");
    Serial.print(tempWantsClose ? "yes" : "no");
    Serial.print(", Bee close: ");
    Serial.print(proximityWantsClose ? "yes" : "no");
    Serial.println(")");
  } else if (!shouldClose && !servosCurrentlyOpen) {
    servo1.write(SERVO1_OPEN_ANGLE);
    servo2.write(SERVO2_OPEN_ANGLE);
    servosCurrentlyOpen = true;
    Serial.println("Servos OPEN (Temp<30C and bee far/absent).");
  }
  // else: already in the correct state, no redundant write
}
