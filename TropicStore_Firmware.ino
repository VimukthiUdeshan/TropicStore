/*
 * TropicStore Smart Fruit Ripening Chamber - AWS IoT + Device Shadow
 * ESP32 DOIT DevKit V1
 *
 * HARDWARE:
 *
 * DHT22 -> GPIO32
 * MQ3   -> GPIO34
 *
 * OLED SH1106:
 * SDA -> GPIO21
 * SCL -> GPIO22
 *
 * ACTIVE LOW RELAYS:
 * Relay 1 - Peltier         -> GPIO26
 * Relay 2 - Exhaust Fan     -> GPIO27
 * Relay 3 - Humidifier Fan  -> GPIO14
 * Relay 4 - Mist Button     -> GPIO25
 *
 * Exhaust Servo -> GPIO13
 *
 * EXHAUST LOGIC:
 *
 * Gas >= HIGH threshold continuously for 10 seconds:
 *      Door opens
 *      Wait 700 ms
 *      Exhaust fan starts
 *
 * Gas <= LOW threshold continuously for 10 seconds:
 *      Exhaust fan stops
 *      Wait 1000 ms
 *      Door closes
 *
 * Between LOW and HIGH:
 *      Keep previous exhaust state.
 */

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <DHT.h>
#include <MovingAverage.h>
#include <ArduinoJson.h>
#include <ESP32Servo.h>
#include <math.h>
#include <time.h>

// ============================================================
// WIFI
// ============================================================

#define WIFI_SSID "######"
#define WIFI_PASSWORD "#######"

// ============================================================
// AWS IoT
// ============================================================

#define AWS_IOT_ENDPOINT \
  "a2#######zlhb-ats.iot.ap-southeast-1.amazonaws.com"

#define AWS_IOT_PORT 8883
#define THING_NAME "ESP32TropicStore"

// ============================================================
// MQTT TOPICS
// ============================================================

#define MQTT_BASE_TOPIC "tropicstore/chamber01"

#define TOPIC_TELEMETRY MQTT_BASE_TOPIC "/telemetry"
#define TOPIC_STATUS MQTT_BASE_TOPIC "/status"

#define TOPIC_SETTINGS_SUB MQTT_BASE_TOPIC "/settings/#"
#define TOPIC_CONTROL_SUB MQTT_BASE_TOPIC "/control/#"

#define TOPIC_TARGET_TEMP \
  MQTT_BASE_TOPIC "/settings/targetTemperature"

#define TOPIC_TEMP_TOLERANCE \
  MQTT_BASE_TOPIC "/settings/temperatureTolerance"

#define TOPIC_TARGET_HUMIDITY \
  MQTT_BASE_TOPIC "/settings/targetHumidity"

#define TOPIC_HUMIDITY_TOLERANCE \
  MQTT_BASE_TOPIC "/settings/humidityTolerance"

#define TOPIC_GAS_HIGH \
  MQTT_BASE_TOPIC "/settings/gasThresholdHigh"

#define TOPIC_GAS_LOW \
  MQTT_BASE_TOPIC "/settings/gasThresholdLow"

#define TOPIC_MODE \
  MQTT_BASE_TOPIC "/control/mode"

#define TOPIC_MANUAL_PELTIER \
  MQTT_BASE_TOPIC "/control/manualPeltier"

#define TOPIC_MANUAL_HUMIDIFIER \
  MQTT_BASE_TOPIC "/control/manualHumidifier"

#define TOPIC_MANUAL_EXHAUST \
  MQTT_BASE_TOPIC "/control/manualExhaust"

#define TOPIC_CALIBRATE_MQ3 \
  MQTT_BASE_TOPIC "/control/calibrateMQ3"

// ============================================================
// AWS DEVICE SHADOW
// ============================================================

#define SHADOW_UPDATE \
  "$aws/things/ESP32TropicStore/shadow/update"

#define SHADOW_UPDATE_DELTA \
  "$aws/things/ESP32TropicStore/shadow/update/delta"

#define SHADOW_UPDATE_ACCEPTED \
  "$aws/things/ESP32TropicStore/shadow/update/accepted"

#define SHADOW_UPDATE_REJECTED \
  "$aws/things/ESP32TropicStore/shadow/update/rejected"

#define SHADOW_GET \
  "$aws/things/ESP32TropicStore/shadow/get"

#define SHADOW_GET_ACCEPTED \
  "$aws/things/ESP32TropicStore/shadow/get/accepted"

#define SHADOW_GET_REJECTED \
  "$aws/things/ESP32TropicStore/shadow/get/rejected"

// ==================== AWS CERTIFICATES ====================
// Amazon Root CA 1
// Paste the complete contents of AmazonRootCA1.pem here.
static const char AWS_ROOT_CA[] PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----
##############################################################
##############################################################
##############################################################
##############################################################
##############################################################
-----END CERTIFICATE-----
)EOF";

// Device certificate
// Paste the complete contents of your xxxx-certificate.pem.crt here.
static const char AWS_DEVICE_CERT[] PROGMEM = R"KEY(
-----BEGIN CERTIFICATE-----
##############################################################
##############################################################
##############################################################
##############################################################
##############################################################
-----END CERTIFICATE-----
)KEY";

// Device private key
// Paste the complete contents of your xxxx-private.pem.key here.
static const char AWS_PRIVATE_KEY[] PROGMEM = R"KEY(
-----BEGIN RSA PRIVATE KEY-----
##############################################################
##############################################################
##############################################################
##############################################################
##############################################################
-----END RSA PRIVATE KEY-----
)KEY";

// ============================================================
// PIN DEFINITIONS
// ============================================================

#define DHT_PIN 32
#define MQ3_PIN 34

#define OLED_SDA 21
#define OLED_SCL 22
#define OLED_ADDRESS 0x3C

// Active LOW relays
#define RELAY_PELTIER 26
#define RELAY_EXHAUST 27
#define RELAY_HUMIDIFIER_FAN 14
#define RELAY_MIST_BUTTON 25

#define SERVO_EXHAUST_PIN 13

// ============================================================
// CONSTANTS
// ============================================================

#define DHT_TYPE DHT22

#define MQ3_WARMUP_TIME 180000UL
#define MQ3_FILTER_WINDOW 5

#define MQ3_RL 30.0
#define MQ3_A 0.4019
#define MQ3_B -1.51187
#define VCC 5.0

// Servo angles
#define EXHAUST_DOOR_CLOSED_ANGLE 0
#define EXHAUST_DOOR_OPEN_ANGLE 70

// Temperature
#define DEFAULT_TARGET_TEMP 13.0
#define DEFAULT_TEMP_TOLERANCE 1.0

// Humidity
#define DEFAULT_TARGET_HUMIDITY 85.0
#define DEFAULT_HUMIDITY_TOLERANCE 5.0

// MQ3 exhaust thresholds
#define DEFAULT_GAS_THRESHOLD_HIGH 500
#define DEFAULT_GAS_THRESHOLD_LOW 0

// Gas must remain continuously above/below threshold
#define GAS_STABLE_TIME 10000UL

// Servo/fan timing
#define EXHAUST_DOOR_OPEN_DELAY 700UL
#define EXHAUST_DOOR_CLOSE_DELAY 1000UL

// General timing
#define SENSOR_READ_INTERVAL 2000UL
#define SERIAL_PRINT_INTERVAL 2000UL
#define OLED_UPDATE_INTERVAL 100UL

#define AWS_SEND_INTERVAL 5000UL
#define AWS_RECONNECT_INTERVAL 10000UL
#define SHADOW_REPORT_INTERVAL 10000UL

#define MIST_BUTTON_PULSE_TIME 250UL

// ============================================================
// OBJECTS
// ============================================================

U8G2_SH1106_128X64_NONAME_F_HW_I2C oled(
  U8G2_R0,
  U8X8_PIN_NONE
);

DHT dht(DHT_PIN, DHT_TYPE);

MovingAverage mq3Filter(MQ3_FILTER_WINDOW);

WiFiClientSecure secureClient;
PubSubClient mqttClient(secureClient);

Servo exhaustDoorServo;

// ============================================================
// SYSTEM MODE
// ============================================================

enum SystemMode {
  MODE_AUTO,
  MODE_MANUAL
};

SystemMode currentMode = MODE_AUTO;

// ============================================================
// SENSOR VARIABLES
// ============================================================

float temperature = 0.0;
float humidity = 0.0;

int gasRaw = 0;
int gasFiltered = 0;

float gasPPM = 0.0;
float gasVoltage = 0.0;
float gasRs = 0.0;
float gasRo = 0.0;

bool gasCalibrated = false;
bool dhtValid = false;

unsigned long mq3StartTime = 0;
bool mq3WarmedUp = false;

// ============================================================
// CONTROL SETTINGS
// ============================================================

float targetTemperature = DEFAULT_TARGET_TEMP;
float temperatureTolerance = DEFAULT_TEMP_TOLERANCE;

float targetHumidity = DEFAULT_TARGET_HUMIDITY;
float humidityTolerance = DEFAULT_HUMIDITY_TOLERANCE;

int gasThresholdHigh = DEFAULT_GAS_THRESHOLD_HIGH;
int gasThresholdLow = DEFAULT_GAS_THRESHOLD_LOW;

// ============================================================
// ACTUATOR STATES
// ============================================================

bool peltierState = false;
bool exhaustState = false;

bool humidifierState = false;
bool humidifierFanState = false;

bool exhaustDoorOpen = false;

// ============================================================
// MANUAL CONTROL
// ============================================================

bool manualPeltier = false;
bool manualHumidifier = false;
bool manualExhaust = false;

// ============================================================
// TIMING
// ============================================================

unsigned long lastSensorRead = 0;
unsigned long lastSerialPrint = 0;
unsigned long lastOLEDUpdate = 0;
unsigned long lastAWSSend = 0;
unsigned long lastShadowReport = 0;

unsigned long lastAWSReconnectAttempt = 0;
unsigned long lastWiFiReconnectAttempt = 0;

// Exhaust stability timers
unsigned long gasHighStartTime = 0;
unsigned long gasLowStartTime = 0;

bool wifiConnected = false;
bool awsConnected = false;

int frame = 0;

// ============================================================
// FUNCTION PROTOTYPES
// ============================================================

void setupWiFi();
void setupAWS();
void syncClock();

void handleConnections();
bool connectAWS();

void mqttCallback(char* topic, byte* payload, unsigned int length);

void subscribeAWSTopics();
void subscribeShadowTopics();

void requestShadow();

void handleShadowDelta(const String& payload);
void handleShadowGet(const String& payload);
void applyShadowState(JsonObject state);
void reportShadowState();

bool parseBoolPayload(const String& value);

void calibrateMQ3();
void readSensors();

void controlTemperature();
void controlHumidity();
void controlGas();
void controlExhaustByGas(int gasValue);

void updateOLED();
void sendAWS();
void printSerial();

void pulseMistButton();
void setHumidifier(bool desiredState);

void setRelay(int pin, bool state);
void updateRelayStates();

void openExhaustDoor();
void closeExhaustDoor();
void updateExhaustDoor();

void manualExhaustControl(bool state);

void drawWiFi(int x, int y);
void drawSnowflake(int x, int y);
void drawDrop(int x, int y);
void drawFan(int x, int y);

// ============================================================
// SETUP
// ============================================================

void setup() {

  Serial.begin(115200);

  Serial.println();
  Serial.println();
  Serial.println("=== Smart Fruit Ripening Chamber ===");
  Serial.println("=== AWS IoT + Device Shadow ===");

  // Relay outputs
  pinMode(RELAY_PELTIER, OUTPUT);
  pinMode(RELAY_EXHAUST, OUTPUT);
  pinMode(RELAY_HUMIDIFIER_FAN, OUTPUT);
  pinMode(RELAY_MIST_BUTTON, OUTPUT);

  // ACTIVE LOW: HIGH = OFF
  digitalWrite(RELAY_PELTIER, HIGH);
  digitalWrite(RELAY_EXHAUST, HIGH);
  digitalWrite(RELAY_HUMIDIFIER_FAN, HIGH);
  digitalWrite(RELAY_MIST_BUTTON, HIGH);

  peltierState = false;
  exhaustState = false;
  humidifierState = false;
  humidifierFanState = false;

  Serial.println("All relays initialized OFF.");

  // ----------------------------------------------------------
  // SERVO
  // ----------------------------------------------------------

  exhaustDoorServo.setPeriodHertz(50);

  exhaustDoorServo.attach(
    SERVO_EXHAUST_PIN,
    500,
    2400
  );

  exhaustDoorServo.write(
    EXHAUST_DOOR_CLOSED_ANGLE
  );

  exhaustDoorOpen = false;

  Serial.printf(
    "Exhaust door initialized CLOSED at %d degrees.\n",
    EXHAUST_DOOR_CLOSED_ANGLE
  );

  delay(500);

  // ADC
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  // Sensors
  Wire.begin(OLED_SDA, OLED_SCL);

  dht.begin();

  delay(100);

  mq3StartTime = millis();

  // OLED
  oled.begin();

  for (int p = 0; p <= 100; p += 5) {

    oled.clearBuffer();

    oled.setFont(u8g2_font_6x10_tr);

    oled.drawStr(25, 18, "TropicStore");
    oled.drawStr(35, 32, "CHAMBER");

    oled.drawFrame(10, 45, 108, 8);
    oled.drawBox(12, 47, p, 4);

    oled.sendBuffer();

    delay(70);
  }

  delay(500);

  setupWiFi();
  setupAWS();

  Serial.println("System initialization complete!");
}

// ============================================================
// LOOP
// ============================================================

void loop() {

  unsigned long currentMillis = millis();

  handleConnections();

  if (mqttClient.connected()) {
    mqttClient.loop();
  }

  if (
    currentMillis - lastSensorRead >=
    SENSOR_READ_INTERVAL
  ) {

    lastSensorRead = currentMillis;

    readSensors();
  }

  if (currentMode == MODE_AUTO) {

    controlTemperature();
    controlHumidity();
    controlGas();
  }

  if (
    currentMillis - lastOLEDUpdate >=
    OLED_UPDATE_INTERVAL
  ) {

    lastOLEDUpdate = currentMillis;

    updateOLED();

    frame++;

    if (frame > 1000) {
      frame = 0;
    }
  }

  if (
    currentMillis - lastAWSSend >=
    AWS_SEND_INTERVAL
  ) {

    lastAWSSend = currentMillis;
    sendAWS();
  }

  if (
    currentMillis - lastShadowReport >=
    SHADOW_REPORT_INTERVAL
  ) {

    lastShadowReport = currentMillis;
    reportShadowState();
  }

  if (
    currentMillis - lastSerialPrint >=
    SERIAL_PRINT_INTERVAL
  ) {

    lastSerialPrint = currentMillis;
    printSerial();
  }
}

// ============================================================
// WIFI
// ============================================================

void setupWiFi() {

  Serial.printf(
    "Connecting to WiFi: %s",
    WIFI_SSID
  );

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  unsigned long startTime = millis();

  while (
    WiFi.status() != WL_CONNECTED &&
    millis() - startTime < 30000UL
  ) {

    Serial.print(".");
    delay(500);
  }

  wifiConnected =
    (WiFi.status() == WL_CONNECTED);

  if (wifiConnected) {

    Serial.println();
    Serial.println("WiFi connected!");

    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());

    syncClock();

  } else {

    Serial.println();
    Serial.println("WiFi connection failed.");
  }
}

// ============================================================
// NTP
// ============================================================

void syncClock() {

  Serial.print("Synchronizing time");

  configTime(
    0,
    0,
    "pool.ntp.org",
    "time.nist.gov"
  );

  time_t now = time(nullptr);
  unsigned long start = millis();

  while (
    now < 1700000000 &&
    millis() - start < 20000UL
  ) {

    Serial.print(".");
    delay(500);

    now = time(nullptr);
  }

  Serial.println();

  if (now >= 1700000000) {
    Serial.println("Time synchronized.");
  } else {
    Serial.println("WARNING: NTP synchronization timed out.");
  }
}

// ============================================================
// AWS SETUP
// ============================================================

void setupAWS() {

  secureClient.setCACert(AWS_ROOT_CA);
  secureClient.setCertificate(AWS_DEVICE_CERT);
  secureClient.setPrivateKey(AWS_PRIVATE_KEY);

  mqttClient.setServer(
    AWS_IOT_ENDPOINT,
    AWS_IOT_PORT
  );

  mqttClient.setCallback(mqttCallback);
  mqttClient.setBufferSize(2048);
  mqttClient.setKeepAlive(60);

  connectAWS();
}

// ============================================================
// AWS CONNECTION
// ============================================================

bool connectAWS() {

  if (WiFi.status() != WL_CONNECTED) {

    wifiConnected = false;
    awsConnected = false;

    return false;
  }

  if (mqttClient.connected()) {

    awsConnected = true;

    return true;
  }

  Serial.printf(
    "Connecting to AWS IoT as %s...\n",
    THING_NAME
  );

  if (mqttClient.connect(THING_NAME)) {

    awsConnected = true;

    Serial.println("AWS IoT connected!");

    subscribeAWSTopics();
    subscribeShadowTopics();

    mqttClient.loop();

    delay(100);

    requestShadow();

    sendAWS();
    reportShadowState();

    return true;
  }

  awsConnected = false;

  Serial.printf(
    "AWS connection failed. MQTT state = %d\n",
    mqttClient.state()
  );

  return false;
}

// ============================================================
// SUBSCRIPTIONS
// ============================================================

void subscribeAWSTopics() {

  mqttClient.subscribe(
    TOPIC_SETTINGS_SUB,
    1
  );

  mqttClient.subscribe(
    TOPIC_CONTROL_SUB,
    1
  );
}

void subscribeShadowTopics() {

  mqttClient.subscribe(
    SHADOW_UPDATE_DELTA,
    1
  );

  mqttClient.subscribe(
    SHADOW_GET_ACCEPTED,
    1
  );

  mqttClient.subscribe(
    SHADOW_GET_REJECTED,
    1
  );

  mqttClient.subscribe(
    SHADOW_UPDATE_REJECTED,
    1
  );
}

// ============================================================
// REQUEST SHADOW
// ============================================================

void requestShadow() {

  if (!mqttClient.connected()) {
    return;
  }

  mqttClient.publish(
    SHADOW_GET,
    ""
  );
}

// ============================================================
// CONNECTION HANDLING
// ============================================================

void handleConnections() {

  unsigned long now = millis();

  wifiConnected =
    (WiFi.status() == WL_CONNECTED);

  if (!wifiConnected) {

    awsConnected = false;

    if (
      now - lastWiFiReconnectAttempt >=
      AWS_RECONNECT_INTERVAL
    ) {

      lastWiFiReconnectAttempt = now;

      WiFi.disconnect();
      WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    }

    return;
  }

  if (!mqttClient.connected()) {

    awsConnected = false;

    if (
      now - lastAWSReconnectAttempt >=
      AWS_RECONNECT_INTERVAL
    ) {

      lastAWSReconnectAttempt = now;

      connectAWS();
    }

  } else {

    awsConnected = true;
  }
}

// ============================================================
// BOOL PARSER
// ============================================================

bool parseBoolPayload(
  const String& value
) {

  String v = value;

  v.trim();
  v.toLowerCase();

  return (
    v == "true" ||
    v == "1" ||
    v == "on" ||
    v == "yes"
  );
}

// ============================================================
// MQTT CALLBACK
// ============================================================

void mqttCallback(
  char* topic,
  byte* payload,
  unsigned int length
) {

  String topicStr(topic);
  String value;

  value.reserve(length + 1);

  for (
    unsigned int i = 0;
    i < length;
    i++
  ) {

    value += (char)payload[i];
  }

  value.trim();

  Serial.printf(
    "[AWS RX] %s -> %s\n",
    topicStr.c_str(),
    value.c_str()
  );

  if (topicStr == SHADOW_UPDATE_DELTA) {

    handleShadowDelta(value);
    return;
  }

  if (topicStr == SHADOW_GET_ACCEPTED) {

    handleShadowGet(value);
    return;
  }

  if (
    topicStr == SHADOW_GET_REJECTED ||
    topicStr == SHADOW_UPDATE_REJECTED
  ) {

    Serial.println(value);
    return;
  }

  if (topicStr == TOPIC_TARGET_TEMP) {

    targetTemperature = value.toFloat();
    reportShadowState();
  }

  else if (topicStr == TOPIC_TEMP_TOLERANCE) {

    temperatureTolerance =
      fabs(value.toFloat());

    reportShadowState();
  }

  else if (topicStr == TOPIC_TARGET_HUMIDITY) {

    targetHumidity = value.toFloat();
    reportShadowState();
  }

  else if (topicStr == TOPIC_HUMIDITY_TOLERANCE) {

    humidityTolerance =
      fabs(value.toFloat());

    reportShadowState();
  }

  else if (topicStr == TOPIC_GAS_HIGH) {

    gasThresholdHigh = value.toInt();

    gasHighStartTime = 0;
    gasLowStartTime = 0;

    reportShadowState();
  }

  else if (topicStr == TOPIC_GAS_LOW) {

    gasThresholdLow = value.toInt();

    gasHighStartTime = 0;
    gasLowStartTime = 0;

    reportShadowState();
  }

  else if (topicStr == TOPIC_MODE) {

    String mode = value;

    mode.toUpperCase();

    if (mode == "MANUAL") {

      currentMode = MODE_MANUAL;

      gasHighStartTime = 0;
      gasLowStartTime = 0;

    } else {

      currentMode = MODE_AUTO;

      gasHighStartTime = 0;
      gasLowStartTime = 0;
    }

    reportShadowState();
  }

  else if (topicStr == TOPIC_MANUAL_PELTIER) {

    manualPeltier =
      parseBoolPayload(value);

    if (currentMode == MODE_MANUAL) {

      setRelay(
        RELAY_PELTIER,
        manualPeltier
      );

      reportShadowState();
    }
  }

  else if (topicStr == TOPIC_MANUAL_HUMIDIFIER) {

    manualHumidifier =
      parseBoolPayload(value);

    if (currentMode == MODE_MANUAL) {

      setHumidifier(
        manualHumidifier
      );

      reportShadowState();
    }
  }

  else if (topicStr == TOPIC_MANUAL_EXHAUST) {

    manualExhaust =
      parseBoolPayload(value);

    if (currentMode == MODE_MANUAL) {

      manualExhaustControl(
        manualExhaust
      );

      reportShadowState();
    }
  }

  else if (topicStr == TOPIC_CALIBRATE_MQ3) {

    if (parseBoolPayload(value)) {

      if (!mq3WarmedUp) {

        Serial.println(
          "MQ3 calibration ignored: warming up."
        );

      } else {

        calibrateMQ3();
      }

      mqttClient.publish(
        TOPIC_CALIBRATE_MQ3,
        "false",
        true
      );
    }
  }

  if (
    gasThresholdLow >
    gasThresholdHigh
  ) {

    Serial.println(
      "WARNING: gasThresholdLow > gasThresholdHigh."
    );
  }
}

// ============================================================
// SHADOW DELTA
// ============================================================

void handleShadowDelta(
  const String& payload
) {

  JsonDocument doc;

  DeserializationError error =
    deserializeJson(doc, payload);

  if (error) {

    Serial.println(error.c_str());
    return;
  }

  JsonObject state =
    doc["state"].as<JsonObject>();

  if (state.isNull()) {
    return;
  }

  applyShadowState(state);
  reportShadowState();
}

// ============================================================
// SHADOW GET
// ============================================================

void handleShadowGet(
  const String& payload
) {

  JsonDocument doc;

  DeserializationError error =
    deserializeJson(doc, payload);

  if (error) {

    Serial.println(error.c_str());
    return;
  }

  JsonObject desired =
    doc["state"]["desired"].as<JsonObject>();

  if (!desired.isNull()) {

    applyShadowState(desired);
  }

  reportShadowState();
}

// ============================================================
// APPLY SHADOW
// ============================================================

void applyShadowState(
  JsonObject state
) {

  if (state.containsKey("targetTemperature")) {

    targetTemperature =
      state["targetTemperature"].as<float>();
  }

  if (state.containsKey("temperatureTolerance")) {

    temperatureTolerance =
      fabs(
        state["temperatureTolerance"].as<float>()
      );
  }

  if (state.containsKey("targetHumidity")) {

    targetHumidity =
      state["targetHumidity"].as<float>();
  }

  if (state.containsKey("humidityTolerance")) {

    humidityTolerance =
      fabs(
        state["humidityTolerance"].as<float>()
      );
  }

  if (state.containsKey("gasThresholdHigh")) {

    gasThresholdHigh =
      state["gasThresholdHigh"].as<int>();
  }

  if (state.containsKey("gasThresholdLow")) {

    gasThresholdLow =
      state["gasThresholdLow"].as<int>();
  }

  if (state.containsKey("mode")) {

    const char* modeValue =
      state["mode"];

    if (modeValue != nullptr) {

      String mode(modeValue);

      mode.toUpperCase();

      if (mode == "MANUAL") {
        currentMode = MODE_MANUAL;
      } else {
        currentMode = MODE_AUTO;
      }
    }
  }

  gasHighStartTime = 0;
  gasLowStartTime = 0;

  Serial.println("Shadow state applied.");
}

// ============================================================
// REPORT SHADOW
// ============================================================

void reportShadowState() {

  if (!mqttClient.connected()) {
    return;
  }

  char payload[800];

  snprintf(
    payload,
    sizeof(payload),

    "{"
      "\"state\":{"
        "\"reported\":{"
          "\"mode\":\"%s\","
          "\"targetTemperature\":%.2f,"
          "\"temperatureTolerance\":%.2f,"
          "\"targetHumidity\":%.2f,"
          "\"humidityTolerance\":%.2f,"
          "\"gasThresholdHigh\":%d,"
          "\"gasThresholdLow\":%d,"
          "\"peltier\":%s,"
          "\"humidifier\":%s,"
          "\"humidifierFan\":%s,"
          "\"exhaust\":%s,"
          "\"exhaustDoorOpen\":%s"
        "}"
      "}"
    "}",

    currentMode == MODE_AUTO
      ? "AUTO"
      : "MANUAL",

    targetTemperature,
    temperatureTolerance,

    targetHumidity,
    humidityTolerance,

    gasThresholdHigh,
    gasThresholdLow,

    peltierState ? "true" : "false",
    humidifierState ? "true" : "false",
    humidifierFanState ? "true" : "false",
    exhaustState ? "true" : "false",
    exhaustDoorOpen ? "true" : "false"
  );

  mqttClient.publish(
    SHADOW_UPDATE,
    payload,
    false
  );
}

// ============================================================
// MQ3 CALIBRATION
// ============================================================

void calibrateMQ3() {

  Serial.println("=== MQ3 CALIBRATION ===");

  const int samples = 50;
  long sum = 0;
  int validSamples = 0;

  for (
    int i = 0;
    i < samples;
    i++
  ) {

    int reading =
      analogRead(MQ3_PIN);

    // Reject obvious ADC glitches
    if (
      reading > 50 &&
      reading < 4050
    ) {

      sum += reading;
      validSamples++;
    }

    delay(50);
  }

  if (validSamples == 0) {

    Serial.println(
      "ERROR: No valid MQ3 calibration samples."
    );

    gasCalibrated = false;
    return;
  }

  float avgRaw =
    sum / (float)validSamples;

  float voltage =
    avgRaw *
    (3.3 / 4095.0);

  if (voltage <= 0) {

    gasCalibrated = false;
    return;
  }

  float Rs =
    MQ3_RL *
    (
      (VCC - voltage) /
      voltage
    );

  if (
    !isfinite(Rs) ||
    Rs <= 0
  ) {

    gasCalibrated = false;
    return;
  }

  gasRo = Rs;
  gasCalibrated = true;

  Serial.printf(
    "Average Raw: %.0f\n",
    avgRaw
  );

  Serial.printf(
    "Ro: %.2f kOhm\n",
    gasRo
  );

  Serial.println(
    "MQ3 calibration complete."
  );
}

// ============================================================
// SENSOR READING
// ============================================================

void readSensors() {

  // ----------------------------------------------------------
  // DHT22
  // ----------------------------------------------------------

  float temp =
    dht.readTemperature();

  float hum =
    dht.readHumidity();

  if (
    isnan(temp) ||
    isnan(hum)
  ) {

    dhtValid = false;

    Serial.println(
      "DHT22 read error on GPIO32!"
    );

  } else {

    temperature = temp;
    humidity = hum;
    dhtValid = true;
  }

  // ----------------------------------------------------------
  // MQ3
  // ----------------------------------------------------------

  gasRaw =
    analogRead(MQ3_PIN);

  unsigned long warmupElapsedMs =
    millis() -
    mq3StartTime;

  if (
    warmupElapsedMs <
    MQ3_WARMUP_TIME
  ) {

    mq3WarmedUp = false;

    gasFiltered = 0;
    gasPPM = 0.0;

    return;
  }

  mq3WarmedUp = true;

  // ----------------------------------------------------------
  // REJECT ADC GLITCHES
  // ----------------------------------------------------------

  if (
    gasRaw <= 50 ||
    gasRaw >= 4050
  ) {

    Serial.printf(
      "MQ3 ADC glitch ignored: %d\n",
      gasRaw
    );

    /*
     * IMPORTANT:
     * Keep previous gasFiltered value.
     */

    return;
  }

  gasVoltage =
    gasRaw *
    (3.3 / 4095.0);

  if (gasVoltage <= 0) {
    return;
  }

  gasRs =
    MQ3_RL *
    (
      (VCC - gasVoltage) /
      gasVoltage
    );

  if (
    !isfinite(gasRs) ||
    gasRs <= 0
  ) {

    return;
  }

  // Only VALID samples reach the moving average
  gasFiltered =
    (int)mq3Filter.addSample(
      gasRaw
    );

  if (
    gasCalibrated &&
    gasRo > 0 &&
    gasRs > 0
  ) {

    float ratio =
      gasRs / gasRo;

    if (
      isfinite(ratio) &&
      ratio > 0
    ) {

      gasPPM =
        MQ3_A *
        pow(
          ratio,
          MQ3_B
        );

      if (
        !isfinite(gasPPM) ||
        gasPPM < 0
      ) {

        gasPPM = 0;
      }
    }

  } else {

    gasPPM = 0;
  }
}

// ============================================================
// TEMPERATURE CONTROL
// ============================================================

void controlTemperature() {

  if (
    currentMode != MODE_AUTO ||
    !dhtValid
  ) {

    return;
  }

  float upperLimit =
    targetTemperature +
    temperatureTolerance;

  float lowerLimit =
    targetTemperature -
    temperatureTolerance;

  if (
    temperature >
    upperLimit
  ) {

    if (!peltierState) {

      setRelay(
        RELAY_PELTIER,
        true
      );

      Serial.println(
        "Peltier ON: temperature high."
      );
    }
  }

  else if (
    temperature <
    lowerLimit
  ) {

    if (peltierState) {

      setRelay(
        RELAY_PELTIER,
        false
      );

      Serial.println(
        "Peltier OFF: lower limit reached."
      );
    }
  }
}

// ============================================================
// HUMIDITY CONTROL
// ============================================================

void controlHumidity() {

  if (
    currentMode != MODE_AUTO ||
    !dhtValid
  ) {

    return;
  }

  float upperLimit =
    targetHumidity +
    humidityTolerance;

  float lowerLimit =
    targetHumidity -
    humidityTolerance;

  if (
    humidity <
    lowerLimit
  ) {

    if (!humidifierState) {

      setHumidifier(true);

      Serial.println(
        "Humidification ON: humidity low."
      );
    }
  }

  else if (
    humidity >
    upperLimit
  ) {

    if (humidifierState) {

      setHumidifier(false);

      Serial.println(
        "Humidification OFF: humidity high."
      );
    }
  }
}

// ============================================================
// GAS CONTROL
// ============================================================

void controlGas() {

  if (
    currentMode != MODE_AUTO
  ) {

    gasHighStartTime = 0;
    gasLowStartTime = 0;

    return;
  }

  if (!mq3WarmedUp) {

    gasHighStartTime = 0;
    gasLowStartTime = 0;

    return;
  }

  int gasValue =
    gasCalibrated
      ? (int)gasPPM
      : gasFiltered;

  controlExhaustByGas(
    gasValue
  );
}

// ============================================================
// STABLE MQ3 EXHAUST CONTROL
// ============================================================

void controlExhaustByGas(
  int gasValue
) {

  unsigned long now =
    millis();

  // ==========================================================
  // EXHAUST OFF
  // ==========================================================

  if (!exhaustState) {

    gasLowStartTime = 0;

    if (
      gasValue >=
      gasThresholdHigh
    ) {

      if (
        gasHighStartTime == 0
      ) {

        gasHighStartTime = now;

        Serial.printf(
          "MQ3 HIGH: %d >= %d. Stability timer started.\n",
          gasValue,
          gasThresholdHigh
        );
      }

      unsigned long elapsed =
        now -
        gasHighStartTime;

      if (
        elapsed >=
        GAS_STABLE_TIME
      ) {

        Serial.println(
          "Gas remained HIGH for 10 seconds."
        );

        // Open door FIRST
        openExhaustDoor();

        delay(
          EXHAUST_DOOR_OPEN_DELAY
        );

        // Then start fan
        setRelay(
          RELAY_EXHAUST,
          true
        );

        Serial.println(
          "EXHAUST FAN ON."
        );

        gasHighStartTime = 0;
      }

    } else {

      // Value dropped below HIGH before timer completed
      if (
        gasHighStartTime != 0
      ) {

        Serial.println(
          "High gas timer cancelled."
        );
      }

      gasHighStartTime = 0;
    }
  }

  // ==========================================================
  // EXHAUST ON
  // ==========================================================

  else {

    gasHighStartTime = 0;

    if (
      gasValue <=
      gasThresholdLow
    ) {

      if (
        gasLowStartTime == 0
      ) {

        gasLowStartTime = now;

        Serial.printf(
          "MQ3 LOW: %d <= %d. Stability timer started.\n",
          gasValue,
          gasThresholdLow
        );
      }

      unsigned long elapsed =
        now -
        gasLowStartTime;

      if (
        elapsed >=
        GAS_STABLE_TIME
      ) {

        Serial.println(
          "Gas remained LOW for 10 seconds."
        );

        // Fan OFF first
        digitalWrite(
          RELAY_EXHAUST,
          HIGH
        );

        updateRelayStates();

        Serial.println(
          "EXHAUST FAN OFF."
        );

        // Wait for fan to slow down
        delay(
          EXHAUST_DOOR_CLOSE_DELAY
        );

        // Then close door
        closeExhaustDoor();

        gasLowStartTime = 0;
      }

    } else {

      // Gas increased again before timer completed
      if (
        gasLowStartTime != 0
      ) {

        Serial.println(
          "Low gas timer cancelled."
        );
      }

      gasLowStartTime = 0;
    }
  }
}

// ============================================================
// MANUAL EXHAUST CONTROL
// ============================================================

void manualExhaustControl(
  bool state
) {

  gasHighStartTime = 0;
  gasLowStartTime = 0;

  if (state) {

    if (exhaustState) {
      return;
    }

    Serial.println(
      "Manual exhaust ON."
    );

    // Door first
    openExhaustDoor();

    delay(
      EXHAUST_DOOR_OPEN_DELAY
    );

    // Fan second
    setRelay(
      RELAY_EXHAUST,
      true
    );

  } else {

    if (!exhaustState) {

      closeExhaustDoor();
      return;
    }

    Serial.println(
      "Manual exhaust OFF."
    );

    // Fan OFF first
    digitalWrite(
      RELAY_EXHAUST,
      HIGH
    );

    updateRelayStates();

    delay(
      EXHAUST_DOOR_CLOSE_DELAY
    );

    // Door second
    closeExhaustDoor();
  }
}

// ============================================================
// MIST BUTTON
// ============================================================

void pulseMistButton() {

  Serial.println(
    "Mist maker button PRESS"
  );

  digitalWrite(
    RELAY_MIST_BUTTON,
    LOW
  );

  delay(
    MIST_BUTTON_PULSE_TIME
  );

  digitalWrite(
    RELAY_MIST_BUTTON,
    HIGH
  );

  Serial.println(
    "Mist maker button RELEASE"
  );
}

// ============================================================
// HUMIDIFIER
// ============================================================

void setHumidifier(
  bool desiredState
) {

  if (
    humidifierState ==
    desiredState
  ) {

    setRelay(
      RELAY_HUMIDIFIER_FAN,
      desiredState
    );

    return;
  }

  if (desiredState) {

    Serial.println(
      "Starting humidification..."
    );

    pulseMistButton();

    humidifierState = true;

    setRelay(
      RELAY_HUMIDIFIER_FAN,
      true
    );

    Serial.println(
      "Mist maker ON."
    );

    Serial.println(
      "Humidifier fan ON."
    );

  } else {

    Serial.println(
      "Stopping humidification..."
    );

    pulseMistButton();

    humidifierState = false;

    setRelay(
      RELAY_HUMIDIFIER_FAN,
      false
    );

    Serial.println(
      "Mist maker OFF."
    );

    Serial.println(
      "Humidifier fan OFF."
    );
  }
}

// ============================================================
// EXHAUST SERVO
// ============================================================

void openExhaustDoor() {

  if (exhaustDoorOpen) {
    return;
  }

  Serial.println(
    "Opening exhaust door..."
  );

  exhaustDoorServo.write(
    EXHAUST_DOOR_OPEN_ANGLE
  );

  exhaustDoorOpen = true;

  Serial.printf(
    "Door OPEN: %d degrees\n",
    EXHAUST_DOOR_OPEN_ANGLE
  );
}

void closeExhaustDoor() {

  if (!exhaustDoorOpen) {
    return;
  }

  Serial.println(
    "Closing exhaust door..."
  );

  exhaustDoorServo.write(
    EXHAUST_DOOR_CLOSED_ANGLE
  );

  exhaustDoorOpen = false;

  Serial.printf(
    "Door CLOSED: %d degrees\n",
    EXHAUST_DOOR_CLOSED_ANGLE
  );
}

void updateExhaustDoor() {

  if (exhaustState) {
    openExhaustDoor();
  } else {
    closeExhaustDoor();
  }
}

// ============================================================
// RELAY CONTROL
// ============================================================

void setRelay(
  int pin,
  bool state
) {

  if (
    pin ==
    RELAY_MIST_BUTTON
  ) {

    Serial.println(
      "WARNING: Mist relay requires pulseMistButton()."
    );

    return;
  }

  digitalWrite(
    pin,
    state
      ? LOW
      : HIGH
  );

  updateRelayStates();

  if (
    pin ==
    RELAY_EXHAUST
  ) {

    updateExhaustDoor();
  }
}

// ============================================================
// UPDATE RELAY STATES
// ============================================================

void updateRelayStates() {

  peltierState =
    (
      digitalRead(
        RELAY_PELTIER
      ) == LOW
    );

  exhaustState =
    (
      digitalRead(
        RELAY_EXHAUST
      ) == LOW
    );

  humidifierFanState =
    (
      digitalRead(
        RELAY_HUMIDIFIER_FAN
      ) == LOW
    );
}

// ============================================================
// AWS TELEMETRY
// ============================================================

void sendAWS() {

  if (
    !wifiConnected ||
    !mqttClient.connected()
  ) {

    return;
  }

  char telemetry[600];

  snprintf(
    telemetry,
    sizeof(telemetry),

    "{"
      "\"temperature\":%.2f,"
      "\"humidity\":%.2f,"
      "\"dht_valid\":%s,"
      "\"mq3_raw\":%d,"
      "\"mq3_filtered\":%d,"
      "\"mq3_ppm\":%.3f,"
      "\"mq3_calibrated\":%s,"
      "\"mq3_warmed_up\":%s"
    "}",

    dhtValid ? temperature : 0.0,
    dhtValid ? humidity : 0.0,

    dhtValid
      ? "true"
      : "false",

    gasRaw,
    gasFiltered,
    gasPPM,

    gasCalibrated
      ? "true"
      : "false",

    mq3WarmedUp
      ? "true"
      : "false"
  );

  mqttClient.publish(
    TOPIC_TELEMETRY,
    telemetry,
    false
  );

  char status[700];

  snprintf(
    status,
    sizeof(status),

    "{"
      "\"mode\":\"%s\","
      "\"relays\":{"
        "\"peltier\":%s,"
        "\"exhaust\":%s,"
        "\"humidifierFan\":%s,"
        "\"humidifier\":%s"
      "},"
      "\"exhaustDoor\":{"
        "\"open\":%s,"
        "\"angle\":%d"
      "},"
      "\"settings\":{"
        "\"targetTemperature\":%.2f,"
        "\"temperatureTolerance\":%.2f,"
        "\"targetHumidity\":%.2f,"
        "\"humidityTolerance\":%.2f,"
        "\"gasThresholdHigh\":%d,"
        "\"gasThresholdLow\":%d,"
        "\"gasStableTimeMs\":%lu"
      "}"
    "}",

    currentMode == MODE_AUTO
      ? "AUTO"
      : "MANUAL",

    peltierState ? "true" : "false",
    exhaustState ? "true" : "false",
    humidifierFanState ? "true" : "false",
    humidifierState ? "true" : "false",

    exhaustDoorOpen ? "true" : "false",

    exhaustDoorOpen
      ? EXHAUST_DOOR_OPEN_ANGLE
      : EXHAUST_DOOR_CLOSED_ANGLE,

    targetTemperature,
    temperatureTolerance,

    targetHumidity,
    humidityTolerance,

    gasThresholdHigh,
    gasThresholdLow,

    GAS_STABLE_TIME
  );

  mqttClient.publish(
    TOPIC_STATUS,
    status,
    true
  );
}

// ============================================================
// OLED ICONS
// ============================================================

void drawWiFi(
  int x,
  int y
) {

  oled.drawCircle(
    x,
    y,
    8,
    U8G2_DRAW_UPPER_LEFT |
    U8G2_DRAW_UPPER_RIGHT
  );

  oled.drawCircle(
    x,
    y,
    5,
    U8G2_DRAW_UPPER_LEFT |
    U8G2_DRAW_UPPER_RIGHT
  );

  if (frame % 4 < 2) {

    oled.drawDisc(
      x,
      y,
      2
    );
  }
}

void drawSnowflake(
  int x,
  int y
) {

  oled.drawLine(x - 5, y, x + 5, y);
  oled.drawLine(x, y - 5, x, y + 5);
  oled.drawLine(x - 4, y - 4, x + 4, y + 4);
  oled.drawLine(x - 4, y + 4, x + 4, y - 4);

  oled.drawDisc(x, y, 1);
}

void drawDrop(
  int x,
  int y
) {

  oled.drawTriangle(
    x,
    y - 5,
    x - 3,
    y,
    x + 3,
    y
  );

  oled.drawCircle(
    x,
    y,
    3
  );
}

void drawFan(
  int x,
  int y
) {

  oled.drawCircle(
    x,
    y,
    7
  );

  for (
    int i = 0;
    i < 4;
    i++
  ) {

    float a =
      frame * 0.25 +
      i * 1.57;

    int px =
      x +
      cos(a) * 4;

    int py =
      y +
      sin(a) * 4;

    oled.drawDisc(
      px,
      py,
      2
    );
  }

  oled.drawDisc(
    x,
    y,
    1
  );
}

// ============================================================
// OLED
// ============================================================

void updateOLED() {

  oled.clearBuffer();

  oled.setFont(
    u8g2_font_6x10_tr
  );

  oled.drawStr(
    2,
    9,
    "FRUIT CHAMBER"
  );

  if (wifiConnected) {
    drawWiFi(119, 8);
  }

  oled.drawHLine(
    0,
    12,
    128
  );

  // Temperature
  oled.setFont(
    u8g2_font_logisoso24_tf
  );

  char tempStr[10];

  if (dhtValid) {

    snprintf(
      tempStr,
      sizeof(tempStr),
      "%.1f",
      temperature
    );

  } else {

    snprintf(
      tempStr,
      sizeof(tempStr),
      "--.-"
    );
  }

  oled.drawStr(
    5,
    40,
    tempStr
  );

  oled.setFont(
    u8g2_font_6x10_tr
  );

  oled.drawStr(
    67,
    38,
    "C"
  );

  oled.drawFrame(
    82,
    20,
    5,
    14
  );

  oled.drawDisc(
    84,
    37,
    4
  );

  // Humidity
  oled.drawStr(
    5,
    55,
    "H:"
  );

  char humStr[10];

  if (dhtValid) {

    snprintf(
      humStr,
      sizeof(humStr),
      "%.0f%%",
      humidity
    );

  } else {

    snprintf(
      humStr,
      sizeof(humStr),
      "--%%"
    );
  }

  oled.drawStr(
    18,
    55,
    humStr
  );

  // MQ3
  oled.drawStr(
    48,
    55,
    "MQ:"
  );

  char mqStr[20];

  if (!mq3WarmedUp) {

    unsigned long elapsed =
      millis() -
      mq3StartTime;

    unsigned long remaining = 0;

    if (
      elapsed <
      MQ3_WARMUP_TIME
    ) {

      remaining =
        (
          MQ3_WARMUP_TIME -
          elapsed
        ) /
        1000UL;
    }

    snprintf(
      mqStr,
      sizeof(mqStr),
      "W%luS",
      remaining
    );

  } else if (
    gasCalibrated &&
    gasPPM > 0
  ) {

    snprintf(
      mqStr,
      sizeof(mqStr),
      "%.0fppm",
      gasPPM
    );

  } else {

    snprintf(
      mqStr,
      sizeof(mqStr),
      "%d",
      gasFiltered
    );
  }

  oled.drawStr(
    70,
    55,
    mqStr
  );

  if (peltierState) {
    drawSnowflake(100, 28);
  }

  if (exhaustState) {
    drawFan(116, 43);
  }

  if (humidifierState) {

    int dropY =
      48 +
      (frame % 8);

    drawDrop(
      94,
      dropY
    );
  }

  oled.sendBuffer();
}

// ============================================================
// SERIAL
// ============================================================

void printSerial() {

  Serial.println(
    "========================================"
  );

  Serial.printf(
    "Temperature: %.1f C\n",
    temperature
  );

  Serial.printf(
    "Humidity: %.1f %%\n",
    humidity
  );

  Serial.printf(
    "Gas Raw: %d\n",
    gasRaw
  );

  Serial.printf(
    "Gas Filtered: %d\n",
    gasFiltered
  );

  Serial.printf(
    "Gas PPM: %.2f\n",
    gasPPM
  );

  Serial.printf(
    "MQ3 Warm-up: %s\n",
    mq3WarmedUp
      ? "Complete"
      : "In Progress"
  );

  Serial.println(
    "----------------------------------------"
  );

  Serial.printf(
    "Peltier: %s\n",
    peltierState
      ? "ON"
      : "OFF"
  );

  Serial.printf(
    "Exhaust Fan: %s\n",
    exhaustState
      ? "ON"
      : "OFF"
  );

  Serial.printf(
    "Exhaust Door: %s\n",
    exhaustDoorOpen
      ? "OPEN"
      : "CLOSED"
  );

  Serial.printf(
    "Humidifier Fan: %s\n",
    humidifierFanState
      ? "ON"
      : "OFF"
  );

  Serial.printf(
    "Mist Maker: %s\n",
    humidifierState
      ? "ON"
      : "OFF"
  );

  Serial.println(
    "----------------------------------------"
  );

  Serial.printf(
    "Gas HIGH threshold: %d\n",
    gasThresholdHigh
  );

  Serial.printf(
    "Gas LOW threshold: %d\n",
    gasThresholdLow
  );

  // Show exhaust stability countdown
  if (
    !exhaustState &&
    gasHighStartTime != 0
  ) {

    unsigned long elapsed =
      millis() -
      gasHighStartTime;

    unsigned long remaining =
      (
        elapsed >= GAS_STABLE_TIME
      )
        ? 0
        : (
            GAS_STABLE_TIME -
            elapsed
          ) / 1000UL;

    Serial.printf(
      "Exhaust ON stability countdown: %lu sec\n",
      remaining
    );
  }

  if (
    exhaustState &&
    gasLowStartTime != 0
  ) {

    unsigned long elapsed =
      millis() -
      gasLowStartTime;

    unsigned long remaining =
      (
        elapsed >= GAS_STABLE_TIME
      )
        ? 0
        : (
            GAS_STABLE_TIME -
            elapsed
          ) / 1000UL;

    Serial.printf(
      "Exhaust OFF stability countdown: %lu sec\n",
      remaining
    );
  }

  Serial.println(
    "----------------------------------------"
  );

  Serial.printf(
    "Mode: %s\n",
    currentMode == MODE_AUTO
      ? "AUTO"
      : "MANUAL"
  );

  Serial.printf(
    "WiFi: %s\n",
    WiFi.status() == WL_CONNECTED
      ? "Connected"
      : "Disconnected"
  );

  Serial.printf(
    "AWS IoT: %s\n",
    mqttClient.connected()
      ? "Connected"
      : "Disconnected"
  );

  Serial.println(
    "========================================"
  );

  Serial.println();
}
