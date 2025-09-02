#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <math.h>
#include "ACS712.h"
#include "secrets.h"

// ============================== Build-Time Configuration ==============================

// -------- Pins --------
static constexpr int PIN_ADC_VRMS = 34;   // ADC1_CH6 (safe on ADC1)
static constexpr int PIN_ACS712   = 32;   // ACS712 analog out

// -------- Sampling & maths --------
static constexpr uint16_t SAMPLES_PER_WINDOW   = 1000;   // samples per RMS window
static constexpr uint32_t SAMPLE_HZ            = 2000;   // sampling rate (Hz) »> > 2x mains
static constexpr float    MAINS_F_HZ           = 50.0f;  // Nigeria: 50 Hz
static_assert(SAMPLES_PER_WINDOW >= 10, "SAMPLES_PER_WINDOW too small");

// -------- Scaling factors --------
// Divider ratio: (Rtop + Rbottom) / Rbottom  e.g. (100k + 10k)/10k = 11
static constexpr float VOLTAGE_DIVIDER_RATIO = 11.0f;
// Transformer primary/secondary ratio: 230V -> 9V => 230/9
static constexpr float TRANSFORMER_RATIO = (230.0f / 9.0f);

// -------- ADC (ESP32) --------
static constexpr float ADC_VREF      = 3.3f;     // for reference; we’ll prefer analogReadMilliVolts
static constexpr float ADC_COUNTS    = 4095.0f;  // 12-bit
static constexpr adc_attenuation_t ADC_ATTEN = ADC_11db; // 0–~3.3V usable

// -------- ACS712 current sensor setup --------
// Constructor: ACS712(analogPin, vRef, ADC resolution, sensitivity mV/A)
// Your library usage indicated mA_AC(), we’ll keep it and wrap with simple post-cal.
ACS712 g_acs(PIN_ACS712, 3.3, 4095, 185);
// Empirical calibration offset (mA). Adjust after a few no-load tests:
static int g_current_cal_mA_offset = 175;
// Deadband to suppress noise near 0 mA:
static int g_current_deadband_mA   = 5;

// -------- Tasking --------
static constexpr TickType_t SENSOR_PERIOD_MS = 2000;  // every 2s

// ============================== Types & Globals ==============================

struct EnergyMetrics {
  float voltage; // Vrms (V)
  float current; // Irms (mA)
  float power;   // P ≈ V*I (W)
};

WiFiClientSecure g_tlsClient;
PubSubClient     g_mqttClient(g_tlsClient);
QueueHandle_t    g_metricsQueue = nullptr;  // size = 1 (overwrite)

// Pre-allocated buffer for one RMS window (voltage samples in volts)
static float g_vSamples[SAMPLES_PER_WINDOW];

// ============================== Utility / Helpers ==============================
static void collectVoltageWindow()
{
  const uint32_t usPerSample = 1000000UL / SAMPLE_HZ;
  uint32_t tStart = micros();
  uint32_t nextT  = tStart;

  // 1) Grab raw samples as volts (DC included)
  float sum = 0.0f;
  for (uint16_t i = 0; i < SAMPLES_PER_WINDOW; i++) {
    // Wait until our next slot
    while ((int32_t)(micros() - nextT) < 0) { /* spin */ }
    nextT += usPerSample;

    // Read calibrated millivolts (ADC1 only)
    // NOTE: On some cores, analogReadMilliVolts requires `analogSetPinAttenuation`.
    int mv = analogReadMilliVolts(PIN_ADC_VRMS);
    float volts = mv / 1000.0f;

    g_vSamples[i] = volts;  // store temporarily with DC
    sum += volts;
  }

  // 2) Remove measured DC mean across the window
  const float mean = sum / SAMPLES_PER_WINDOW;
  for (uint16_t i = 0; i < SAMPLES_PER_WINDOW; i++) {
    g_vSamples[i] -= mean;  // AC-coupled
  }
}

/**
 * @brief Compute RMS of the already AC-coupled buffer (g_vSamples).
 */
static float computeRMS_Volts()
{
  double acc = 0.0; // double for numeric stability on large N
  for (uint16_t i = 0; i < SAMPLES_PER_WINDOW; i++) {
    acc += (double)g_vSamples[i] * (double)g_vSamples[i];
  }
  return sqrt(acc / (double)SAMPLES_PER_WINDOW);
}

/**
 * @brief Convert secondary Vrms (ADC node) to estimated mains Vrms.
 * Vrms_mains = Vrms_adc * DIVIDER_RATIO * TRANSFORMER_RATIO
 */
static float scaleToMainsVrms(float vrms_adc_volts)
{
  return vrms_adc_volts * VOLTAGE_DIVIDER_RATIO * TRANSFORMER_RATIO;
}

/**
 * @brief Read AC current (mA RMS) using ACS712 helper, then apply calibration and deadband.
 * NOTE: This uses the same approach you had (`mA_AC()` averaged) to match your library.
 *       For best accuracy, prefer a sample-based RMS (like voltage) and calibrate offset.
 */
static float readCurrent_mA()
{
  float acc = 0.0f;
  constexpr int N = 100;
  for (int i = 0; i < N; i++) {
    acc += g_acs.mA_AC();  // library-provided AC measurement (avg over short window)
  }
  float mA = fabs(acc / N) - g_current_cal_mA_offset;
  if (mA < g_current_deadband_mA) mA = 0.0f;
  return mA;
}

// ============================== Wi-Fi & MQTT ==============================

static void wifiConnectBlocking()
{
  if (WiFi.status() == WL_CONNECTED) return;
  Serial.print("WiFi: connecting to ");
  Serial.print(ssid);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(400);
    Serial.print('.');
  }
  Serial.printf("\nWiFi: connected, IP=%s\n", WiFi.localIP().toString().c_str());
}

static void mqttCallback(char* topic, byte* payload, unsigned int length)
{
  // Optional: handle downstream commands here
  String msg; msg.reserve(length);
  for (unsigned i = 0; i < length; ++i) msg += (char)payload[i];
  Serial.printf("MQTT <- [%s]: %s\n", topic, msg.c_str());
}

static void mqttEnsureConnected()
{
  if (g_mqttClient.connected()) return;

  Serial.print("MQTT: connecting...");
  // Client ID: chip ID based
  String cid = "ESP32Client-" + String((uint32_t)ESP.getEfuseMac(), HEX);

  if (g_mqttClient.connect(cid.c_str(), mqtt_user, mqtt_pass)) {
    Serial.println("ok");
    if (subscribeTopic && strlen(subscribeTopic) > 0) {
      g_mqttClient.subscribe(subscribeTopic);
      Serial.printf("MQTT: subscribed to %s\n", subscribeTopic);
    }
  } else {
    Serial.printf("failed, rc=%d\n", g_mqttClient.state());
  }
}

static bool mqttPublishMetrics(const EnergyMetrics& m)
{
  StaticJsonDocument<160> doc;
  doc["voltage"] = m.voltage;       // V
  doc["current"] = m.current;       // mA
  doc["power"]   = m.power;         // W

  char buf[160];
  const size_t n = serializeJson(doc, buf, sizeof(buf));
  if (n == 0) return false;

  const bool ok = g_mqttClient.publish(publishTopic, (const uint8_t*)buf, n);
  if (ok) {
    Serial.print("MQTT -> ");
    Serial.println(buf);
  }
  return ok;
}

// ============================== Tasks ==============================

/**
 * @brief Sensor Task: sample ADC, compute Vrms, read Irms, compute power, queue it.
 * Pinned to APP_CPU (core 1).
 */
static void sensorTask(void* arg)
{
  (void)arg;
  Serial.printf("Sensor task on core %d\n", xPortGetCoreID());

  for (;;) {
    EnergyMetrics m{};

    // --- Voltage ---
    collectVoltageWindow();
    const float vrms_adc = computeRMS_Volts();
    m.voltage            = scaleToMainsVrms(vrms_adc);  // Vrms at mains

    // --- Current ---
    m.current = readCurrent_mA(); // Irms in mA

    // --- Power (no phase angle compensation) ---
    m.power = (m.voltage * (m.current / 1000.0f)); // W

    // Local print
    Serial.printf("Meter: V=%.1f V | I=%.0f mA | P=%.2f W\n", m.voltage, m.current, m.power);

    // Send latest to queue (overwrite semantics)
    if (g_metricsQueue) xQueueOverwrite(g_metricsQueue, &m);

    vTaskDelay(pdMS_TO_TICKS(SENSOR_PERIOD_MS));
  }
}

/**
 * @brief Cloud Task: keeps Wi-Fi/MQTT alive and publishes latest metrics from queue.
 * Pinned to PRO_CPU (core 0).
 */
static void cloudTask(void* arg)
{
  (void)arg;
  Serial.printf("Cloud task on core %d\n", xPortGetCoreID());

  for (;;) {
    // 1) Connectivity
    wifiConnectBlocking();
    if (!g_mqttClient.connected()) mqttEnsureConnected();
    g_mqttClient.loop();

    // 2) Publish when we get fresh data
    EnergyMetrics m;
    if (xQueueReceive(g_metricsQueue, &m, pdMS_TO_TICKS(5000)) == pdTRUE) {
      if (!mqttPublishMetrics(m)) {
        Serial.println("MQTT: publish failed");
      }
    }

    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

// ============================== Setup & Loop ==============================

void setup()
{
  Serial.begin(115200);
  delay(100);

  // --- ADC config ---
  analogReadResolution(12);
  analogSetAttenuation(ADC_ATTEN);            // global default
  analogSetPinAttenuation(PIN_ADC_VRMS, ADC_ATTEN); // ensure pin-level setting (needed on some cores)

  // --- Wi-Fi/MQTT base config ---
  g_tlsClient.setInsecure();  // ⚠ disables TLS cert validation; replace with proper CA in production
  g_mqttClient.setServer(mqtt_server, mqtt_port);
  g_mqttClient.setCallback(mqttCallback);

  // --- Queue (size=1, overwrite) ---
  g_metricsQueue = xQueueCreate(1, sizeof(EnergyMetrics));

  // --- Tasks ---
  xTaskCreatePinnedToCore(sensorTask, "SensorTask", 4096, nullptr, 2, nullptr, 1); // core 1
  xTaskCreatePinnedToCore(cloudTask,  "CloudTask",  8192, nullptr, 1, nullptr, 0); // core 0

  Serial.println("Energy Monitor: started.");
}

void loop()
{
  // Nothing—work happens in tasks
  vTaskDelay(pdMS_TO_TICKS(1000));
}
