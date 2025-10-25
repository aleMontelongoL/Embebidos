/****************************************
 * Broker MQTT del carro (PWM)
 ****************************************/
const char* MQTT_HOST = "test.mosquitto.org";
const uint16_t MQTT_PORT = 1883;

/****************************************
 * Librerías
 ****************************************/
#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>

#include <Adafruit_MLX90614.h>  // MLX90614 (Wire)
#include "MAX30105.h"           // MAX30102 (SparkFun)
#include "spo2_algorithm.h"     // Define FreqS=25 y BUFFER_SIZE=100

/****************************************
 * Wi-Fi (usa red de Fercio)
 ****************************************/
const char* WIFI_SSID = "Fercio";
const char* WIFI_PASS = "TW1A2P3D";

/****************************************
 * MQTT (anónimo, igual que el carro)
 ****************************************/
const char* MQTT_CLIENT_ID = "r2d2";
WiFiClient   espClient;
PubSubClient client(espClient);

// Tópicos de salud (se conservan)
const char* TOPIC_HEALTH  = "esp32/health";
const char* TOPIC_HR      = "esp32/health/hr";
const char* TOPIC_SPO2    = "esp32/health/spo2";
const char* TOPIC_TA      = "esp32/health/ta";
const char* TOPIC_TO      = "esp32/health/to";

// Tópicos del carro con PWM (compatibles)
const char* TOPIC_CMD     = "car/cmd";    // p.ej. "FWD","REV","STOP"
const char* TOPIC_SPEED   = "car/speed";  // p.ej. "0..100"
const char* TOPIC_STATE   = "car/state";  // heartbeat/estado

/****************************************
 * I²C único (Wire)
 ****************************************/
#define I2C_SDA    21
#define I2C_SCL    22
#define I2C_FREQ   100000   // 100 kHz (más estable para MLX)

/****************************************
 * Sensores
 ****************************************/
Adafruit_MLX90614 mlx = Adafruit_MLX90614(); // 0x5A
bool mlxOK = false;

MAX30105 particleSensor;
bool maxOK = false;

// Del algoritmo
#define SAMPLE_DELAY_MS (1000 / FreqS)   // ~40 ms (25 Hz)

/****************************************
 * Tiempo
 ****************************************/
uint32_t lastUpdateMs = 0;
const uint32_t UPDATE_MS = 2000;
uint32_t lastRSSI = 0;

/****************************************
 * Utilidades sensores
 ****************************************/
void scanI2C(TwoWire &bus, const char* name) {
  Serial.printf("\n[SCAN] %s\n", name);
  uint8_t found = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    bus.beginTransmission(addr);
    uint8_t err = bus.endTransmission();
    if (err == 0) {
      Serial.printf("  - Dispositivo en 0x%02X\n", addr);
      found++;
    }
  }
  if (!found) Serial.println("  (sin dispositivos)");
}

bool initMLX() {
  for (int i = 0; i < 3; i++) {
    if (mlx.begin(0x5A, &Wire)) return true;
    delay(50);
  }
  return false;
}

bool initMAX30102() {
  if (!particleSensor.begin(Wire, I2C_SPEED_STANDARD)) { // ~100 kHz
    return false;
  }
  byte ledBrightness = 0x1F;
  byte sampleAverage = 4;
  byte ledMode = 2;          // Rojo + IR
  int  sampleRate   = 100;   // Hz
  int  pulseWidth   = 411;   // us
  int  adcRange     = 16384; // rango
  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);
  particleSensor.setPulseAmplitudeRed(0x1F);
  particleSensor.setPulseAmplitudeIR(0x1F);
  particleSensor.setPulseAmplitudeGreen(0);
  return true;
}

bool readHRSpO2(int32_t &hr, int8_t &hrValid, int32_t &spo2, int8_t &spo2Valid) {
  if (!maxOK) { hr=-1; hrValid=0; spo2=-1; spo2Valid=0; return false; }
  uint32_t irBuffer[BUFFER_SIZE];
  uint32_t redBuffer[BUFFER_SIZE];

  for (int i = 0; i < BUFFER_SIZE; i++) {
    uint32_t t0 = millis();
    while (!particleSensor.available()) {
      particleSensor.check();
      if (millis() - t0 > 2000) return false;
      delay(1);
    }
    redBuffer[i] = particleSensor.getRed();
    irBuffer[i]  = particleSensor.getIR();
    particleSensor.nextSample();

    uint32_t t1 = millis();
    while (millis() - t1 < SAMPLE_DELAY_MS) {
      particleSensor.check();
      delay(1);
    }
  }

  maxim_heart_rate_and_oxygen_saturation(
    irBuffer, BUFFER_SIZE, redBuffer,
    &spo2, &spo2Valid, &hr, &hrValid);

  return (hrValid == 1 || spo2Valid == 1);
}

bool readTemps(double &Ta, double &To) {
  if (!mlxOK) { Ta = NAN; To = NAN; return false; }
  Ta = mlx.readAmbientTempC();
  To = mlx.readObjectTempC();
  return isfinite(Ta) && isfinite(To);
}

/****************************************
 * Wi-Fi robusto (state machine + BSSID opcional)
 ****************************************/
enum class WifiState : uint8_t { IDLE, CONNECTING, CONNECTED };
volatile WifiState wifiState = WifiState::IDLE;

uint8_t targetBSSID[6] = {0};
int     targetChannel  = 0;
bool    useBSSIDLock   = true;  // si fallan auth/handshake, lo desactivamos temporalmente

const char* reasonText(uint8_t r){
  switch(r){
    case 1:  return "UNSPECIFIED";
    case 2:  return "AUTH_EXPIRE";
    case 3:  return "AUTH_LEAVE";
    case 4:  return "ASSOC_EXPIRE";
    case 5:  return "ASSOC_TOOMANY";
    case 6:  return "NOT_AUTHED";
    case 7:  return "NOT_ASSOCED";
    case 8:  return "ASSOC_LEAVE";
    case 9:  return "ASSOC_NOT_AUTHED";
    case 15: return "4WAY_HANDSHAKE_TIMEOUT";
    case 17: return "IE_INVALID";
    case 23: return "MIC_FAILURE";
    case 24: return "4WAY_HANDSHAKE_TIMEOUT2";
    case 29: return "ASSOC_FAIL";
    case 201: return "NO_AP_FOUND";
    case 202: return "AUTH_FAIL";
    case 203: return "ASSOC_FAIL2";
    case 204: return "HANDSHAKE_TIMEOUT";
    default: return "UNKNOWN";
  }
}

void wifiFindAP(const char* ssid) {
  targetChannel = 0;
  memset(targetBSSID, 0, sizeof(targetBSSID));
  Serial.println("[WiFi] Escaneando AP objetivo...");
  int n = WiFi.scanNetworks(false, true);
  if (n <= 0) { Serial.println("[WiFi] No se encontraron redes"); return; }
  int best = -1000; int idxSel = -1;
  for (int i=0;i<n;i++){
    if (WiFi.SSID(i) == ssid) {
      int rssi = WiFi.RSSI(i);
      if (rssi > best) { best = rssi; idxSel = i; }
    }
  }
  if (idxSel >= 0) {
    targetChannel = WiFi.channel(idxSel);
    auto b = WiFi.BSSID(idxSel);
    memcpy(targetBSSID, b, 6);
    Serial.printf("[WiFi] AP '%s' encontrado. Canal=%d RSSI=%d dBm BSSID=%02X:%02X:%02X:%02X:%02X:%02X\n",
                  ssid, targetChannel, best,
                  targetBSSID[0],targetBSSID[1],targetBSSID[2],
                  targetBSSID[3],targetBSSID[4],targetBSSID[5]);
  } else {
    Serial.println("[WiFi] No se encontró el SSID objetivo en el escaneo.");
  }
}

void onWiFiEvent(WiFiEvent_t event, WiFiEventInfo_t info){
  switch(event){
    case ARDUINO_EVENT_WIFI_STA_START:
      Serial.println("[WiFi] STA_START");
      break;
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      wifiState = WifiState::CONNECTED;
      Serial.printf("[WiFi] GOT_IP: %s (RSSI %d dBm)\n",
                    WiFi.localIP().toString().c_str(), WiFi.RSSI());
      break;
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED: {
      wifiState = WifiState::IDLE;
      uint8_t r = info.wifi_sta_disconnected.reason;
      Serial.printf("[WiFi] DISCONNECTED reason=%u (%s)\n", r, reasonText(r));
      if (r == 15 || r == 202 || r == 204) {
        useBSSIDLock = false;
      }
      break;
    }
    default: break;
  }
}

void wifiInitStation(){
  WiFi.mode(WIFI_STA);
  WiFi.persistent(false);
  WiFi.setSleep(false);
  WiFi.setAutoReconnect(false);       // reconexión manual con backoff
  WiFi.onEvent(onWiFiEvent);
  WiFi.setTxPower(WIFI_POWER_19_5dBm); // potencia máxima permitida
}

void wifiStartConnect(){
  if (wifiState == WifiState::CONNECTING) return;

  WiFi.disconnect(true, true);
  delay(50);

  if (useBSSIDLock && targetChannel > 0 && targetBSSID[0] != 0) {
    Serial.printf("[WiFi] Conectando a \"%s\" anclado a canal %d / BSSID ...\n", WIFI_SSID, targetChannel);
    WiFi.begin(WIFI_SSID, WIFI_PASS, targetChannel, targetBSSID, true);
  } else {
    Serial.printf("[WiFi] Conectando a \"%s\" (sin anclaje)...\n", WIFI_SSID);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
  }

  wifiState = WifiState::CONNECTING;
}

void wifiService(){
  static uint32_t nextAction = 0;
  static uint16_t backoffMs  = 1000;
  static uint32_t startedAt  = 0;

  if (millis() < nextAction) return;

  if (wifiState == WifiState::CONNECTED) {
    backoffMs = 1000;
    startedAt = 0;
    return;
  }

  if (wifiState == WifiState::CONNECTING) {
    if (startedAt == 0) startedAt = millis();
    if (millis() - startedAt > 8000) {
      Serial.println("[WiFi] Timeout conectando. Re-escanear y reintentar...");
      wifiFindAP(WIFI_SSID);
      if (useBSSIDLock == true) useBSSIDLock = false;
      wifiState = WifiState::IDLE;
      startedAt = 0;
      backoffMs = min<uint16_t>(backoffMs * 2, 30000);
      nextAction = millis() + backoffMs;
    } else {
      nextAction = millis() + 300;
    }
    return;
  }

  // IDLE -> nuevo intento
  if (useBSSIDLock && (targetChannel == 0 || targetBSSID[0] == 0)) {
    wifiFindAP(WIFI_SSID);
  }
  wifiStartConnect();
  nextAction = millis() + backoffMs;
  backoffMs = min<uint16_t>(backoffMs * 2, 30000);
}

/****************************************
 * MQTT helpers con backoff
 ****************************************/
void mqttCallback(char* topic, byte* payload, unsigned int length){
  // Solo imprime lo recibido en los tópicos del carro
  String msg; msg.reserve(length);
  for (unsigned int i=0;i<length;i++) msg += (char)payload[i];
  Serial.printf("[MQTT] %s => %s\n", topic, msg.c_str());
  // Aquí podrías mapear car/cmd y car/speed a actuadores si lo necesitas.
}

void mqttSetup(){
  client.setServer(MQTT_HOST, MQTT_PORT);
  client.setKeepAlive(30);
  client.setSocketTimeout(5);
  client.setBufferSize(512);
  client.setCallback(mqttCallback);
}

void mqttService(){
  static uint32_t nextTry = 0;
  static uint16_t backoff = 1000;

  if (wifiState != WifiState::CONNECTED) {
    nextTry = millis() + 1000;
    backoff = 1000;
    return;
  }

  if (client.connected()) {
    client.loop();
    backoff = 1000;
    return;
  }

  if (millis() < nextTry) return;

  Serial.print("[MQTT] Conectando...");

  // ID único (sin usuario/clave, igual que el carro)
  String clientId = String(MQTT_CLIENT_ID) + "-" + String((uint32_t)ESP.getEfuseMac(), HEX);
  bool ok = client.connect(clientId.c_str());

  if (ok) {
    Serial.println("OK");
    backoff = 1000;

    // Suscripciones compatibles con el carro
    client.subscribe(TOPIC_CMD);
    client.subscribe(TOPIC_SPEED);

    // Publica presencia/estado en el tópico del carro
    client.publish(TOPIC_STATE, "online", true);
  } else {
    Serial.printf("falló rc=%d\n", client.state());
    backoff = backoff < 30000 ? backoff * 2 : 30000;
  }
  nextTry = millis() + backoff;
}

/****************************************
 * Setup
 ****************************************/
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n[START] ESP32 Health + Wi-Fi/MQTT (compat car/PWM)");

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(I2C_FREQ);

  scanI2C(Wire, "Wire (SDA=21, SCL=22)");

  mlxOK = initMLX();
  Serial.println(mlxOK ? "MLX90614 OK (0x5A)" : "MLX90614 NO detectado");

  maxOK = initMAX30102();
  Serial.println(maxOK ? "MAX30102 OK (0x57 tipico)" : "MAX30102 NO detectado");

  wifiInitStation();
  wifiFindAP(WIFI_SSID);  // aprende canal/BSSID
  wifiStartConnect();     // primer intento

  mqttSetup();
}

/****************************************
 * Loop
 ****************************************/
void loop() {
  wifiService();
  mqttService();

  uint32_t now = millis();
  if (now - lastUpdateMs >= UPDATE_MS) {
    lastUpdateMs = now;

    double Ta = NAN, To = NAN;
    bool tempOK = readTemps(Ta, To);

    int32_t hr = -1; int8_t hrValid = 0;
    int32_t s2 = -1; int8_t s2Valid = 0;
    (void)readHRSpO2(hr, hrValid, s2, s2Valid);

    Serial.print("Ta=");
    if (tempOK) Serial.print(Ta, 1); else Serial.print("NaN");
    Serial.print("C | To=");
    if (tempOK) Serial.print(To, 1); else Serial.print("NaN");
    Serial.print("C  ||  HR=");
    Serial.print(hr);
    Serial.print(" bpm (");
    Serial.print(hrValid ? "ok" : "inv");
    Serial.print(") | SpO2=");
    Serial.print(s2);
    Serial.print("% (");
    Serial.print(s2Valid ? "ok" : "inv");
    Serial.println(")");

    if (client.connected()) {
      // JSON de salud
      char json[220];
      snprintf(json, sizeof(json),
               "{\"hr\":%ld,\"hrValid\":%d,\"spo2\":%ld,\"spo2Valid\":%d,\"Ta\":%.1f,\"To\":%.1f}",
               (long)hr, (int)hrValid, (long)s2, (int)s2Valid,
               tempOK ? Ta : -99.9, tempOK ? To : -99.9);
      client.publish(TOPIC_HEALTH, json, true);

      // Atajos numéricos
      char buf[32];
      snprintf(buf, sizeof(buf), "%ld", (long)hr); client.publish(TOPIC_HR, buf, true);
      snprintf(buf, sizeof(buf), "%ld", (long)s2); client.publish(TOPIC_SPO2, buf, true);
      dtostrf(tempOK ? Ta : -99.9, 0, 1, buf);     client.publish(TOPIC_TA, buf, true);
      dtostrf(tempOK ? To : -99.9, 0, 1, buf);     client.publish(TOPIC_TO, buf, true);

      // Heartbeat/estado compatible con el carro
      String hb = String("rssi=") + String(WiFi.RSSI()) + "dBm";
      client.publish(TOPIC_STATE, hb.c_str(), false);
    }
  }

  // Telemetría de señal cada 5 s
  if (wifiState == WifiState::CONNECTED && millis() - lastRSSI > 5000) {
    lastRSSI = millis();
    Serial.printf("[WiFi] RSSI: %d dBm\n", WiFi.RSSI());
  }
}