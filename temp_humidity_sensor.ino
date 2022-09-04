#include <Wire.h>
#include "OneButton.h"
#include "src/WiFiManager/WiFiManager.h" // https://github.com/tzapu/WiFiManager
#include "PubSubClient.h"

#define HTU21D_I2C_ADDRESS 0x40

#define HTU21D_RES_RH8_TEMP12 0x01
#define HTU21D_HEATER_ON 0x04
#define HTU21D_HEATER_OFF 0xFB
#define HTU21D_USER_REGISTER_WRITE 0xE6
#define HTU21D_USER_REGISTER_READ 0xE7
#define HTU21D_TRIGGER_TEMP_MEASURE_HOLD 0xE3
#define HTU21D_CRC8_POLYNOMINAL 0x13100
#define HTU21D_TEMP_COEFFICIENT -0.15
#define HTU21D_TRIGGER_HUMD_MEASURE_HOLD 0xE5

#define I2C_ERROR 0xFF

#define I2C_SDA 40
#define I2C_SCL 39

#define INPUT_PIN 16
#define LED_PIN 15

#define HOSTNAME "tempsensor"   // http://tempsensor.local

#define MQTT_BROKER "192.168.8.102"
#define MQTT_CLIENT_ID "tempsensor"
#define MQTT_USER ""
#define MQTT_PASS ""
#define MQTT_ROOT_TOPIC "tempsensor"

#define START_CHECKING_AFTER 15000 //15s

#define TIME_BETWEEN_MQTT_PUBLISH 30 //30s
#define SECONDS_BETWEEN_RETRIES 15
#define MAX_CONNECTION_RETRIES 5

#define BLINK_LED_TICK_PERIOD 1000

bool activeWIFI, enableMQTT, activeMQTT, lastButtonState, buttonState, lastLedState;
int64_t lastReadingsCommunicationTime, lastTimeMQTTPublished;
float temp, hum, lastTemp, lastHum;
int32_t ledTick;

OneButton button(INPUT_PIN, true, true);
WiFiManager wm;
WiFiClient espClient;
PubSubClient mqttClient(espClient);

void setup() {
  pinMode(LED_PIN, OUTPUT);
  Serial.begin(115200);
  initButton();
  initSensor();
  initWifi();
}

void loop() {
  processWifi();
  if (activeWIFI) {
    button.tick();
    mqttClientLoop();
    if ((millis() - lastReadingsCommunicationTime >= START_CHECKING_AFTER)) {
      lastReadingsCommunicationTime = millis();

      readSensor(temp, hum);
    }
    if (lastTemp != temp) {
      lastTemp = temp;
      publishTempHum(temp, hum);
    }    
    if (lastHum != hum) {
      lastHum = hum;
     publishTempHum(temp, hum);
    }
    if (lastButtonState != buttonState) {
      publishButton(buttonState);
      lastButtonState = buttonState;
      buttonState = false;
    }
  } 
  else {  
    blinkLed();
  }
}

void initSensor() {
  Wire.begin(I2C_SDA, I2C_SCL);
  
  uint8_t userRegisterData = 0;

  Wire.beginTransmission(HTU21D_I2C_ADDRESS);
  Wire.write(HTU21D_USER_REGISTER_READ);
  Wire.endTransmission(true);
  Wire.requestFrom(HTU21D_I2C_ADDRESS, 1);
  if (Wire.available() != 1) {
    return;
  }
  userRegisterData = Wire.read();

  userRegisterData &= 0x7E;
  userRegisterData |= HTU21D_RES_RH8_TEMP12;
  userRegisterData &= HTU21D_HEATER_OFF;
  
  Wire.beginTransmission(HTU21D_I2C_ADDRESS);
  Wire.write(HTU21D_USER_REGISTER_WRITE);
  Wire.write(userRegisterData);
  Wire.endTransmission(true);
}

float readTemperature() {
  int8_t qntRequest = 3;
  uint16_t rawTemperature = 0;
  uint8_t checksum = 0;

  Wire.beginTransmission(HTU21D_I2C_ADDRESS);
  Wire.write(HTU21D_TRIGGER_TEMP_MEASURE_HOLD);
  if (Wire.endTransmission(true) != 0) {
    return I2C_ERROR;
  }

  delay(22);

  Wire.requestFrom(HTU21D_I2C_ADDRESS, qntRequest);
  if (Wire.available() != qntRequest) {
    return I2C_ERROR;
  }
  rawTemperature = Wire.read() << 8;
  rawTemperature |= Wire.read();
  checksum = Wire.read();
  
  if (checkCRC8(rawTemperature) != checksum) {
    return I2C_ERROR;
  }
  return (0.002681 * (float)rawTemperature - 46.85);
}

float readCompensatedHumidity(float temperature) {
  float humidity = readHumidity();
  if (humidity == I2C_ERROR || temperature == I2C_ERROR) {
    return I2C_ERROR;
  }
  if (temperature > 0 && temperature < 80) {
    humidity = humidity + (25.0 - temperature) * HTU21D_TEMP_COEFFICIENT;
  }
  return humidity;
}

uint8_t checkCRC8(uint16_t data) {
  for (uint8_t bit = 0; bit < 16; bit++) {
    if (data & 0x8000) {
      data = (data << 1) ^ HTU21D_CRC8_POLYNOMINAL;
    } else {
      data <<= 1;
    }
  }
  return data >>= 8;
}

float readHumidity() {
  uint16_t rawHumidity = 0;
  uint8_t checksum = 0;
  float humidity = 0;

  Wire.beginTransmission(HTU21D_I2C_ADDRESS);
  Wire.write(HTU21D_TRIGGER_HUMD_MEASURE_HOLD);
  if (Wire.endTransmission(true) != 0) {
    return I2C_ERROR;
  }
  delay(4);
  Wire.requestFrom(HTU21D_I2C_ADDRESS, 3);
  if (Wire.available() != 3) {
    return I2C_ERROR;
  }

  rawHumidity = Wire.read() << 8;
  rawHumidity |= Wire.read();
  checksum = Wire.read();
 
  if (checkCRC8(rawHumidity) != checksum) {
    return I2C_ERROR;
  }

  rawHumidity ^= 0x02;
  humidity = (0.001907 * (float)rawHumidity - 6);
  
  if (humidity < 0) {
    humidity = 0;
  } else if (humidity > 100) {
    humidity = 100;
  }
  return humidity;
}

void readSensor(float &temp, float &hum) {
  temp = readTemperature();
  hum = readCompensatedHumidity(temp);
}

void initButton() {
  button.attachClick(clickButton);
}

void clickButton() {
  buttonState = true;
}

void WiFiEvent(WiFiEvent_t event) {
  switch (event) {
    case SYSTEM_EVENT_STA_GOT_IP:
      activeWIFI = true;
      enableMQTT = true;
      initMQTT();
      digitalWrite(LED_PIN, HIGH);
      break;
    case WL_DISCONNECTED:
      activeWIFI = false;
      enableMQTT = false;
      break;  
    default:
      break;
  }
}

void initWifi() {
  WiFi.mode(WIFI_STA);
  WiFi.onEvent(WiFiEvent); 
    
  //reset settings - wipe credentials for testing
  //wm.resetSettings();
  wm.setConfigPortalBlocking(false);

  if (wm.autoConnect("Temp sensor")){
    Serial.println("connected...");
  } else {
    Serial.println("Configportal running");
  }
}

void processWifi() {
  wm.process();
}

void initMQTT() {
  if (mqttClient.connected()) {
    mqttClient.disconnect();
  }
  Serial.printf("-->[MQTT] Initializing MQTT to broker IP: %s\n", MQTT_BROKER);
  mqttClient.setServer(MQTT_BROKER, 1883);
  mqttReconnect();
}

void mqttClientLoop() {
  if (enableMQTT) {
    mqttReconnect();
    if (mqttClient.connected()) {
      mqttClient.loop();
    }
  }
}

void publishTempHum(float temp, float hum) {
  if (activeWIFI && mqttClient.connected() && (millis() - lastTimeMQTTPublished >= TIME_BETWEEN_MQTT_PUBLISH * 1000)) {
    publishFloatMQTT("/temp", temp);
    publishFloatMQTT("/hum", hum);
    lastTimeMQTTPublished = millis();
  }
}

void publishFloatMQTT(String topic, float payload) {
  char charPublish[20];
  dtostrf(payload, 0, 2, charPublish);
  topic = MQTT_ROOT_TOPIC + topic;
  Serial.printf("-->[MQTT] Publishing %.0f to ", payload);
  Serial.println("topic: " + topic);
  mqttClient.publish((topic).c_str(), charPublish);
}

void publishIntMQTT(String topic, int16_t payload) {
  char charPublish[20];
  dtostrf(payload, 0, 0, charPublish);
  topic = MQTT_ROOT_TOPIC + topic;
  Serial.printf("-->[MQTT] Publishing %d to ", payload);
  Serial.println("topic: " + topic);
  mqttClient.publish((topic).c_str(), charPublish);
}

void publishButton(int16_t payload) {
  publishIntMQTT("/button", payload);
}

void mqttReconnect() {
  static uint64_t timeStamp = 0;
  static uint16_t connectionRetries = 0;
  if (millis() - timeStamp >= (SECONDS_BETWEEN_RETRIES * 1000)) { // Max one try each secondsBetweenRetries*1000 seconds
    timeStamp = millis();
    if (!mqttClient.connected()) {
      Serial.printf("-->[MQTT] Attempting MQTT connection... ");
      if (mqttClient.connect(MQTT_CLIENT_ID, MQTT_USER, MQTT_PASS)) {
        activeMQTT = true;
        Serial.printf("connected\n");
      } else {
        ++connectionRetries;
        mqttClient.setSocketTimeout(2);
        Serial.printf(" not possible to connect to %s ", MQTT_BROKER);
        Serial.printf("Connection status:  %d. (%d of %d retries)\n", mqttClient.state(), connectionRetries, MAX_CONNECTION_RETRIES); // Possible States: https://pubsubclient.knolleary.net/api#state
        if (connectionRetries >= MAX_CONNECTION_RETRIES) {
          enableMQTT = false;
          Serial.printf("-->[MQTT] Max retries to connect to MQTT broker reached, disabling MQTT...\n");
        }
      }
    }
  }
}

void blinkLed() {
  if (ledTick == BLINK_LED_TICK_PERIOD) {
    ledTick = 0;
    lastLedState = !lastLedState;
    digitalWrite(LED_PIN, lastLedState);
  }
  ledTick++;
}
