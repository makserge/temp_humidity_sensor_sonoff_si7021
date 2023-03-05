#include <FS.h>
#include <SPIFFS.h>
#include <ArduinoJson.h> // https://github.com/bblanchon/ArduinoJson
#include "OneButton.h"
#include "src/WiFiManager/WiFiManager.h" // https://github.com/tzapu/WiFiManager
#include "PubSubClient.h"

#define SENSOR_PIN 21
#define INPUT_PIN 16
#define LED_PIN  15
#define OUTPUT_PIN 17

#define HOSTNAME "tempsensor"   // http://tempsensor.local

#define CONFIG_FILE "/config.json"

#define MQTT_SOCKET_TIMEOUT 60
#define MQTT_BROKER "192.168.8.100"
#define MQTT_PORT "1883"
#define MQTT_CLIENT_ID "tempsensor"
#define MQTT_ROOT_TOPIC "tempsensor"

#define MQTT_BUTTON_TOPIC "button"
#define MQTT_TEMPERATURE_TOPIC "temperature"
#define MQTT_HUMIDITY_TOPIC "humidity"
#define MQTT_OUTPUT_TOPIC "output"

#define START_CHECKING_AFTER 15000 //15s

#define TIME_BETWEEN_MQTT_PUBLISH 30 //30s
#define SECONDS_BETWEEN_RETRIES 15
#define MAX_CONNECTION_RETRIES 5

#define BLINK_LED_TICK_PERIOD_NO_WIFI 1000
#define BLINK_LED_TICK_PERIOD_WIFI 5
#define BLINK_LED_TICK_PERIOD_NO_SENSOR 10000
#define BLINK_LED_TICK_PERIOD_NO_MQTT 50000

bool activeWIFI, enableMQTT, activeMQTT, lastButtonState, buttonState, lastLedState;
int64_t lastReadingsCommunicationTime, lastTimeMQTTPublished;
float temp, hum, lastTemp, lastHum;
int32_t ledTick, ledTickPeriod;

OneButton button(INPUT_PIN, true, true);
WiFiManager wm;
WiFiClient espClient;
PubSubClient mqttClient(espClient);

char mqttBroker[40] = MQTT_BROKER;
char mqttPort[6] = MQTT_PORT;
char mqttUser[20];
char mqttPass[20];
char mqttRootTopic[30] = MQTT_ROOT_TOPIC;

WiFiManagerParameter customMqttBroker, customMqttPort, customMqttUser, customMqttPass, customMqttRootTopic;
 
void setup() {
  pinMode(LED_PIN, OUTPUT);
  pinMode(OUTPUT_PIN, OUTPUT);
  
  Serial.begin(115200);
  loadCredentials();
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
      readSensor(SENSOR_PIN);
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
  blinkLed();
}

void saveConfigCallback () {
  if (SPIFFS.begin()) {
    File configFile = SPIFFS.open(CONFIG_FILE, "w");
    if (!configFile) {
      Serial.println("failed to open config file for writing");
      return;
    }

    DynamicJsonDocument json(1024);

    json["mqtt_broker"] = customMqttBroker.getValue();
    json["mqtt_port"] = customMqttPort.getValue();
    json["mqtt_user"] = customMqttUser.getValue();
    json["mqtt_pass"] = customMqttPass.getValue();
    json["mqtt_topic"] = customMqttRootTopic.getValue();

    serializeJson(json, configFile);
    configFile.close();

    SPIFFS.end();
  } else {
    Serial.println("failed to mount FS");
  }
 }
 
void loadCredentials() {
  if (SPIFFS.begin()) {
    if (SPIFFS.exists(CONFIG_FILE)) {
      File configFile = SPIFFS.open(CONFIG_FILE, "r");
      if (configFile) {
        size_t size = configFile.size();
        std::unique_ptr<char[]> buf(new char[size]);

        configFile.readBytes(buf.get(), size);

        DynamicJsonDocument json(1024);
        auto deserializeError = deserializeJson(json, buf.get());
        if (!deserializeError) {
          strcpy(mqttBroker, json["mqtt_broker"]);
          strcpy(mqttPort, json["mqtt_port"]);
          strcpy(mqttUser, json["mqtt_user"]);
          strcpy(mqttPass, json["mqtt_pass"]);
          strcpy(mqttRootTopic, json["mqtt_topic"]);
        } else {
          Serial.println("failed to load json config");
        }
        configFile.close();
        SPIFFS.end();
      }
    }
  } else {
    Serial.println("failed to mount FS");
  }
}

void initSensor() {
  pinMode(SENSOR_PIN, INPUT);
  delay(2000);
  readSensor(SENSOR_PIN);
}

bool waitState(int pin, bool state) {
  uint64_t timeout = micros();
  while (micros() - timeout < 100) {
    if (digitalRead(pin) == state) return true;
    delayMicroseconds(1);
  }
  return false;
}

void readSensor(int pin) {
  uint8_t data[5] = {0};

  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
  delayMicroseconds(500);
  digitalWrite(pin, HIGH);
  delayMicroseconds(20);
  pinMode(pin, INPUT);

  uint32_t i = 0;
  if (waitState(pin, 0) && waitState(pin, 1) && waitState(pin, 0)) {
    for (i = 0; i < 40; i++) {
      if (!waitState(pin, 1)) {
        break;
      }
      delayMicroseconds(35);
      if (digitalRead(pin) == HIGH) {
        data[i / 8] |= (1 << (7 - i % 8));
      }
      if (!waitState(pin, 0)) {
        break;
      }
    }
  }

  uint8_t checksum = (data[0] + data[1] + data[2] + data[3]) & 0xFF;
  if ((i == 40)&& (data[4] == checksum)) {
    double tempInternal = (((data[2] & 0x7F) << 8) | data[3]) * 0.1;
    if (data[2] & 0x80) {
      tempInternal *= -1;
    }
    temp = tempInternal;  
    hum = ((data[0] << 8) | data[1]) * 0.1;
  }
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
      ledTickPeriod = BLINK_LED_TICK_PERIOD_WIFI;
      activeWIFI = true;
      enableMQTT = true;
      initMQTT();
      digitalWrite(LED_PIN, HIGH);
      break;
    case WL_DISCONNECTED:
      ledTickPeriod = BLINK_LED_TICK_PERIOD_NO_WIFI;
      activeWIFI = false;
      enableMQTT = false;
      break;  
    default:
      break;
  }
}

void initWifi() {
  ledTickPeriod = BLINK_LED_TICK_PERIOD_NO_WIFI;
  WiFi.mode(WIFI_STA);
  WiFi.onEvent(WiFiEvent); 
    
  //reset settings - wipe credentials for testing
  //wm.resetSettings();
  wm.setConfigPortalBlocking(false);

  new (&customMqttBroker) WiFiManagerParameter("mqtt_broker", "MQTT broker", mqttBroker, 40);
  new (&customMqttPort) WiFiManagerParameter("mqtt_port", "MQTT port", mqttPort, 6);
  new (&customMqttUser) WiFiManagerParameter("mqtt_user", "MQTT user", mqttUser, 20);
  new (&customMqttPass) WiFiManagerParameter("mqtt_pass", "MQTT pass", mqttPass, 20);
  new (&customMqttRootTopic) WiFiManagerParameter("mqtt_topic", "MQTT root topic", mqttRootTopic, 30);

  wm.addParameter(&customMqttBroker);
  wm.addParameter(&customMqttPort);
  wm.addParameter(&customMqttUser);
  wm.addParameter(&customMqttPass);
  wm.addParameter(&customMqttRootTopic);

  wm.setSaveConfigCallback(saveConfigCallback);

  if (wm.autoConnect("Temp sensor")) {
    Serial.println("connected...");
  } else {
    Serial.println("Configportal running");
  }
}

void processWifi() {
  wm.process();
}

void mqttClientCallback(char* topic, byte* payload, unsigned int length) {
  digitalWrite(OUTPUT_PIN, ((char)payload[0] == '1'));
}

void initMQTT() {
  if (mqttClient.connected()) {
    mqttClient.disconnect();
  }
  Serial.printf("-->[MQTT] Initializing MQTT to broker IP: %s\n", mqttBroker);
  mqttClient.setSocketTimeout(MQTT_SOCKET_TIMEOUT);
  mqttClient.setServer(mqttBroker, atol(mqttPort));
  mqttClient.setCallback(mqttClientCallback);
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
    publishFloatMQTT(MQTT_TEMPERATURE_TOPIC, temp);
    publishFloatMQTT(MQTT_HUMIDITY_TOPIC, hum);
    lastTimeMQTTPublished = millis();
  }
}

void publishFloatMQTT(String topic, float payload) {
  char charPublish[20];
  dtostrf(payload, 0, 2, charPublish);
  topic = String(mqttRootTopic) + "/" + topic;
  Serial.printf("-->[MQTT] Publishing %.0f to ", payload);
  Serial.println("topic: " + topic);
  mqttClient.publish((topic).c_str(), charPublish);
}

void publishIntMQTT(String topic, int16_t payload) {
  char charPublish[20];
  dtostrf(payload, 0, 0, charPublish);
  topic = String(mqttRootTopic) + "/" + topic;
  Serial.printf("-->[MQTT] Publishing %d to ", payload);
  Serial.println("topic: " + topic);
  mqttClient.publish((topic).c_str(), charPublish);
}

void publishButton(int16_t payload) {
  publishIntMQTT(MQTT_BUTTON_TOPIC, payload);
}

void mqttReconnect() {
  static uint64_t timeStamp = 0;
  static uint16_t connectionRetries = 0;
  if (millis() - timeStamp >= (SECONDS_BETWEEN_RETRIES * 1000)) { // Max one try each secondsBetweenRetries*1000 seconds
    timeStamp = millis();
    if (!mqttClient.connected()) {
      Serial.printf("-->[MQTT] Attempting MQTT connection... ");
      if (mqttClient.connect(MQTT_CLIENT_ID, mqttUser, mqttPass)) {
        activeMQTT = true;
        
        String subscribeTopic = String(mqttRootTopic) + "/" + MQTT_OUTPUT_TOPIC;
        mqttClient.subscribe(subscribeTopic.c_str());

        publishTempHum(temp, hum);
        
        Serial.println("connected");
      } else {
        ++connectionRetries;
        mqttClient.setSocketTimeout(2);
        Serial.printf(" not possible to connect to %s ", mqttBroker);
        Serial.printf("Connection status:  %d. (%d of %d retries)\n", mqttClient.state(), connectionRetries, MAX_CONNECTION_RETRIES); // Possible States: https://pubsubclient.knolleary.net/api#state
        if (connectionRetries >= MAX_CONNECTION_RETRIES) {
          ledTickPeriod = BLINK_LED_TICK_PERIOD_NO_MQTT;
          Serial.println("-->[MQTT] Max retries to connect to MQTT broker reached, disabling MQTT...");
          delay(3000);
          connectionRetries = 0;
        }
      }
    }
  }
}

void blinkLed() {
  if (ledTick == ledTickPeriod) {
    ledTick = 0;
    lastLedState = !lastLedState;
    digitalWrite(LED_PIN, lastLedState);
  }
  ledTick++;
}
