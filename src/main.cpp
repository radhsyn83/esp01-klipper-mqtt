#include <Arduino.h>

#include <ESP8266WiFi.h>
#include <Ticker.h>
#include <AsyncMqttClient.h>
#include <Wire.h>
#include <SPI.h>

#define WIFI_SSID "BrumBrum"
#define WIFI_PASSWORD "sense324"

#define MQTT_HOST "192.168.1.137"
#define MQTT_PORT 1883
#define MQTT_USERNAME "syn"
#define MQTT_PASSWORD "sense324"

#define MQTT_PUB_POWER "klipper/power"
#define MQTT_PUB_LIGHT "klipper/light"
#define MQTT_PUB_STATUS "klipper/status"

const String TURN_ON = "on";
const String TURN_OFF = "off";

const int PIN_LIGHT = 2;
const int PIN_POWER = 0;

AsyncMqttClient mqttClient;
Ticker mqttReconnectTimer;

WiFiEventHandler wifiConnectHandler;
WiFiEventHandler wifiDisconnectHandler;
Ticker wifiReconnectTimer;

void connectToWifi() {
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

void onWifiDisconnect(const WiFiEventStationModeDisconnected& event) {
  Serial.println("Disconnected from Wi-Fi.");
  mqttReconnectTimer.detach(); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
  wifiReconnectTimer.once(2, connectToWifi);
}

void connectToMqtt() {
  Serial.println("Connecting to MQTT...");
  mqttClient.connect();
}

void onWifiConnect(const WiFiEventStationModeGotIP& event) {
  Serial.println("Connected to Wi-Fi.");
  connectToMqtt();
}

void printSeparationLine()
{
  Serial.println("************************************************");
}


void onMqttConnect(bool sessionPresent) {
  Serial.print("Connected to MQTT broker: ");
  Serial.print(MQTT_HOST);
  Serial.print(", port: ");
  Serial.println(MQTT_PORT);

  printSeparationLine();
  Serial.print("Session present: ");
  Serial.println(sessionPresent);

  mqttClient.subscribe(MQTT_PUB_LIGHT, 0);
  mqttClient.subscribe(MQTT_PUB_POWER, 0);

  mqttClient.publish(MQTT_PUB_STATUS, 1, true, "online");

  printSeparationLine();
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  (void)reason;

  Serial.println("Disconnected from MQTT.");

  if (WiFi.isConnected()) {
    mqttReconnectTimer.once(2, connectToMqtt);
  }
}

void onMqttSubscribe(const uint16_t &packetId, const uint8_t &qos)
{
  Serial.println("Subscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
  Serial.print("  qos: ");
  Serial.println(qos);
}

void onMqttUnsubscribe(const uint16_t &packetId)
{
  Serial.println("Unsubscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

void onMqttPublish(uint16_t packetId) {
  Serial.print("Publish acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

void relayOn(int pin, bool active)
{
  if (active)
  {
    digitalWrite(pin, LOW);
  }
  else
  {
    digitalWrite(pin, HIGH);
  }
}

void doMagic(char *topic, String action)
{
  bool state = true;

  Serial.print("--Topic : ");
  Serial.println(topic);
  Serial.print("--Payload : ");
  Serial.println(action);

  // change state
  if (action == TURN_ON)
  {
    state = false;
  }

  if (strcmp(topic, MQTT_PUB_POWER) == 0)
  {
    relayOn(PIN_POWER, state);
  }

  if (strcmp(topic, MQTT_PUB_LIGHT) == 0)
  {
    relayOn(PIN_LIGHT, state);
  }

  printSeparationLine();
}

void onMqttMessage(char *topic, char *payload, const AsyncMqttClientMessageProperties &properties,
                   const size_t &len, const size_t &index, const size_t &total)
{
  String tempPayload = "";
  for (int i = 0;(unsigned) i < len; i++)
  {
    tempPayload += (char)payload[i];
  }
  doMagic(topic, tempPayload);
}

void setup() {
  Serial.begin(115200);
  Serial.println();

  wifiConnectHandler = WiFi.onStationModeGotIP(onWifiConnect);
  wifiDisconnectHandler = WiFi.onStationModeDisconnected(onWifiDisconnect);
  
  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onSubscribe(onMqttSubscribe);
  mqttClient.onUnsubscribe(onMqttUnsubscribe);
  mqttClient.onMessage(onMqttMessage);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  mqttClient.setCredentials(MQTT_USERNAME, MQTT_PASSWORD);

  connectToWifi();
  
  pinMode(PIN_POWER, OUTPUT);
  pinMode(PIN_LIGHT, OUTPUT);

  //reset pin
  // digitalWrite(PIN_POWER, HIGH);
  // digitalWrite(PIN_LIGHT, HIGH);
}

void loop() {}