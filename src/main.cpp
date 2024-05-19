#include <Arduino.h>
#include <web.h>
#include <shared.h>
#include <sensor.h>
#include <heater.h>
#include <PID_v1.h>
#include <config.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

#include "env.h"

#define PID_INTERVAL 200

double currentTempBoiler = 0;

double gTargetTemp=S_TSET;
double gOvershoot=S_TBAND;
double gOutputPwr=0.0;
double gP = S_P, gI = S_I, gD = S_D;
double gaP = S_aP, gaI = S_aI, gaD = S_aD;

boolean overShootMode = false;

unsigned long time_now=0;
unsigned long time_last=0;

PID ESPPID(&currentTempBoiler, &gOutputPwr, &gTargetTemp, gP, gI, gD, DIRECT);


WiFiClient espClient;
PubSubClient client(espClient);
unsigned long lastMsg = 0;

const char* mqtt_server = MQTT_IP;

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");

  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  char buffer[128];
  strncpy(buffer, (char*) payload, length);
  buffer[length] = '\0';

  int temp = atoi(buffer);
  if (gTargetTemp != temp)
  {
    gTargetTemp = temp;
    saveConfig();
  }
}

void setup() {
  Serial.begin(115200);

  Serial.println("Mounting SPIFFS...");
  if(!prepareFS()) {
    Serial.println("Failed to mount SPIFFS !");
  } else {
    Serial.println("Mounted.");
  }
  Serial.println("Loading config...");
  if (!loadConfig()) {
    Serial.println("Failed to load config. Using default values and creating config...");
    if (!saveConfig()) {
     Serial.println("Failed to save config");
    } else {
      Serial.println("Config saved");
    }
  } else {
    Serial.println("Config loaded");
  }


  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  int i = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(++i); Serial.print(' ');
  }

  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

  setupWeb();
  Serial.print("Webserver listening...");

  setupSensor();
  setupHeater();

  // start PID
  ESPPID.SetTunings(gP, gI, gD);
  ESPPID.SetSampleTime(PID_INTERVAL);
  ESPPID.SetOutputLimits(0, 1000);
  ESPPID.SetMode(AUTOMATIC);

  time_now=millis();
  time_last=time_now;
}


const char* boiler_state_topic = "silvia/boiler/state";
const char* boiler_temperature_topic = "silvia/boiler/temperature/state";
const char* brewhead_temperature_topic = "silvia/brewhead/temperature/state";

const char* boiler_target_temperature_set_topic = "silvia/boiler/target/set";
const char* boiler_target_temperature_state_topic = "silvia/boiler/target/state";

const char* availablility_topic = "silvia/boiler/available";

void sendTemperatureDiscovery()
{
  char buffer[1024];
  DynamicJsonDocument doc(1024);

  JsonObject device  = doc.createNestedObject("device");
  device["identifiers"][0] = "Rancilio_Silvia_MW";
  device["model"] = "Rancilio_Silvia_MW";
  device["name"] = "Rancilio Silvia";
  device["manufacturer"] = "opidopiopi"; 
  device["sw_version"] = "1.0";  

  doc["name"] = "Boiler State";
  doc["unique_id"] = "boiler_state"; 
  doc["device_class"] = "heat";
  doc["state_topic"] = boiler_state_topic;
  doc["availability_topic"] = availablility_topic;
  doc["val_tpl"] = "{{ value_json.state }}";
  serializeJson(doc, buffer);
  client.publish("homeassistant/binary_sensor/silvia_boiler_state/config", buffer);

  doc["name"] = "Brewhead Temperature";
  doc["unique_id"] = "brewhead_temp"; 
  doc["device_class"] = "temperature";
  doc["state_class"] = "measurement";
  doc["unit_of_measurement"] = "C";  
  doc["state_topic"] = brewhead_temperature_topic;
  doc["val_tpl"] = "{{ value_json.temperature }}";
  serializeJson(doc, buffer);
  client.publish("homeassistant/sensor/silvia_brewhead_temp/config", buffer);

  doc["name"] = "Boiler Temperature";
  doc["unique_id"] = "boiler_temp"; 
  doc["device_class"] = "temperature";
  doc["state_class"] = "measurement";
  doc["unit_of_measurement"] = "C";  
  doc["state_topic"] = boiler_temperature_topic;
  doc["val_tpl"] = "{{ value_json.temperature }}";
  serializeJson(doc, buffer);
  client.publish("homeassistant/sensor/silvia_boiler_temp/config", buffer);

  doc["name"] = "Boiler Target Temperature";
  doc["unique_id"] = "boiler_target_temp"; 
  doc["device_class"] = "temperature";
  doc["command_topic"] = boiler_target_temperature_set_topic;
  doc["state_topic"] = boiler_target_temperature_state_topic;
  doc["max"] = 120;
  doc["min"] = 0;
  doc["step"] = 0.5;
  doc["mode"] = "box";
  serializeJson(doc, buffer);
  client.publish("homeassistant/number/silvia_boiler_target_temp/config", buffer);
}


void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "nodemcu-silvia";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str(), "", "", availablility_topic, 2, true, "offline")) {
      client.setBufferSize(1024);
      Serial.println("connected");

      client.publish(availablility_topic, "online", true);

      sendTemperatureDiscovery();

      client.subscribe(boiler_target_temperature_set_topic);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

unsigned long last_update = 0;
double lastTargetTemp = gTargetTemp + 1;

void loop() {
  time_now=millis();
  currentTempBoiler = getBoilerTemperature();

  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  if (time_now - last_update >= 500)
  {
    Serial.print("Temp: ");
    Serial.println(currentTempBoiler);
    DynamicJsonDocument doc(512);
    doc["temperature"] = currentTempBoiler;

    char buffer[512];
    serializeJson(doc, buffer);
    client.publish(boiler_temperature_topic, buffer);

    doc.clear();
    doc["temperature"] = getBrewHeadTemperature();

    serializeJson(doc, buffer);
    client.publish(brewhead_temperature_topic, buffer);

    doc.clear();
    doc["state"] = heaterState ? "ON" : "OFF";

    serializeJson(doc, buffer);
    client.publish(boiler_state_topic, buffer);

    if (abs(lastTargetTemp - gTargetTemp) > 0.1)
    {
      doc.clear();
      doc["temperature"] = gTargetTemp;
      lastTargetTemp = gTargetTemp;

      serializeJson(doc, buffer);
      client.publish(boiler_target_temperature_state_topic, buffer);
    }

    last_update = time_now;
  }

  if(time_now-time_last >= PID_INTERVAL or time_last > time_now) {
    if( !overShootMode && abs(gTargetTemp - currentTempBoiler)>=gOvershoot ) {        
      ESPPID.SetTunings(gaP, gaI, gaD);
      overShootMode=true;
    }
    else if( overShootMode && abs(gTargetTemp-currentTempBoiler)<gOvershoot ) {
      ESPPID.SetTunings(gP,gI,gD);
      overShootMode=false;
    }
    if(ESPPID.Compute()==true) {   
      setHeatPowerPercentage(gOutputPwr);
    }
    time_last=time_now;
  }
  delay(200);
  updateHeater();
}