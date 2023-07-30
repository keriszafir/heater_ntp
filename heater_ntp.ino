/* Simple heater controller for G11, G12, G12r tariffs
Developed for Olimex ESP8266 eval board
2023 Keri Szafir, Keritech Electronics
*/

#include <NTPClient.h>
#include <time.h>
#include <PubSubClient.h>
#include <WiFiManager.h>
#include <WiFiUdp.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// #define RESET_SETTINGS
// #define DEBUG
#define relay_gpio 5
#define button_gpio 13
#define W1_gpio 12

// initial WLAN SSID and parameters in the config
const char *ap_ssid = "HeaterController";
char *ntp_server = "192.168.1.1";
char *mqtt_server = "192.168.1.1";
char *mqtt_topic_root = "heater";
char *tariff = "G12r";
WiFiManagerParameter par_ntp_server("ntp_server", "NTP server", ntp_server, 40);
WiFiManagerParameter par_mqtt_server("mqtt_server", "MQTT Server", mqtt_server, 40);
WiFiManagerParameter par_mqtt_topic_root("topic", "MQTT Topic Root", mqtt_topic_root, 40);
WiFiManagerParameter par_tariff("tariff", "Utility Tariff", tariff, 5);

OneWire oneWire(W1_gpio);
DallasTemperature sensors(&oneWire);
WiFiClient espClient;
WiFiManager wm;
PubSubClient mqttClient(espClient);
WiFiUDP ntpUDP;
NTPClient ntpClient(ntpUDP, ntp_server, 0, 60000);

const char* timezone = "CET-1CEST,M3.5.0,M10.5.0/3";
int manual = 0;
int timer = 0;
int relay_state = 0;
int mqtt_rc_attempts_left = 10;
struct tm * lt;
float temp;

void handle_button() {
  if (!digitalRead(button_gpio)) {
    // start and end time
    unsigned long st, et;
    st = millis();

    while(!digitalRead(button_gpio)) {
      // wait for button release
      wait(50);
    }
    et = millis();

    // determine button press length and action
    if (et - st > 30000UL) {
      // 30s = phactory reset
      Serial.println("Settings reset initiated.");
      wm.resetSettings();
      wait(4000UL);
      ESP.restart();
    } else if (et - st > 5000UL) {
      // 5s = restart the heater
      Serial.println("Restarting.");
      wait(1000UL);
      ESP.restart();
    } else if (et - st > 500UL) {
      // 500ms debounce = manual override on-off action
      manual = !manual;
      Serial.print("Device is in ");
      Serial.print(manual ? "manual" : "automatic");
      Serial.println(" control mode.");
      // inform mqtt on change
      mqtt_report();
    }
  }
}

void check_temp() {
  sensors.requestTemperatures();
  temp = sensors.getTempCByIndex(0);
}

void check_time() {
  time_t epochtime = ntpClient.getEpochTime();
  lt = localtime(&epochtime);
}

void check_tariff() {
  int timer_enable;
  int hour = lt->tm_hour;
  if (!strcmp(tariff, "G12r")) {
    // 13-16, 22-7 (heating on at 5-7)
    if (hour >= 16) {
      timer_enable = 0;
    } else if (hour >= 13) {
      timer_enable = 1;
    } else if (hour >= 7) {
      timer_enable = 0;
    } else if (hour >= 5) {
      timer_enable = 1;
    } else {
      timer_enable = 0;
    }
  } else if (!strcmp(tariff, "G12")) {
    // 13-15, 22-6 (heating on at 4-6)
    if (hour >= 15) {
      timer_enable = 0;
    } else if (hour >= 13) {
      timer_enable = 1;
    } else if (hour >= 6) {
      timer_enable = 0;
    } else if (hour >= 4) {
      timer_enable = 1;
    } else {
      timer_enable = 0;
    }
  } else {
    // G11 or other tariff - set arbitrarily
    if (hour >= 17) {
      timer_enable = 0;
    } else if (hour >= 15) {
      timer_enable = 1;
    } else if (hour >= 5) {
      timer_enable = 0;
    } else if (hour >= 3) {
      timer_enable = 1;
    } else {
      timer_enable = 0;
    }
  }
  if (timer_enable != timer) {
  // update state and inform on mqtt
  timer = timer_enable;
  mqtt_report();
  }
}

void handle_relay() {
  int prev_relay_state = relay_state;
  if (timer || manual) {
    digitalWrite(relay_gpio, HIGH);
    relay_state = 1;
  } else {
    digitalWrite(relay_gpio, LOW);
    relay_state = 0;
  }
  // quick exit if relay state unchanged - avoid unnecessary printing to serial
  if (relay_state == prev_relay_state) return;

  // inform on status change
  mqtt_report();
  // report the state if changed
  if (relay_state) Serial.println("Heater ON"); else Serial.println("Heater OFF");
}

void mqtt_callback(char* topic, byte* payload, unsigned int length) {
  String msg;
  Serial.print("Message arrived @");
  Serial.print(topic);
  Serial.print(": ");
  for (int i = 0; i < length; i++) msg += (char)payload[i];
  
  // message interpretation logic
  String subtopic = String(topic);
  subtopic.replace(mqtt_topic_root, "");
  if (!strcmp(subtopic.c_str(), "/manual")) manual = msg.toInt();
  // poll the parameters
  if (!strcmp(msg.c_str(), "?")) mqtt_report();
}

void mqtt_reconnect() {
  // MQTT reconnection routine; do nothing if already connected
  if (mqttClient.connected()) return;

  // Loop until we're reconnected or 10 attempts failed
  // clear this counter on restart
  while ((!mqttClient.connected()) && (mqtt_rc_attempts_left)) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "Heater-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (mqttClient.connect(clientId.c_str())) {
      // reset the failed reconnect counter
      mqtt_rc_attempts_left = 10;
      Serial.print(" MQTT connected... ");
      // ... and resubscribe
      char subtopics[20];
      sprintf(subtopics, "%s/#", mqtt_topic_root);
      mqttClient.subscribe(mqtt_topic_root);
      mqttClient.subscribe(subtopics);
      Serial.print("subscribing ");
      Serial.println(subtopics);
    } else {
      mqtt_rc_attempts_left -= 1;
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.print(", attempts left: ");
      Serial.print(mqtt_rc_attempts_left);
      if (mqtt_rc_attempts_left) {
        Serial.println(" - try again in 5 seconds");
        // Wait 5 seconds before retrying
        wait(5000UL);
      } else Serial.println(" - MQTT disabled. Restart the device to re-enable.");
    }
  }
}

void mqtt_setup() {
  // MQTT initialization
  Serial.print("Connecting to MQTT server: ");
  Serial.println(mqtt_server);
  if (strcmp(mqtt_server, "")) {
    // MQTT initialization if not left empty
    mqttClient.setServer(mqtt_server, 1883);
    mqttClient.setCallback(mqtt_callback);
    mqtt_reconnect();

    char msg[150];
    sprintf(msg, "Keritech Electronics heater controller at %s", WiFi.localIP().toString().c_str());
    mqttClient.publish(mqtt_topic_root, msg);
    Serial.println(msg);
    mqtt_report();
  } else Serial.println("MQTT server not specified. Skipping MQTT setup.");
}

void mqtt_report() {
  // quick exit if no mqtt
  if (!mqttClient.connected()) return;

  char msg[200], subtopic[50];
  sprintf(msg, "{time: %s, tariff: %s, timer control: %d, manual control: %d, working: %d, temp: %f}", asctime(lt), tariff, timer, manual, relay_state, temp);
  sprintf(subtopic, "{%s/status}", mqtt_topic_root);
  mqttClient.publish(subtopic, msg);
}

void debug_info() {
  #ifdef DEBUG
  Serial.print(asctime(lt));
  Serial.print("Tariff: ");
  Serial.println(tariff);
  Serial.print("Timer control: ");
  Serial.print(timer);
  Serial.print(", manual control: ");
  Serial.print(manual);
  Serial.print(", relay: ");
  Serial.println(relay_state);
  Serial.print("Current temperature: ");
  Serial.print(temp);
  Serial.println("°C");
  wait(500UL);
  #endif
}

void delayed_restart() {
  Serial.println("Restarting after config save...");
  wait(2000UL);
  ESP.restart();
}

void wait(unsigned long t) {
  // wait for specified time and do usual infrastructure-related stuff
  int ts = millis();
  while (millis() - ts < t) {
    wm.process();
    ntpClient.update();
    mqtt_reconnect();
    mqttClient.loop();
  }
}

void wifi_setup() {
  #ifdef RESET_SETTINGS
  wm.resetSettings();
  #endif
  #ifdef DEBUG
  wm.setDebugOutput(true);
  #endif
  Serial.println("Initializing WLAN...");
  WiFi.mode(WIFI_STA);
  wm.setClass("invert");
  wm.setConnectTimeout(20);
  wm.setConfigPortalBlocking(false);
  wm.setSaveParamsCallback(delayed_restart);
  wm.addParameter(&par_ntp_server);
  wm.addParameter(&par_mqtt_server);
  wm.addParameter(&par_mqtt_topic_root);
  wm.addParameter(&par_tariff);
  strcpy(ntp_server, par_ntp_server.getValue());
  strcpy(mqtt_server, par_mqtt_server.getValue());
  strcpy(mqtt_topic_root, par_mqtt_topic_root.getValue());
  strcpy(tariff, par_tariff.getValue());
  wm.autoConnect(ap_ssid);
  Serial.print("WiFi IP: ");
  Serial.println(WiFi.localIP());
}

void ntp_setup() {
  // set local timezone to pre-configured and initialize NTP
  setenv("TZ", timezone, 1);
  Serial.print("Connecting to NTP server: ");
  Serial.println(ntp_server);
  ntpClient.begin();
  ntpClient.update();
  check_time();
  Serial.print("Time updated. Current local time is: ");
  Serial.println(asctime(lt));
}

void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.println("Water heater controller - Keritech Electronics - 2023");
  pinMode(relay_gpio, OUTPUT);
  pinMode(button_gpio, INPUT_PULLUP);
  sensors.begin();
  check_temp();
  wifi_setup();
  ntp_setup();
  mqtt_setup();
}

void loop() {
  wm.process();
  ntpClient.update();
  mqtt_reconnect();
  mqttClient.loop();
  check_time();
  check_tariff();
  check_temp();
  handle_button();
  handle_relay();
  debug_info();
}
