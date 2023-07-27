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

// #define DEBUG

// initial WLAN SSID and parameters in the config
const char *ap_ssid = "HeaterController";
WiFiManagerParameter ntp_server("ntp_server", "NTP server", "192.168.1.1", 40);
WiFiManagerParameter tariff("tariff", "Utility Tariff", "G12r", 5);
WiFiManagerParameter mqtt_server("mqtt_server", "MQTT Server", "", 40);
WiFiManagerParameter mqtt_topic_root("topic", "MQTT Topic Root", "heater", 40);

WiFiClient espClient;
WiFiManager wm;
PubSubClient mqttClient(espClient);
WiFiUDP ntpUDP;
NTPClient ntpClient(ntpUDP, ntp_server.getValue(), 0, 60000);

#define relay_gpio 5
#define button_gpio 4

const char* timezone = "CET-1CEST,M3.5.0,M10.5.0/3";
int mqtt_enable = 0;
int manual = 0;
int timer = 0;
int relay_state = 0;
int mqtt_failed_rc_attempts = 0;
int ct = 0;
int debounce_time = 200;
struct tm * lt;

void handle_button() {
  if (!digitalRead(button_gpio)) {
   ct = millis();
   while(millis() - ct < debounce_time) {
     // just wait
    }
  }
  // check again to see if it was not a Fluke
  // toggle if button still pressed
  if (!digitalRead(button_gpio)) {
    manual = !manual;
    Serial.print("Device is in ");
    Serial.print(manual ? "manual" : "automatic");
    Serial.println(" control mode.");
    mqtt_report();
  }
  while(!digitalRead(button_gpio)) {
    // wait for button release
    if (millis() - ct > 30000) {
      // reset settings after 30s
      reset_wm();
    }
  }
}

void check_time() {
  time_t epochtime = ntpClient.getEpochTime();
  lt = localtime(&epochtime);
}

void check_tariff() {
  int timer_enable;
  int hour = lt->tm_hour;
  if (!strcmp(tariff.getValue(), "G12r")) {
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
  } else if (!strcmp(tariff.getValue(), "G12")) {
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
  if (relay_state == prev_relay_state) {
    return;
  }
  // inform on status change
  mqtt_report();
  // report the state if changed
  if (relay_state) {
    Serial.println("Heater ON");
  } else {
    Serial.println("Heater OFF");
  }
}

void mqtt_callback(char* topic, byte* payload, unsigned int length) {
  String msg;
  Serial.print("Message arrived @");
  Serial.print(topic);
  Serial.print(": ");
  for (int i = 0; i < length; i++) {
    msg += (char)payload[i];
  }
  
  // message interpretation logic
  String subtopic = String(topic);
  subtopic.replace(mqtt_topic_root.getValue(), "");
  if (!strcmp(subtopic.c_str(), "/manual")) {
    manual = msg.toInt();
  }
}

void mqtt_reconnect() {
  // MQTT reconnection routine
  // Loop until we're reconnected or 10 attempts failed
  while ((!mqttClient.connected()) && (mqtt_failed_rc_attempts <= 10)) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "Heater-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (mqttClient.connect(clientId.c_str())) {
      // reset the failed reconnect counter
      mqtt_failed_rc_attempts = 0;
      Serial.print(" MQTT connected... ");
      // ... and resubscribe
      char subtopics[20];
      sprintf(subtopics, "%s/#", mqtt_topic_root.getValue());
      mqttClient.subscribe(mqtt_topic_root.getValue());
      mqttClient.subscribe(subtopics);
      Serial.print("subscribing ");
      Serial.println(subtopics);
    } else {
      mqtt_failed_rc_attempts += 1;
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.print(", attempts left: ");
      if (mqtt_failed_rc_attempts < 10) {
        Serial.print(10 - mqtt_failed_rc_attempts);
        Serial.println(" - try again in 5 seconds");
        // Wait 5 seconds before retrying
        delay(5000);
      } else {
        Serial.println("0 - MQTT functionality disabled.");
      }
    }
  }
}

void mqtt_setup() {
  // MQTT initialization
  const char* mqtt_address = mqtt_server.getValue();
  if (strcmp(mqtt_address, "")) {
    mqtt_enable = 1;
    // MQTT initialization if not left empty
    Serial.print("Connecting to MQTT server: ");
    Serial.println(mqtt_address);
    mqttClient.setServer(mqtt_server.getValue(), 1883);
    mqttClient.setCallback(mqtt_callback);
    mqtt_reconnect();

    char msg[150];
    sprintf(msg, "Keritech Electronics heater controller at %s", WiFi.localIP().toString().c_str());
    mqttClient.publish(mqtt_topic_root.getValue(), msg);
    Serial.println(msg);
    mqtt_report();
  }
}

void mqtt_report() {
  if (mqtt_enable) {
    char msg[200];
    sprintf(msg, "{tariff: %s, timer control: %d, manual control: %d, working: %d}", tariff.getValue(), timer, manual, relay_state);
    mqttClient.publish(mqtt_topic_root.getValue(), msg);
  }
}

void debug_info() {
  #ifdef DEBUG
  Serial.print(asctime(lt));
  Serial.print("Tariff: ");
  Serial.println(tariff.getValue());
  Serial.print("Timer: ");
  Serial.print(timer);
  Serial.print(", manual control: ");
  Serial.print(manual);
  Serial.print(", relay: ");
  Serial.println(relay_state);
  delay(500);
  #endif
}

void reset_wm() {
  Serial.println("Settings reset initiated.");
  wm.resetSettings();
  delay(4000);
  ESP.restart();
}

void setup() {
  Serial.begin(115200);
  Serial.println("Water heater controller - Keritech Electronics - 2023");
  pinMode(relay_gpio, OUTPUT);
  pinMode(button_gpio, INPUT_PULLUP);
  // WLAN initialization with WiFiManager
  WiFi.mode(WIFI_STA);
  wm.setSaveParamsCallback(setup);
  wm.setDebugOutput(false);
  wm.addParameter(&ntp_server);
  wm.addParameter(&mqtt_server);
  wm.addParameter(&mqtt_topic_root);
  wm.addParameter(&tariff);
  // connect WLAN
  wm.autoConnect(ap_ssid);
  Serial.print("WiFi IP: ");
  Serial.println(WiFi.localIP());
  // set local timezone to pre-configured
  setenv("TZ", timezone, 1);
  // NTP initialization
  Serial.print("Connecting to NTP server: ");
  Serial.println(ntp_server.getValue());
  ntpClient.begin();
  ntpClient.update();
  check_time();
  Serial.print("Time updated. Current local time is: ");
  Serial.println(asctime(lt));
  mqtt_setup();
}

void loop() {
  ntpClient.update();
  if (mqtt_enable) {
    mqtt_reconnect();
    mqttClient.loop();
  }
  check_time();
  check_tariff();
  handle_button();
  handle_relay();
  debug_info();
}
