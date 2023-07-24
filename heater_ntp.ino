/* Simple heater controller for G11, G12, G12r tariffs
Developed for Olimex ESP8266 eval board
2023 Keri Szafir, Keritech Electronics
*/

#include <NTPClient.h>
#include <PubSubClient.h>
#include <WiFiManager.h>
#include <WiFiUdp.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// initial WLAN SSID and parameters in the config
const char *ap_ssid = "StiebelEltron";
WiFiManagerParameter ntp_server("ntp_server", "ntp server", "192.168.4.1", 40);
WiFiManagerParameter timezone_offset("timezone_offset", "timezone offset", "2", 40);
WiFiManagerParameter tariff("tariff", "tariff", "G12r", 5);
WiFiManagerParameter mqtt_server("mqtt_server", "mqtt server", "", 40);
WiFiManagerParameter mqtt_topic_root("topic", "mqtt topic root", "heater", 40);

WiFiClient espClient;
WiFiManager wm;
PubSubClient mqttClient(espClient);
WiFiUDP ntpUDP;
NTPClient ntpClient(ntpUDP, ntp_server.getValue(), atoi(timezone_offset.getValue()), 360000);

#define relay_gpio 5
#define button_gpio 4

// initial off
int mqtt_enable = 0;
int heater_enable = 0;
int timer_enable = 0;
int relay_state = 0;
int manual_override = 0;
int mqtt_failed_rc_attempts = 0;
int ct = 0;
int debounce_time = 200;

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
    heater_enable = !heater_enable;
    Serial.print("Device is ");
    Serial.println(heater_enable ? "ON" : "OFF");
  }
  while(!digitalRead(button_gpio)) {
    // wait for button release
  }
}

void check_time() {
  int hour = ntpClient.getHours();
  if (tariff.getValue() == "G12r") {
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
  } else if (tariff.getValue() == "G12") {
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
}

void handle_relay() {
  int prev_relay_state = relay_state;
  if ((heater_enable && timer_enable) || manual_override) {
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
  Serial.println(msg);
  
  // message interpretation logic
  String subtopic = String(topic);
  subtopic.replace(mqtt_topic_root.getValue(), "");
  if (subtopic == "/manual_override") {
    manual_override = msg.toInt();
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

void reset_wm() {
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
  wm.addParameter(&timezone_offset);
  wm.addParameter(&tariff);
  wm.addParameter(&mqtt_server);
  wm.addParameter(&mqtt_topic_root);
  // connect WLAN
  wm.autoConnect(ap_ssid);
  // NTP initialization
  Serial.print("Connecting to NTP server: ");
  Serial.println(ntp_server.getValue());
  // update time every hour
  ntpClient.begin();
  // set timezone
  ntpClient.update();
  Serial.print("Time updated. Current local time is: ");
  Serial.println(ntpClient.getFormattedTime());
  //  set MQTT
  const char* mqtt_address = mqtt_server.getValue();
  if (mqtt_address) {
    mqtt_enable = 1;
    // MQTT initialization if not left empty
    Serial.print("Connecting to MQTT server: ");
    Serial.println(mqtt_address);
    mqttClient.setServer(mqtt_address, 1883);
    mqttClient.setCallback(mqtt_callback);
    mqtt_reconnect();

    char msg[150];
    sprintf(msg, "Keritech heater controller at %s", WiFi.localIP().toString().c_str());
    mqttClient.publish(mqtt_topic_root.getValue(), msg);
    Serial.println(msg);
  }
}

void loop() {
  if (mqtt_enable) {
    mqtt_reconnect();
    mqttClient.loop();
  }
  check_time();
  handle_button();
  handle_relay();
}
