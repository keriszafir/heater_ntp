/* Simple heater controller for G11, G12, G12r tariffs
Developed for Olimex ESP8266 eval board
2023 Keri Szafir, Keritech Electronics
*/

#include <NTPClient.h>
#include <WiFiManager.h>
#include <WiFiUdp.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// initial WLAN SSID and parameters in the config
const char *ap_ssid = "StiebelEltron";
WiFiManagerParameter ntp_server("ntp_server", "ntp server", "192.168.4.1", 40);
WiFiManagerParameter timezone_offset("timezone_offset", "timezone offset", "2", 40);
WiFiManagerParameter tariff("tariff", "tariff", "G12r", 5);
//WiFiManagerParameter mqtt_server("mqtt_server", "mqtt server", "192.168.4.1", 40);
//WiFiManagerParameter mqtt_topic_root("topic", "mqtt topic root", "mtzvfdclk", 40);

WiFiClient espClient;
WiFiUDP ntpUDP;
NTPClient ntpClient(ntpUDP, ntp_server.getValue(), 0, 360000);

#define relay_gpio 5
#define button_gpio 0

// initial off
int heater_enable = 0;
int timer_enable = 0;
int relay_state = 0;
int manual_override = 0;
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
  if ((heater_enable && timer_enable) || manual_override) {
    digitalWrite(relay_gpio, HIGH);
    relay_state = 1;
  } else {
    digitalWrite(relay_gpio, LOW);
    relay_state = 0;
  }
  // report the state
  if (relay_state) {
    Serial.println("Heater ON");
  } else {
    Serial.println("Heater OFF");
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("Water heater controller - Keritech Electronics - 2023");
  pinMode(relay_gpio, OUTPUT);
  pinMode(button_gpio, INPUT_PULLUP);
  // WLAN initialization with WiFiManager
  WiFi.mode(WIFI_STA);
  WiFiManager wm;
  wm.setSaveParamsCallback(setup);
  wm.setDebugOutput(false);
  wm.addParameter(&ntp_server);
  wm.addParameter(&timezone_offset);
  wm.addParameter(&tariff);
  //wm.addParameter(&mqtt_server);
  //wm.addParameter(&mqtt_topic_root);
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
}

void loop() {
  check_time();
  handle_button();
  handle_relay();
}
