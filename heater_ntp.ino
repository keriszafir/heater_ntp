/* Simple programmable heater controller
Developed for Olimex ESP8266 eval board
2023 Keri Szafir, Keritech Electronics
*/

#include <FS.h>
#include <ArduinoJson.h>
#include <Adafruit_NeoPixel.h>
#include <NTPClient.h>
#include <time.h>
#include <PubSubClient.h>
#include <WiFiManager.h>
#include <WiFiUdp.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// #define RESET_SETTINGS
// #define DEBUG

#ifdef ESP32
  // test & development on ESP32
  #include <SPIFFS.h>
  #define DEBUG
  // ESP32 pins different than prod
  #define relay_gpio 2
  #define button_gpio 13
  #define W1_gpio 4
  #define NP_gpio 14
#else
  // Olimex module pins on prod, customize if necessary
  #define relay_gpio 5
  #define button_gpio 14
  #define W1_gpio 4
  #define NP_gpio 12
#endif


const int red[3] = {255, 0, 0};
const int green[3] = {0, 255, 0};
const int blue[3] = {0, 0, 255};
const int orange[3] = {255, 165, 0};
const int purple[3] = {160, 0, 255};
const int white[3] = {255, 255, 255};
const int off[3] = {0, 0, 0};
const uint8_t brightness = 5;

// initial WLAN SSID and parameters in the config
const char ap_ssid[40] = "HeaterController";
char ntp_server[40] = "192.168.1.1";
char mqtt_server[40] = "192.168.1.1";
char mqtt_topic_root[40] = "heater";
char start_hours[40] = "5,6,13,14";
char stop_hours[40] = "7,16";


OneWire oneWire(W1_gpio);
DallasTemperature sensors(&oneWire);
WiFiClient espClient;
WiFiManager wm;
PubSubClient mqttClient(espClient);
WiFiUDP ntpUDP;
NTPClient ntpClient(ntpUDP, ntp_server, 0, 60000);

Adafruit_NeoPixel pixels(1, NP_gpio, NEO_RGB + NEO_KHZ800);

WiFiManagerParameter par_ntp_server("ntp_server", "NTP server", ntp_server, 40);
WiFiManagerParameter par_mqtt_server("mqtt_server", "MQTT Server", mqtt_server, 40);
WiFiManagerParameter par_mqtt_topic_root("topic", "MQTT Topic Root", mqtt_topic_root, 40);
WiFiManagerParameter par_start_hours("start_hours", "Start Hours (comma separated)", start_hours, 40);
WiFiManagerParameter par_stop_hours("stop_hours", "Stop Hours (comma separated)", stop_hours, 40);

const char* timezone = "CET-1CEST,M3.5.0,M10.5.0/3";
// control states
int manual = 0;
int timer = 0;
int relay_state = 0;
// time between MQTT reconnection attempts in minutes
int mqtt_rc_interval = 10;
// time structure
struct tm * lt;
// current temperature
float temp;

void led(const int* color) {
  //control led
  char info[30];
  sprintf(info, "LED R: %d, G: %d, B: %d", color[0], color[1], color[2]);
  dbgprintln(info);
  pixels.setBrightness(brightness);
  pixels.setPixelColor(0, pixels.Color(color[0], color[1], color[2]));
  pixels.show();
}

void wait(unsigned long t) {
  // wait for specified time and do usual infrastructure-related stuff
  unsigned long ts = millis();
  while (millis() - ts < t) {
    delay(1);
    wm.process();
    ntpClient.update();
    mqtt_reconnect();
    mqttClient.loop();
  }
}

void mqtt_report() {
  // quick exit if no mqtt
  if (mqttClient.connected()) {
    dbgprintln("Sending MQTT report...");
    char msg[200], subtopic[50];
    sprintf(msg, "{time: %s, start hours: %s, stop hours: %s, timer control: %d, manual control: %d, working: %d, temp: %f}", 
            asctime(lt), start_hours, stop_hours, timer, manual, relay_state, temp);
    sprintf(subtopic, "{%s/status}", mqtt_topic_root);
    mqttClient.publish(subtopic, msg);
  }
}

void handle_button() {
  if (!digitalRead(button_gpio)) {
    // start and end time
    unsigned long st, et;
    st = millis();

    while(!digitalRead(button_gpio)) {
      // wait for button release
      wait(10);
      et = millis();
      if (et - st > 30000UL) led(red);
      else if (et - st > 5000UL) led(orange);
      else if (et - st > 500UL) led(blue);
    }

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

int test_hr(int hr, char* hrlist) {
  char info[70];
  char hrs[40];
  sprintf(hrs, "%s", hrlist);
  sprintf(info, "testing if current hour %d is in list: %s... ", hr, hrs);
  dbgprint(info);

  char* p = strtok(hrs, ",");
  while (p != NULL) {
    if (atoi(p) == hr) {
      dbgprintln("found!");
      return 1;
    }
    p = strtok(NULL, ",");
  }
  dbgprintln("no match...");
  return 0;
}

void check_working_hours() {
  char info[100];
  sprintf(info, "start hours: %s, stop hours: %s", start_hours, stop_hours);
  dbgprintln(info);
  int te = timer;
  int cur_hr = lt->tm_hour;
  if (test_hr(cur_hr, start_hours)){
    te = 1;
    dbgprintln("timer control power ON");
  } else if (test_hr(cur_hr, stop_hours)) {
    te = 0;
    dbgprintln("timer control power OFF");
  }
  if (te != timer) {
  // update state and inform on mqtt
  timer = te;
  mqtt_report();
  }
}

void handle_relay() {
  int prev_relay_state = relay_state;
  if (timer || manual) {
    digitalWrite(relay_gpio, HIGH);
    relay_state = 1;
    led(red);
  } else {
    digitalWrite(relay_gpio, LOW);
    relay_state = 0;
    led(green);
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
  
  String subtopic = String(topic);
  subtopic.replace(mqtt_topic_root, "");
  // poll the parameters
  if (!strcmp(msg.c_str(), "?")) mqtt_report();
  // message interpretation logic
  if (!strcmp(subtopic.c_str(), "/manual")) manual = msg.toInt();
  else if (!strcmp(subtopic.c_str(), "/start_hours")) {
    strcpy(start_hours, msg.c_str());
    save();
  } else if (!strcmp(subtopic.c_str(), "/stop_hours")) {
    strcpy(stop_hours, msg.c_str());
    save();
  }
}

bool mqtt_reconnect() {
  static unsigned long last_mqtt_rc_attempt_time;
  static int mqtt_rc_attempts_left = 10;
  char info[50];

  // MQTT reconnection routine; do nothing if already connected
  if (mqttClient.connected()) return true;
  // check if specified time interval passed since last MQTT reconnect attempt
  if (ntpClient.getEpochTime() - last_mqtt_rc_attempt_time > mqtt_rc_interval * 60) {
    // reset the attempts counter
    mqtt_rc_attempts_left = 10;
  }
  
  if (mqtt_rc_attempts_left <= 0) return false;

  led(purple);
  // Loop until we're reconnected or 10 attempts failed
  // reset this counter on successful connection
  while ((!mqttClient.connected()) && (mqtt_rc_attempts_left > 0)) {
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
      // indicate mqtt connection
      led(white);
      delay(1000);
      return true;
    } else {
      mqtt_rc_attempts_left -= 1;
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.print(", attempts left: ");
      Serial.print(mqtt_rc_attempts_left);
      if (mqtt_rc_attempts_left > 0) {
        Serial.println(" - try again in 5 seconds");
        delay(5000);
      } else break;
    }
  }
  // fell off the end of the while loop = hold on for mqtt_rc_interval
  last_mqtt_rc_attempt_time = ntpClient.getEpochTime();
  sprintf(info, "\nMQTT connection failed - try again in %d minutes", mqtt_rc_interval);
  Serial.println(info);
  return false;
}

void mqtt_setup() {
  // MQTT initialization
  Serial.print("Connecting to MQTT server: ");
  Serial.println(mqtt_server);
  if (strcmp(mqtt_server, "")) {
    // MQTT initialization if not left empty
    mqttClient.setServer(mqtt_server, 1883);
    mqttClient.setCallback(mqtt_callback);
    if (!mqtt_reconnect()) return;
    char msg[150];
    sprintf(msg, "Keritech Electronics heater controller at %s", WiFi.localIP().toString().c_str());
    mqttClient.publish(mqtt_topic_root, msg);
    Serial.println(msg);
    mqtt_report();
    dbgprintln("MQTT is go for launch...");
  } else Serial.println("MQTT server not specified. Skipping MQTT setup.");
}

void dbgprint(char* msg) {
  #ifdef DEBUG
  Serial.print(msg);
  #endif
}

void dbgprintln(char* msg) {
  #ifdef DEBUG
  Serial.println(msg);
  #endif
}

void debug_info() {
  dbgprintln("debug info:");
  char info[80];

  sprintf(info, "time now: %sstart hours: %s, stop hours: %s", asctime(lt), start_hours, stop_hours);
  dbgprintln(info);

  sprintf(info, "timer control: %s, manual control: %s, relay: %s", 
          timer?"ON":"OFF", manual?"ON":"OFF", relay_state?"ON":"OFF");
  dbgprintln(info);

  sprintf(info, "current temperature: %f°C", temp);
  dbgprintln(info);
  wait(100UL);
}

void save_params_cb() {
  get_wm_param_values();
  save();
}

void load() {
  // configuration load routine from FS
  Serial.println("mounting FS...");

  if (SPIFFS.begin()) {
    dbgprintln("mounted file system");
    if (SPIFFS.exists("/config.json")) {
      //file exists, reading and loading
      dbgprintln("reading config file");
      File configFile = SPIFFS.open("/config.json", "r");
      if (configFile) {
        dbgprintln("opened config file");
        size_t size = configFile.size();
        // Allocate a buffer to store contents of the file.
        std::unique_ptr<char[]> buf(new char[size]);

        configFile.readBytes(buf.get(), size);

  #if defined(ARDUINOJSON_VERSION_MAJOR) && ARDUINOJSON_VERSION_MAJOR >= 6
        DynamicJsonDocument json(1024);
        auto deserializeError = deserializeJson(json, buf.get());
        serializeJson(json, Serial);
        if ( ! deserializeError ) {
  #else
        DynamicJsonBuffer jsonBuffer;
        JsonObject& json = jsonBuffer.parseObject(buf.get());
        json.printTo(Serial);
        if (json.success()) {
  #endif
          dbgprintln("\nparsed json");
          strcpy(ntp_server, json["ntp_server"]);
          strcpy(mqtt_server, json["mqtt_server"]);
          strcpy(mqtt_topic_root, json["mqtt_topic_root"]);
          strcpy(start_hours, json["start_hours"]);
          strcpy(stop_hours, json["stop_hours"]);
        } else {
          dbgprintln("failed to load json config");
        }
        configFile.close();
      }
    }
  } else {
    dbgprintln("failed to mount FS");
  }
}

void save() {
  // configuration save routine to FS
  // do nothing if no need to
  Serial.println("Saving configuration");
  #if defined(ARDUINOJSON_VERSION_MAJOR) && ARDUINOJSON_VERSION_MAJOR >= 6
    DynamicJsonDocument json(1024);
  #else
    DynamicJsonBuffer jsonBuffer;
    JsonObject& json = jsonBuffer.createObject();
  #endif

  json["ntp_server"] = ntp_server;
  json["mqtt_server"] = mqtt_server;
  json["mqtt_topic_root"] = mqtt_topic_root;
  json["start_hours"] = start_hours;
  json["stop_hours"] = stop_hours;

  File configFile = SPIFFS.open("/config.json", "w");
  if (!configFile) {
    dbgprintln("failed to open config file for writing");
  }

  #if defined(ARDUINOJSON_VERSION_MAJOR) && ARDUINOJSON_VERSION_MAJOR >= 6
    serializeJson(json, configFile);
  #else
    json.printTo(configFile);
  #endif
    configFile.close();
    //end save
  // update the wifimanager fields
  set_wm_param_values();
}

void get_wm_param_values() {
  // update runtime variables with wifimanager field contents
  dbgprintln("updating parameters");
  strcpy(ntp_server, par_ntp_server.getValue());
  strcpy(mqtt_server, par_mqtt_server.getValue());
  strcpy(mqtt_topic_root, par_mqtt_topic_root.getValue());
  strcpy(start_hours, par_start_hours.getValue());
  strcpy(stop_hours, par_stop_hours.getValue());
}

void set_wm_param_values() {
  // update WM parameters with current runtime variable values
  dbgprintln("configuring and adding parmeters");
  par_ntp_server.setValue(ntp_server, 40);
  par_mqtt_server.setValue(mqtt_server, 40);
  par_mqtt_topic_root.setValue(mqtt_topic_root, 40);
  par_start_hours.setValue(start_hours, 40);
  par_stop_hours.setValue(stop_hours, 40);
}

void wifi_setup() {
  #ifdef RESET_SETTINGS
  wm.resetSettings();
  #endif
  #ifdef DEBUG
  wm.setDebugOutput(true);
  #endif

  set_wm_param_values();
  wm.addParameter(&par_ntp_server);
  wm.addParameter(&par_mqtt_server);
  wm.addParameter(&par_mqtt_topic_root);
  wm.addParameter(&par_start_hours);
  wm.addParameter(&par_stop_hours);
  Serial.println("Initializing WLAN...");
  WiFi.mode(WIFI_STA);
  wm.setClass("invert");
  wm.setConnectTimeout(180);
  wm.setMinimumSignalQuality(10);
  wm.setConfigPortalBlocking(false);
  wm.setSaveParamsCallback(save_params_cb);
  dbgprintln("attempting WLAN autoconnect...");
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
  pixels.begin();  
  led(blue);
  SPIFFS.begin();
  Serial.begin(115200);
  Serial.println();
  Serial.println("Water heater controller - Keritech Electronics - 2023");
  pinMode(relay_gpio, OUTPUT);
  pinMode(button_gpio, INPUT_PULLUP);
  sensors.begin();
  check_temp();
  load();
  wifi_setup();
  ntp_setup();
  mqtt_setup();
  dbgprintln("Setup procedure completed.");
}

void loop() {
  dbgprintln("looping wifimanager...");
  wm.process();
  dbgprintln("updating NTP...");
  ntpClient.update();
  dbgprintln("reconnecting MQTT if necessary...");
  mqtt_reconnect();
  dbgprintln("looping MQTT client...");
  mqttClient.loop();
  dbgprintln("checking current time...");
  check_time();
  dbgprintln("checking if heater should be working now...");
  check_working_hours();
  dbgprintln("checking temperature...");
  check_temp();
  dbgprintln("control logic...");
  handle_button();
  handle_relay();
  debug_info();
}
