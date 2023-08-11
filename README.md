# heater_ntp
An IoT storage water heater controller with timer control, NTP time synchronization, MQTT status reporting and remote control.
For standalone use, or for integration with home automation systems.

# How it works

On power-up, the LED lights up blue and the firmware tries connecting with a wi-fi network if it was set via WifiManager. Failing that, it exposes an unsecured SSID ("HeaterController"), with a configuration web interface available at 192.168.4.1. After saving config, the firmware will try to connect to the specified wi-fi network, then connect to a specified NTP server and update local time (currently hardcoded to CET/CEST).
Now, the LED lights up purple to indicate that the wi-fi and NTP setup was successful.
The next step is MQTT server connection attempt; the firmware will keep trying to connect 10 times, for 5 seconds each. If successful, the LED will light up white for a second. If failed, next MQTT recconnection attempt will take place after 10 minutes.

After wi-fi, NTP and MQTT setup, normal operation begins. The firmware maintains connections, checks time and decides whether the heater should be on or off. It is possible to manually override the automatic on/off using the control button. The MQTT interface allows for controlling 
```mqtt_topic_root/``` message: ```?```: check status,
```mqtt_topic_root/start_hours``` / ```mqtt_topic_root/stop_hours``` message: comma-separated list of ints: change start or stop hours,
```mqtt_topic_root/manual``` message: int {0, 1}: manual override OFF or ON
