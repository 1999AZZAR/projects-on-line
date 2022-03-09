//define pin
#define Relay1            D1
#define Relay2            D2
#define Relay3            D3
#define Relay4            D4
//library
#include <ESP8266WiFi.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
//wifi config
#define WLAN_SSID       ".........."             // Your SSID
#define WLAN_PASS       ".........."        // Your password
//aio config
#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  1883                   // use 8883 for SSL
#define AIO_USERNAME    "aio username"
#define AIO_KEY         "aio key"
