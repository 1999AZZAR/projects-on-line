//library
#include <MD_MAX72xx.h>
#include <SPI.h>
#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"

//aio config
#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  1883                               // use 8883 for SSL
#define AIO_USERNAME    "Azzar"                            // your username
#define AIO_KEY         "aio_THCz74XRMp9B7dLQCLZll9yu8chV" //your key

//definition
#define HARDWARE_TYPE MD_MAX72XX::PAROLA_HW
#define HARDWARE_TYPE MD_MAX72XX::FC16_HW
#define MAX_DEVICES 2     //how many max72xx panel
#define MAX_INTENSITY 0x1 //intensity of the max72xx
#define SCROLL_DELAY 67   //scroll speed, lower = faster

//pin connect
#define CLK_PIN   D5 // or SCK
#define DATA_PIN  D7 // or MOSI
#define CS_PIN    D8 // or SS

//millis timing
const unsigned long eventInterval = 600000; //1000 for 1s
unsigned long previousTime = 0; 

//wifi config
#define WLAN_SSID       "ora_bayar"       // Your SSID
#define WLAN_PASS       "password"        // Your password
