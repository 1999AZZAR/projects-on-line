//library
#include "ESP8266WiFi.h"
#include "ESP8266WiFiMulti.h"
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include "DHT.h"

//define pin output
#define Relay1            D2
#define Relay2            D3

//define dht module
#define DHTPIN            D4
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

//Defining the instance of ESP8266MultiWifi
WiFiClient client;
ESP8266WiFiMulti wifi_multi;

//wifi config
//WiFi 1
const char* ssid1 = "ora_bayar";
const char* password1 = "password";
//WiFi 2
const char* ssid2 = "Netnonik";
const char* password2 = "hey_there";
//WiFi 3
const char* ssid3 = "basal";
const char* password3 = "go_there";

//Defines the TimeOut(ms) which will be used to try and connect with any specific Access Point
uint16_t connectTimeOutPerAP = 5000;

//aio config
//adafruit config
#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  1883                   // use 8883 for SSL
#define AIO_USERNAME    "Azzar"
#define AIO_KEY         "aio_THCz74XRMp9B7dLQCLZll9yu8chV"

//Set up the adafruit mqtt client
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);
//Set up the feed you're subscribing to
Adafruit_MQTT_Subscribe Light1 = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME"/feeds/relay0");
Adafruit_MQTT_Subscribe Light2 = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/relay2");
//Set up the feed you're publishing to
Adafruit_MQTT_Publish temp   = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/sensors.temperature");
Adafruit_MQTT_Publish humi   = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/sensors.humidity");
Adafruit_MQTT_Publish tum1   = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME"/feeds/relay0"); 
Adafruit_MQTT_Publish tum2   = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/relay2");
