#include "config.h"

char* str;
String payload;
uint32_t present;
bool first_time;
uint16_t  scrollDelay;
#define  CHAR_SPACING  1

#define BUF_SIZE  75
char curMessage[BUF_SIZE];
char newMessage[BUF_SIZE];
bool newMessageAvailable = false;

WiFiClient client;

Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);
Adafruit_MQTT_Subscribe minidisplay = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/display.minidisplay");

void MQTT_connect();

//MD_MAX72XX mx = MD_MAX72XX(CS_PIN, MAX_DEVICES);
MD_MAX72XX mx = MD_MAX72XX(HARDWARE_TYPE, CS_PIN, MAX_DEVICES);
uint8_t scrollDataSource(uint8_t dev, MD_MAX72XX::transformType_t t)
{
  static char       *p = curMessage;
  static uint8_t    state = 0;
  static uint8_t    curLen, showLen;
  static uint8_t    cBuf[8];
  uint8_t colData;
  switch (state)
  {
    case 0: // Load the next character from the font table
      showLen = mx.getChar(*p++, sizeof(cBuf) / sizeof(cBuf[0]), cBuf);
      curLen = 0;
      state++;
      if (*p == '\0')
      {
        p = curMessage;
        if (newMessageAvailable)
        {
          strcpy(curMessage, str);  // copy it in
          newMessageAvailable = false;
        }
      }
    case 1:
      colData = cBuf[curLen++];
      if (curLen == showLen)
      {
        showLen = CHAR_SPACING;
        curLen = 0;
        state = 2;
      }
      break;
    case 2:
      colData = 0;
      curLen++;
      if (curLen == showLen)
        state = 0;
      break;
    default:
      state = 0;
  }
  return (colData);
}

void scrollText(void)
{
  static uint32_t   prevTime = 0;
  if (millis() - prevTime >= scrollDelay)
  {
    mx.transform(MD_MAX72XX::TSL);
    //mx.transform(MD_MAX72XX::TFLR);
    prevTime = millis();
  }
}
void  no_connection(void)
{
  newMessageAvailable = 1;
  strcpy(curMessage, "No Internet! ");
  scrollText();
}

void wifi_connect() {
  Serial.println(); Serial.println();
  Serial.print("Connecting : ");
  Serial.println(WLAN_SSID);

  WiFi.begin(WLAN_SSID, WLAN_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();

  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void setup()
{
  Serial.begin(115200);
  mx.begin();
  mx.setShiftDataInCallback(scrollDataSource);

  scrollDelay = SCROLL_DELAY;
  strcpy(curMessage, "---");
  newMessage[0] = '\0';


  Serial.print("\n[MD_MAX72XX Message Display]\nType a message for the scrolling display\nEnd message line with a newline");
  Serial.println();
  Serial.println();
  Serial.println();

  for (uint8_t t = 4; t > 0; t--) {
    Serial.printf("[SETUP] WAIT %d...\n", t);
    Serial.flush();
    delay(1000);
  }

  for (uint8_t t = 4; t > 0; t--) {
    Serial.printf("[SETUP] WAIT %d...\n", t);
    Serial.flush();
    delay(1000);
  }
  newMessageAvailable = 1;
  present = millis();
  first_time = 1;
  mqtt.subscribe(&minidisplay);
  str = "  waiting...  ";
}
void loop()
{
  unsigned long currentTime = millis();
  while (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi not connected!");
    delay(1000);
  }
  MQTT_connect();
  Adafruit_MQTT_Subscribe *subscription;
  while ((subscription = mqtt.readSubscription(1))) {
    if (subscription == &minidisplay) {
      payload = "";
      Serial.print(F("Got: "));
      Serial.println((char *)minidisplay.lastread);
      str = (char*)minidisplay.lastread;
      payload = (String) str;
      payload += "     ";
      str = &payload[0];
      newMessageAvailable = 1;
    }
  }
  scrollText();
  if (currentTime - previousTime >= eventInterval) {
    wifi_connect();
    previousTime = currentTime;
  }
}
void MQTT_connect() {
  int8_t ret;
  if (mqtt.connected()) {
    return;
  }
  Serial.print("Connecting to MQTT... ");
  uint8_t retries = 3;
  while ((ret = mqtt.connect()) != 0) {
    Serial.println(mqtt.connectErrorString(ret));
    Serial.println("Retrying MQTT connection in 5 seconds...");
    mqtt.disconnect();
    delay(5000);
    retries--;
    if (retries == 0) {
      while (1);
    }
  }
  Serial.println("MQTT Connected!");
}
