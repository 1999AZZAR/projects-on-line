#include "config.h"

void setup() {
  Serial.begin (9600);
  pinMode (greenled, OUTPUT);
  pinMode (redled, OUTPUT);
  pinMode (relay, OUTPUT);
  pinMode (blueled, OUTPUT);
  pinMode (incontrol, INPUT);
}

void loop() {
  while (Serial.available() > 0) {
    result = Serial.read();
    switch (result)
    {
      case '1': {
          if (digitalRead(incontrol) == HIGH) {
            digitalWrite(greenled, HIGH);
            digitalWrite(redled, LOW);
            digitalWrite(relay, HIGH);
            digitalWrite(blueled, HIGH);
            delay(ACCESS_DELAY);
          } else {
            digitalWrite(greenled, HIGH);
            digitalWrite(redled, HIGH);
            digitalWrite(relay, LOW);
            digitalWrite(blueled, LOW);
            delay(DENIED_DELAY);
          }
        }
        break;
      case '2': {
          if (digitalRead(incontrol) == HIGH) {
            digitalWrite(greenled, LOW);
            digitalWrite(redled, HIGH);
            digitalWrite(relay, LOW);
            digitalWrite(blueled, HIGH);
            delay(DENIED_DELAY);
          } else {
            digitalWrite(greenled, LOW);
            digitalWrite(redled, HIGH);
            digitalWrite(relay, LOW);
            digitalWrite(blueled, LOW);
            delay(DENIED_DELAY);
          }
        }
        break;
      case '3': {
          if (digitalRead(incontrol) == HIGH) {
            digitalWrite(greenled, LOW);
            digitalWrite(redled, LOW);
            digitalWrite(relay, LOW);
            digitalWrite(blueled, HIGH);
            delay(DENIED_DELAY);
          } else {
            digitalWrite(greenled, LOW);
            digitalWrite(redled, LOW);
            digitalWrite(relay, LOW);
            digitalWrite(blueled, LOW);
            delay(DENIED_DELAY);
          }
          break;
        }
    }
  }
  delay (100);
}
