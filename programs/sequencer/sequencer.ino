#include "config.h"

void setup() {
  pinMode(Relay1, OUTPUT);
  pinMode(Relay2, OUTPUT);
  pinMode(Relay3, OUTPUT);
  pinMode(Relay4, OUTPUT);
  pinMode(push, INPUT);

  digitalWrite(Relay1, HIGH);
  digitalWrite(Relay2, HIGH);
  digitalWrite(Relay3, HIGH);
  digitalWrite(Relay4, HIGH);
}

void loop() {
  sensorValue = analogRead(A0);
  deltime = map(sensorValue, 0, 1024, 1, 255) * 50;

  if (digitalRead(push) == HIGH) {
    digitalWrite(Relay1, LOW);
    delay(deltime);
    digitalWrite(Relay2, LOW);
    delay(deltime);
    digitalWrite(Relay3, LOW);
    delay(deltime);
    digitalWrite(Relay4, LOW);
  }

  else {
    digitalWrite(Relay4, HIGH);
    delay(deltime);
    digitalWrite(Relay3, HIGH);
    delay(deltime);
    digitalWrite(Relay2, HIGH);
    delay(deltime);
    digitalWrite(Relay1, HIGH);
  }
}
