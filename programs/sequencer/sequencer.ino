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
  byte mappedValue = map(sensorValue, 0, 1024, 250, 5000);

  if (digitalRead(push) == HIGH) {
    digitalWrite(Relay1, LOW);
    delay(mappedValue);
    digitalWrite(Relay2, LOW);
    delay(mappedValue);
    digitalWrite(Relay3, LOW);
    delay(mappedValue);
    digitalWrite(Relay4, LOW);
  } else {
    digitalWrite(Relay4, HIGH);
    delay(mappedValue);
    digitalWrite(Relay3, HIGH);
    delay(mappedValue);
    digitalWrite(Relay2, HIGH);
    delay(mappedValue);
    digitalWrite(Relay1, HIGH);
  }
}
