#include <Wire.h>
#include <LiquidCrystal_I2C(0x27, 16, 2);

void setup() {
  lcd.init();
  lcd.backlight();
  Serial.begin(115200);
}

void.loop() {
  if (Serial.available()) {
    delay(100);
    lcd.setCursor(0, 0);
    while (Serial.available() > 0{
    lcd.write(Serial.read());
    }
  }
}
