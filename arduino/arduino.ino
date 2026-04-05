#include <Servo.h>

Servo headServo;
Servo blinkServo;

String input = "";

void setup() {
  Serial.begin(9600);
  headServo.attach(9);
  blinkServo.attach(10);

  headServo.write(90);
  blinkServo.write(90);
}

void doBlink() {
  blinkServo.write(40);
  delay(120);
  blinkServo.write(90);
}

void loop() {
  if (Serial.available()) {
    input = Serial.readStringUntil('\n');
    input.trim();

    int headVal = 90;
    int blinkVal = 0;

    int headIdx = input.indexOf("HEAD:");
    int blinkIdx = input.indexOf("BLINK:");

    if (headIdx != -1) {
      int commaIdx = input.indexOf(',', headIdx);
      String headStr;
      if (commaIdx != -1) {
        headStr = input.substring(headIdx + 5, commaIdx);
      } else {
        headStr = input.substring(headIdx + 5);
      }
      headVal = headStr.toInt();
      headVal = constrain(headVal, 60, 120);
      headServo.write(headVal);
    }

    if (blinkIdx != -1) {
      String blinkStr = input.substring(blinkIdx + 6);
      blinkVal = blinkStr.toInt();
      if (blinkVal == 1) {
        doBlink();
      }
    }
  }
}