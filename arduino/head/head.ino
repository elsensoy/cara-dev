#include <Servo.h>

Servo earL, earR, eyeL, eyeR;

void setup() {
  Serial.begin(9600);
  earL.attach(3);  // Example pins
  earR.attach(5);
  // Initialize to neutral
  earL.write(90); earR.write(90);
}

void loop() {
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    
    if (cmd == 'H') { // Happy: Ears perk up
      earL.write(160); earR.write(20); 
    } 
    else if (cmd == 'B') { // Blink/Sad: Ears droop
      earL.write(30); earR.write(150);
    } 
    else if (cmd == 'N') { // Neutral
      earL.write(90); earR.write(90);
    }
  }
}
