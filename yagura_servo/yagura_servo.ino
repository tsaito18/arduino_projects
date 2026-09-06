#include <Servo.h>

Servo myservo;
const int SV_PIN = 4;
const bool IS_270_DEGREES = true;

int rawVal = 0;
int targetVal = 0;
int currentVal = 0;
int speedDelay = 20;

void setup() {
  Serial.begin(115200);

  myservo.write(currentVal);
  myservo.attach(SV_PIN, 500, 2400);

  Serial.println("Ready.");
}

void loop() {
  if (Serial.available()) {
    rawVal = Serial.parseInt();

    // 残った改行コードなどを捨てる
    while (Serial.available() > 0) { Serial.read(); }

    if (IS_270_DEGREES) {
      rawVal = constrain(rawVal, 0, 270);
      targetVal = (int)(rawVal * 180.0 / 270.0);
    } else {
      rawVal = constrain(rawVal, 0, 180);
      targetVal = rawVal;
    }

    Serial.print("Move to: ");
    Serial.print(rawVal);

    if (targetVal > currentVal) {
      for (int i = currentVal; i <= targetVal; i++) {
        myservo.write(i);
        delay(speedDelay);
      }
    } else if (targetVal < currentVal) {
      for (int i = currentVal; i >= targetVal; i--) {
        myservo.write(i);
        delay(speedDelay);
      }
    }

    currentVal = targetVal;

    Serial.println(" ... Done.");
  }
}
