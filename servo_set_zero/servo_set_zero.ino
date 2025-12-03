#include <Servo.h>

Servo myservo;
const int SV_PIN = 7;

void setup() {
  Serial.begin(115200);

  myservo.attach(SV_PIN, 500, 2400);
  myservo.write(0);
}

void loop() {}