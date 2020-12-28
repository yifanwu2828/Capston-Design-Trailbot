#include <AFMotor.h>

AF_DCMotor motor(1);

void setup() {
  Serial.begin(9600);
  motor.setSpeed(100);
}

void loop() {
  motor.run(FORWARD);
  delay(100);
  motor.run(RELEASE);
}
