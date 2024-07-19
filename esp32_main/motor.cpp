#include "motor.h"
// #include "analogWrite.h"

Motor::Motor(int _pinA, int _pinB, float _maxSpeed){
  pinA = _pinA;
  pinB = _pinB;
  maxSpeed = _maxSpeed;

  pinMode(pinA, OUTPUT);
  pinMode(pinB, OUTPUT);

  digitalWrite(pinA, LOW);
  digitalWrite(pinB, LOW);

  // analogWriteFrequency(pinA, 100);
  // analogWriteFrequency(pinB, 100);
  ledcAttach(pinA, 250, 8);
  ledcAttach(pinB, 250, 8);

};

void Motor::setSpeed(float speed) {
  int pwm = (abs(speed) < maxSpeed) ? (speed / maxSpeed) * 255.0 : 255 * speed / abs(speed);
  if (pwm >= 0) {
    ledcWrite(pinB, 0);
    ledcWrite(pinA, pwm);
  } else {
    ledcWrite(pinA, 0);
    ledcWrite(pinB, -pwm);
  }
};