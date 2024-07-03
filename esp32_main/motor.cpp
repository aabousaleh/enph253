#include "motor.h"

Motor::Motor(int _pinA, int _pinB, float _maxSpeed){
  pinA = _pinA;
  pinB = _pinB;
  maxSpeed = _maxSpeed;

  pinMode(pinA, OUTPUT);
  pinMode(pinB, OUTPUT);

  digitalWrite(pinA, LOW);
  digitalWrite(pinB, LOW);

};

void Motor::setSpeed(float speed) {
  int pwm = speed / maxSpeed * 255;
  if (pwm >= 0) {
    analogWrite(pinB, 0);
    analogWrite(pinA, pwm);
  } else {
    analogWrite(pinA, 0);
    analogWrite(pinB, -pwm);
  }
};