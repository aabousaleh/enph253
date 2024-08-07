#include "motor.h"
// #include "analogWrite.h"

Motor::Motor(int _pinA, int _pinB, float _maxSpeed){
  pinA = _pinA;
  pinB = _pinB;
  maxSpeed = _maxSpeed;
  currentAverageSpeed = 0;

  pinMode(pinA, OUTPUT);
  pinMode(pinB, OUTPUT);

  digitalWrite(pinA, LOW);
  digitalWrite(pinB, LOW);

  ledcAttach(pinA, 250, 12);
  ledcAttach(pinB, 250, 12);

};

void Motor::setSpeed(float speed) {
  int pwm = (abs(speed) < maxSpeed) ? (speed / maxSpeed) * 4095.0 : 4095 * speed / abs(speed);
  if (pwm >= 0) {
    ledcWrite(pinB, 0);
    ledcWrite(pinA, pwm);
  } else {
    ledcWrite(pinA, 0);
    ledcWrite(pinB, -pwm);
  }
};
