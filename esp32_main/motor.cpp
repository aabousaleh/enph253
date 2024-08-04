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
    ledcWrite(pinA, pwm);
    ledcWrite(pinB, 0);
  } else {
    ledcWrite(pinA, 0);
    ledcWrite(pinB, -pwm);
  }
};

void Motor::updateSpeeds(double newSpeed) {
  speeds[4] = speeds[3];
  speeds[3] = speeds[2];
  speeds[2] = speeds[1];
  speeds[1] = speeds[0];
  speeds[0] = newSpeed;
}

double Motor::averageSpeed() {
  double firstAverage = (speeds[0] + speeds[1] + speeds[2] + speeds[3] + speeds[4]) / 5;
  int outliers = 0;
  double newAverage = firstAverage;
  for (int i = 0; i < 5; i++) {
    if (abs(firstAverage - speeds[i])/ firstAverage > 0.3) {
      outliers++;
      if (outliers == 5) break;
      newAverage = ((newAverage * (6.0 - outliers)) - speeds[i]) / (5.0 - outliers);
    }
  }
  currentAverageSpeed = newAverage;
  return newAverage;
};

void Motor::clearSpeeds() {
  speeds[4] = 0;
  speeds[3] = 0;
  speeds[2] = 0;
  speeds[1] = 0;
  speeds[0] = 0;
}

void Motor::setAverageSpeeds(double speed) {
  speeds[4] = speed;
  speeds[3] = speed;
  speeds[2] = speed;
  speeds[1] = speed;
  speeds[0] = speed;
}