#include motor.h

Motor::Motor(int pin1, int pin2){
  pinA = pin1;
  pinB = pin2;

  pinMode(pinA, OUTPUT);
  pinMode(pinB, OUTPUT);

  digitalWrite(pinA, LOW);
  digitalWrite()

};

/*
  Motor *motor: pointer to the motor you want to drive
  int pwm: a value between -255 and 255, where the sign gives direction
*/
void setSpeed(Motor *motor, int pwm) {
  if (pwm >= 0) {
    analogWrite(pinA, pwm);
    analogWrite(pinB, 0);
  } else {
    analogWrite(pinA, 0);
    analogWrite(pinB, -pwm);
  }
};