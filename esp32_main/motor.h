#ifndef MOTOR_H
#define MOTOR_H

#include "Arduino.h"

class Motor{

  public:
    int pinA;
    int pinB;
    Motor(int pin1, int pin2);
    void setSpeed(int pwm);

};

#endif