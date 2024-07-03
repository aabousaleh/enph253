#ifndef MOTOR_H
#define MOTOR_H

#include "Arduino.h"

class Motor{

  public:
    int pinA; //pinA of motor
    int pinB; //pinB of motor
    float maxSpeed; //max speed of motor in deg/s (MUST BE POSITIVE VALUE)
    Motor(int _pinA, int _pinB, float _maxSpeed);

    /*
      Motor *motor: pointer to the motor you want to drive
      float speed: a value <= maxSpeed, where the sign gives direction. Given in deg/s
    */
    void setSpeed(float speed);

};

#endif