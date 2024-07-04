#ifndef PID_H
#define PID_H

#include "Arduino.h"

class Error{

  public:
    double p; //proportional value of error
    double i; //integral value of error
    double d; //derivative value of error
    double eLog[3]; //array to hold i, i-1, and i-2 values of error
    Error();
    /*
      Given a setpoint, reading, and time interval dt, this method will update the values of p, i, and d accordingly
    */
    void updateError(double setpoint, double reading, double dt);

};
#endif