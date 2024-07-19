#ifndef PID_H
#define PID_H

#include "Arduino.h"

class Error{

  public:
    long p; //proportional value of error
    long i; //integral value of error
    long d; //derivative value of error
    long eLog[3]; //array to hold i, i-1, and i-2 values of error
    long iMax;
    Error(long _iMax);
    /*
      Given a setpoint, reading, and time interval dt, this method will update the values of p, i, and d accordingly
    */
    void updateError(double setpoint, double reading, double dt);

};
#endif