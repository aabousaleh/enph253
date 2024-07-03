#ifndef PID_H
#define PID_H

#include "Arduino.h"

class Error{

  public:
    double p;
    double i;
    double d;
    double eLog[3];
    Error();
    void updateError(Error *error, double setpoint, double reading, double dt);

};
#endif