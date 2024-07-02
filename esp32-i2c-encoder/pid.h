#ifndef pid
#define pid

#include "Arduino.h"

class Error{

  public:
    double p;
    double i;
    double d;
    double eLog[3];
    Error();
    Error *updateError(Error *error, double setpoint, double reading, double dt);

};
#endif