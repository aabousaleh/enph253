#include "pid.h"
// updateError(Error error_object, double setpoint, double reading, double time_interval)

// updates error object with new error, derivative of error, and integral of error values
// */
Error::Error(long _iMax) {
  p = 0;
  i = 0;
  d = 0;
  iMax = _iMax;
  eLog[0] = 0;
  eLog[1] = 0;
  eLog[2] = 0;
};

void Error::updateError(double s, double r, double dt) {
  eLog[2] = eLog[1];
  eLog[1] = eLog[0];
  eLog[0] = s - r;

  p = eLog[0];
  d = (eLog[0] - eLog[2]) / (2*dt);
  double newI = 5 * eLog[0] * dt;  //random 5*
  //(eLog[0] + 4*eLog[1] + eLog[2]) * dt / 3;
  // if (abs(newI) < 2400) {
  //   i += newI;
  // }
  i += newI;
  if (abs(i) > iMax) i = iMax * i / abs(i);
  //if (i < 0) i =0;

  // Serial.print("Setpoint, Reading, and P-Error: ");
  // Serial.print(s);
  // Serial.print(" ");
  // Serial.print(r);
  // Serial.print(" ");
  // Serial.println(p);
};