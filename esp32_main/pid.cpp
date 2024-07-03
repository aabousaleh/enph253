#include "pid.h"
// updateError(Error error_object, double setpoint, double reading, double time_interval)

// updates error object with new error, derivative of error, and integral of error values
// */
Error::Error() {
  p = 0;
  i = 0;
  d = 0;
  eLog[0] = 0;
  eLog[1] = 0;
  eLog[2] = 0;
};

void Error::updateError(double s, double r, double dt) {
  //distances d = {1,2};
  eLog[2] = eLog[1];
  eLog[1] = eLog[0];
  eLog[0] = s - r;

  p = eLog[0];
  d = (eLog[0] - eLog[2]) / (2*dt);
  i += (eLog[0] + 4 * eLog[1] + eLog[2]) * dt / 3;

  Serial.print("Setpoint, Reading, and P-Error: ");
  Serial.print(s);
  Serial.print(" ");
  Serial.print(r);
  Serial.print(" ");
  Serial.println(p);
  //return e;
};