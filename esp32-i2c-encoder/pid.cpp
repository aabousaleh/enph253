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

Error *updateError(Error *e, double s, double r, double dt) {
  //distances d = {1,2};
  e->eLog[2] = e->eLog[1];
  e->eLog[1] = e->eLog[0];
  e->eLog[0] = s - r;

  e->p = e->eLog[0];
  e->d = (e->eLog[0] - e->eLog[2]) / (2*dt);
  e->i += (e->eLog[0] + 4 * e->eLog[1] + e->eLog[2]) * dt / 3;

  return e;
};