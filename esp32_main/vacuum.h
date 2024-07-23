#ifndef VACUUM_H
#define VACUUM_H

#include "Arduino.h"

class Vacuum {

  public:
  int ctrlPin; //control pin (digital out) for vacuum
  int sensPin; //current sense pin (analog in) for vacuum
  int valvePin; //control pin (digital out) for solenoid valve
  Vacuum(int _ctrlPin, int _sensPin, int _valvePin);

  void setSucc(bool succ);
  bool objSecured();
}

#endif
