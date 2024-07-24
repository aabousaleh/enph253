#ifndef VACUUM_H
#define VACUUM_H

#include "Arduino.h"

class Vacuum {

  public:
  int ctrlPin; //control pin (digital out) for vacuum
  int sensPin; //current sense pin (analog in) for vacuum
  int valvePin; //control pin (digital out) for solenoid valve
  Vacuum(int _ctrlPin, int _sensPin, int _valvePin);

  /**
 * Turns vacuum end effector on or off (picks up or drops object)
 * 
 * Param: 'true' to suction object, 'false' to drop the object
 */
  void setSucc(bool succ);

  /**
 * Confirms if an object has been picked up.
 * 
 * Return: 'true' if object was picked up, 'false' otherwise
 */
  bool objSecured();
};

#endif
