#ifndef VACUUM_H
#define VACUUM_H

#include "Arduino.h"

class Vacuum {

  public:
  //setup the pump, valve and pump sense pins
  Vacuum();

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
