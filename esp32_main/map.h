#ifndef MAP_H
#define MAP_H

#include "Arduino.h"

class Map{

  public:
    Map(/*int numStations, String _stations[numStations]*/);
    int getFacingDirection(); //1 is to the right, -1 is to the left
    int getDrivingDirection(); //1 is forward, -1 is backward
    int getMovingDirection(); //1 is moving right, -1 is moving left
    //String getLocation(); //# between 0 and 4
    int getLocation();
    void flipFacingDirection(); //flips facing direction
    void flipDrivingDirection(); //flips driving direction
    String getState();

  protected:
    //String stations[];
    int facingDirection;
    int drivingDirection;
    int location;
    void IRAM_ATTR updateLocationRight(); //when right tape sensor detects tape
    void IRAM_ATTR updateLocationLeft(); //when left tape sensor detect tape
    int stateIndex;
    String states[4] = {"Speeding", "Adjusting", "Spinning", "Grabbing"};//adjust size as you add states in constructor

};
#endif