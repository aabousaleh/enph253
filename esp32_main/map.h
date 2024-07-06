#ifndef MAP_H
#define MAP_H

#include "Arduino.h"

class Map{

  public:
    Map();
    int getFacingDirection(); //1 is to the right, -1 is to the left
    int getDrivingDirection(); //1 is forward, -1 is backward
    int getMovingDirection(); //1 is moving right, -1 is moving left
    int getLocation(); //5 total locations, separated by black tape lines on middle row stations
    void flipFacingDirection(); //flips facing direction
    void flipDrivingDirection(); //flips driving direction
    String getState(); //returns current state of what the robot is trying to do

  protected:
    int facingDirection;
    int drivingDirection;
    int location;
    void IRAM_ATTR updateLocationRight(); //when right tape sensor detects tape
    void IRAM_ATTR updateLocationLeft(); //when left tape sensor detect tape
    int stateIndex; //variable to keep track of the current state of the robot
    String states[5] = {"Speeding", "Adjusting", "Spinning", "Arming", "Waiting"}; //adjust size as you add states

};
#endif