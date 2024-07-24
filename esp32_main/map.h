#ifndef MAP_H
#define MAP_H

#include "Arduino.h"
#include "definitions.h"

class Map{

  public:
    Map();
    int getFacingDirection(); //1 is to the right, -1 is to the left
    int getDrivingDirection(); //1 is forward, -1 is backward
    int getMovingDirection(); //1 is moving right, -1 is moving left
    // int getLocation(); //5 total locations, separated by black tape lines on middle row stations
    void flipFacingDirection(); //flips facing direction
    void flipDrivingDirection(); //flips driving direction
    State getState(); //returns current state of what the robot is trying to do
    // void IRAM_ATTR updateLocationRight(); //when right tape sensor detects tape
    // void IRAM_ATTR updateLocationLeft(); //when left tape sensor detect tape
    // int location;
    State state;
    Instruction getNextInstruction();
    double getNextLocation();
    void nextRecipe();


  protected:
    int facingDirection;
    int drivingDirection;
    int locationIndex;
    int instructionIndex;
    int recipeIndex;
    Instruction *currentRecipeInstructions;
    double *currentRecipeLocations;
    Food recipes[1] = {TEST};
    Instruction burgerInstructions[22] = {GO, GRAB, TURN, GO, PLACE, TURN, GO, GRAB, TURN, GO, PLACE, WAIT, GRAB, GO, PLACE, TURN, GO, GRAB, TURN, GO, PLACE, END};
    double burgerLocations[7] = {BUNS, PLATES, PATTIES, COOKTOP, PLATES, BUNS, PLATES};
    Instruction testInstructions[4] = {GO, TURN, GO, END};
    double testLocations[2] = {COOKTOP, PATTIES};

};
#endif