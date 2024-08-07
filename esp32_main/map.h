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
    void setMovingDirection(int dir);
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
    Ingredient getNextIngredient();
    void nextRecipe();


  protected:
    int facingDirection;
    int drivingDirection;
    int movingDirection;
    int locationIndex;
    int instructionIndex;
    int ingredientIndex;
    int recipeIndex;
    Instruction *currentRecipeInstructions;
    double *currentRecipeLocations;
    Ingredient *currentRecipeIngredients;
    Recipe recipes[10] = {CHEESE_PLATE, CHEESE_PLATE, CHEESE_PLATE, CHEESE_PLATE, FLING, END_RECIPE};

    //FLING
    Instruction flingInstructions[8] = {GO, GRAB, SPECIAL, END};
    double flingLocations[2] = {CHEESE, SERVING};
    Ingredient flingIngredients[1] = {cheese};

    //Burger, robot 1
    Instruction burgerInstructions[30] = {GO, GRAB, TURN, GO, RECEIVE_CHECKPOINT, PLACE, TURN, GRAB, TURN, PLACE, GO, GRAB, GO, PLACE, SEND_CHECKPOINT, RECEIVE_CHECKPOINT, GRAB, TURN, GO, END};
    double burgerLocations[7] = {CHEESE, PLATES, COOKTOP, PLATES, SERVING};
    Ingredient burgerIngredients[5] = {cheese, lettuce, patty, plate};

    //burger, robot 0
    // Instruction burgerInstructions[22] = {GO, GRAB, TURN, GO, PLACE, SEND_CHECKPOINT, TURN, GO, GRAB, TURN, GO, PLACE, GO, GRAB, GO, PLACE, TURN, GO, GRAB, TURN, GO, RECEIVE_CHECKPOINT, PLACE, SEND_CHECKPOINT, RECEIVE_CHECKPOINT, END};
    // double burgerLocations[7] = {BUNS, PLATES, PATTIES, COOKTOP, TOMATOES, PLATES, BUNS, PLATES};
    // Ingredient burgerIngredients[5] = {bottom_bun, patty, patty, top_bun, plate};

    //Salad, robot 1
    Instruction saladInstructions[20] = {GO, GRAB, TURN, PLACE, RECEIVE_CHECKPOINT, GRAB, TURN, GO, PLACE, END};
    double saladLocations[4] = {LETTUCE, SERVING};
    Ingredient saladIngredients[3] = {lettuce, plate};

    //Salad, robot 0
    // Instruction saladInstructions[10] = {GO, GRAB, GO, PLACE, END};
    // double saladLocations[2] = {TOMATOES, PLATES};
    // Ingredient saladIngredients[3] = {tomato};

    //cheese, robot 1
    Instruction cheeseInstructions[15] = {GO, GRAB, TURN, GO, PLACE, GRAB, TURN, GO, PLACE, END};
    double cheeseLocations[5] = {CHEESE, POTATOES, SERVING};
    Ingredient cheeseIngredients[2] = {cheese, plate};

    //Test
    Instruction testInstructions[30] = {GO, GRAB, SPECIAL, END};//{GRAB, TURN, PLACE, TURN, GRAB, TURN, PLACE, TURN, GRAB, TURN, PLACE, TURN, GRAB, TURN, PLACE, TURN, GRAB, TURN, PLACE, TURN, GRAB, TURN, PLACE, END};
    double testLocations[2] = {CHEESE, SERVING};//{SERVING, CHEESE};
    Ingredient testIngredients[10] = {cheese};//{bottom_bun, patty, tomato, cheese, lettuce, top_bun};

    Instruction endInstructions[1] = {WAIT};

};
#endif