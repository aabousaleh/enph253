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
    void flipFacingDirection(); //flips facing direction
    State getState(); //returns current state of what the robot is trying to do
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
    Recipe recipes1[10] = {CHEESE_PLATE, CHEESE_PLATE, CHEESE_PLATE, CHEESE_PLATE, FLING, END_RECIPE};
    Recipe recipes2[5] = {DELUXE_BURGER, DELUXE_BURGER, DELUXE_BURGER, DELUXE_BURGER, END_RECIPE};

    //FLING
    Instruction flingInstructions[8] = {GO, GRAB, SPECIAL, END};
    double flingLocations[2] = {CHEESE, SERVING};
    Ingredient flingIngredients[1] = {cheese};

    //Deluxe cheese burger, robot 1
    Instruction deluxeBurgerInstructions[30] = {GO, GRAB, TURN, GO, RECEIVE_CHECKPOINT, PLACE, TURN, GRAB, TURN, PLACE, GO, GRAB, GO, PLACE, SEND_CHECKPOINT, RECEIVE_CHECKPOINT, GRAB, TURN, GO, END};
    double deluxeBurgerLocations[7] = {CHEESE, PLATES, COOKTOP, PLATES, SERVING};
    Ingredient deluxeBurgerIngredients[5] = {cheese, lettuce, patty, plate};

    //burger robot 1
    Instruction burgerInstructions[20] = {GO, RECEIVE_CHECKPOINT, WAIT, GRAB, GO, RECEIVE_CHECKPOINT, PLACE, RECEIVE_CHECKPOINT, GRAB, TURN, GO, PLACE};
    double burgerLocations[3] = {COOKTOP, PLATES, SERVING};
    Ingredient burgerIngredients[2] = {patty, plate};

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
    double cheeseLocations[5] = {CHEESE, PLATES, SERVING};
    Ingredient cheeseIngredients[2] = {cheese, plate};

    //Test
    Instruction testInstructions[30] = {GO, GO, END};//{GRAB, TURN, PLACE, TURN, GRAB, TURN, PLACE, TURN, GRAB, TURN, PLACE, TURN, GRAB, TURN, PLACE, TURN, GRAB, TURN, PLACE, TURN, GRAB, TURN, PLACE, END};
    double testLocations[2] = {PLATES, COOKTOP};//{SERVING, CHEESE};
    Ingredient testIngredients[10] = {cheese};//{bottom_bun, patty, tomato, cheese, lettuce, top_bun};

    Instruction endInstructions[1] = {END_HEAT};

};
#endif