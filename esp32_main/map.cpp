#include "map.h"

Map::Map() {
  // location = 0;
  facingDirection = 1;
  drivingDirection = 1;
  locationIndex = 0;
  instructionIndex = 0;
  recipeIndex = 0;
  nextRecipe();
};

int Map::getFacingDirection() {
  return facingDirection;
};

int Map::getDrivingDirection() {
  return drivingDirection;
};

//TODO: consider making drivingDirection dependent on moving direction (we know where we want to go based on position, we send driving direction instruction based on that and facing direction: D = M/F)
int Map::getMovingDirection() {
  return facingDirection * drivingDirection;
};

State Map::getState() {
  return state;
};

void Map::flipFacingDirection() {
  facingDirection *= -1;
};

void Map::flipDrivingDirection() {
  drivingDirection *= -1;
};

Instruction Map::getNextInstruction() {
  return currentRecipeInstructions[instructionIndex++];
}

double Map::getNextLocation() {
  return currentRecipeLocations[locationIndex++];
}

void Map::nextRecipe() {
  Food nextRecipe = recipes[recipeIndex++];
  instructionIndex = 0;
  locationIndex = 0;
  switch (nextRecipe) {
    case BURGER: {
      currentRecipeInstructions = burgerSansGrab;
      currentRecipeLocations = burgerLocations;
      break;
    }
    case TEST: {
      currentRecipeInstructions = testInstructions;
      currentRecipeLocations = testLocations;
      break;
    }
    default: {

      break;
    }

  }
}

// void IRAM_ATTR Map::updateLocationRight() {
//   if (facingDirection == (1 - ROBOT_ID*2)) { // 1 - 0*2 = 1, 1 - 1*2 = -1
//     location += drivingDirection;
//   }
// };

// void IRAM_ATTR Map::updateLocationLeft() {
//   if (facingDirection == (-1 + ROBOT_ID*2)) {
//     location -= drivingDirection;
//   }
// };