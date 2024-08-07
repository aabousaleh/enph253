#include "map.h"

Map::Map() {
  // location = 0;
  facingDirection = -1;
  drivingDirection = 1;
  movingDirection = 1;
  locationIndex = 0;
  instructionIndex = 0;
  ingredientIndex =
  recipeIndex = 0;
  nextRecipe();
};

int Map::getFacingDirection() {
  return facingDirection;
};

int Map::getDrivingDirection() {
  return facingDirection * movingDirection;
};

//TODO: consider making drivingDirection dependent on moving direction (we know where we want to go based on position, we send driving direction instruction based on that and facing direction: D = M/F)
int Map::getMovingDirection() {
  return movingDirection;
};

void Map::setMovingDirection(int dir) {
  movingDirection = dir;
}

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

Ingredient Map::getNextIngredient() {
  return currentRecipeIngredients[ingredientIndex++];
}

void Map::nextRecipe() {
  Recipe nextRecipe = recipes[recipeIndex++];
  instructionIndex = 0;
  locationIndex = 0;
  ingredientIndex = 0;
  switch (nextRecipe) {
    case BURGER: {
      currentRecipeInstructions = burgerInstructions;
      currentRecipeLocations = burgerLocations;
      currentRecipeIngredients = burgerIngredients;
      break;
    }
    case TEST: {
      currentRecipeInstructions = testInstructions;
      currentRecipeLocations = testLocations;
      currentRecipeIngredients = testIngredients;
      break;
    }
    case CHEESE_PLATE: {
      currentRecipeInstructions = cheeseInstructions;
      currentRecipeLocations = cheeseLocations;
      currentRecipeIngredients = cheeseIngredients;
      break;
    }
    case SALAD: {
      currentRecipeInstructions = saladInstructions;
      currentRecipeLocations = saladLocations;
      currentRecipeIngredients = saladIngredients;
    }
    case FLING: {
      currentRecipeInstructions = flingInstructions;
      currentRecipeLocations = flingLocations;
      currentRecipeIngredients = flingIngredients;
    }
    default: {
      currentRecipeInstructions = endInstructions;
      // currentRecipeLocations = cheeseLocations;
      // currentRecipeIngredients = cheeseIngredients;
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