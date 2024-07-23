#include "map.h"

Map::Map() {
  // location = 0;
  facingDirection = 1;
  drivingDirection = 1;
  locationIndex = 0;
  instructionIndex = 0;
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

// int Map::getLocation() {
//   return location;
// };

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
  return instructions[instructionIndex];
}

double Map::getNextLocation() {
  return locations[locationIndex];
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