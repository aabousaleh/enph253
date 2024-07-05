#include "map.h"
#include "definitions.h"

Map::Map() {
  location = 0;
  facingDirection = 1;
  drivingDirection = 1;
  stateIndex = 0;
};

int Map::getFacingDirection() {
  return facingDirection;
};

int Map::getDrivingDirection() {
  return drivingDirection;
};

int Map::getMovingDirection() {
  return facingDirection * drivingDirection;
};

int Map::getLocation() {
  return location;
};

String Map::getState() {
  return states[stateIndex];
};

void Map::flipFacingDirection() {
  facingDirection *= -1;
};

void Map::flipDrivingDirection() {
  drivingDirection *= -1;
};

void IRAM_ATTR Map::updateLocationRight() {
  if (facingDirection == (1 - ROBOT_ID*2)) { // 1 - 0*2 = 1, 1 - 1*2 = -1
    location += drivingDirection;
  }
};

void IRAM_ATTR Map::updateLocationLeft() {
  if (facingDirection == (-1 + ROBOT_ID*2)) {
    location -= drivingDirection;
  }
};