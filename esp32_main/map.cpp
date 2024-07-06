#include "map.h"
#include "definitions.h"

//all the following are in inches
const double TAPE_WIDTH = 0.75;

//+- 0.25 on the rest
const double FIELD_LENGTH = 96;
const double TOMATOES = 6;
const double CUTTING = 29.75;
const double COOKTOP = 72.25;
const double PLATES = 90;

//for robot id 0:
const double PATTIES = 12.25;
const double BUNS = 48;
const double POTATOES = 84;

//for robot id 1:
const double CHEESE = 6;
const double SERVING_AREA = 48;
const double LETTUCE = 90;

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

//TODO: consider making drivingDirection dependent on moving direction (we know where we want to go based on position, we send driving direction instruction based on that and facing direction: D = M/F)
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