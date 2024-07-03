#include "map.h"

Map::Map(/*int numStations, String _stations[numStations]*/) {
  /*stations = _stations;*/
  location = 0;
  facingDirection = 1;
  drivingDirection = 1;
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

/*String getLocation() {
  return stations[location];
}*/

int Map::getLocation() {
  return location;
};

void Map::flipFacingDirection() {
  facingDirection *= -1;
};

void Map::flipDrivingDirection() {
  drivingDirection *= -1;
};

void IRAM_ATTR Map::updateLocationRight() {
  if (facingDirection == 1) {
    location += drivingDirection;
  }
};

void IRAM_ATTR Map::updateLocationLeft() {
  if (facingDirection == -1) {
    location -= drivingDirection;
  }
};