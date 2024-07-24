#include "vacuum.h"
#include "analogRead.h"

Vacuum::Vacuum(int _ctrlPin, int _sensPin, int _valvePin) {
  ctrlPin = _ctrlPin;
  sensPin = _sensPin;
  valvePin = _valvePin;

  pinMode(ctrlPin, OUTPUT);
  pinMode(valvePin, OUTPUT);

  digitalWrite(ctrlPin, LOW);
  digitalWrite(valvePin, LOW);
}

void Vacuum::setSucc(bool succ) {
  if (succ) {
    digitalWrite(ctrlPin, HIGH);
  } else { //turns on solenoid, turns off pump
    digitalWrite(valvePin, HIGH);
    digitalWrite(ctrlPin, LOW);
    delay(100); //TODO: MIGHT NEED TO CHANGE THIS DELAY VALUE
    digitalWrite(valvePin, LOW);
  }
}

bool Vacuum::objSecured() {
  bool pickedUp = false;

  //TODO: NEED TO BE CHANGED EXPERIMENTALLY
  int LOWER_THRESHOLD = 400;
  int HIGHER_THRESHOLD = 600;

  //TODO: MIGHT NEED TO CHANGE THIS
  const int NUM_READINGS = 50;
  int currentReadings[NUM_READINGS];
  int index = 0;
  int total = 0;
  int avg = 0;

  //intializes all values in the array to 0
  for (int i = 0; i < NUM_READINGS; i++) {
    currentReadings[i] = 0;
  }
  
  const uint32_t DESIRED_PERIOD = 1000;
  uint32_t tStart = millis();
  uint32_t tNow = millis();
  bool startCheck = false;

  while (tNow - tStart <= DESIRED_PERIOD) {
    total -= currentReadings[index];
    currentReadings[index] = analogRead(sensPin);
    total += currentReadings[index];

    if (index < NUM_READINGS) i++;
    else {
      i = 0;
      startCheck = true;
    }

    avg = total / NUM_READINGS;

    if (startCheck) {
      if (avg < LOWER_THRESHOLD || avg > HIGHER_THRESHOLD) pickedUp = true;
    }

    tNow = millis();
  }

  return pickedUp;
}
