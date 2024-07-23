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
  int currentReading = analogRead(sensPin);

  if (currentReading < LOWER_THRESHOLD || currentReading > HIGHER_THRESHOLD) pickedUp = true;

  return pickedUp;
}
