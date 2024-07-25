/**
 * This library converts from regular PWM duty cycles to calibrated PWM duty cycles.
 */
#ifndef CALIBRATION_H
#define CALIBRATION_H

#include "definitions.h"
#include <Arduino.h>

#define PWM_FREQ 250
#define PWM_RESOLUTION 12

#define MOTOR_LEFT 0
#define MOTOR_RIGHT 1

#define NUMBER_OF_CALIBRATION_POINTS 41

void setupMotors();

bool sendPWM(float dutyCycle, char channel);

#endif