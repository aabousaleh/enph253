/**
 * This library converts from cartesian cordinates of the arm position into pwm outputs for the servos.
 * NOTE:    In this state it is possible to input invalid values. For example it is possible to input a value for the servo where
 *          the arm mechisms may self intersect.
 */
#ifndef ARM_H
#define ARM_H

#include <Arduino.h>
#include "definitions.h"

#define POLYNOMIAL_DEGREE 5 //Don't change unless your name is Michael
#define SERVO_PWM_RESOLUTION 16 //PWM resolution, should be decently high since servos only use 2.5%-12.5% duty cycle
#define SERVO_PWM_FREQUENCY 50 //Servo PWM frequency, default is 50Hz

#define SHOULDER_SERVO_CALIBRATION_POINTS (float[2][2]){{PI, 0.08}, {3*PI/2,0.032}} //Calibaration points in the form of (radians, duty)
#define ELBOW_SERVO_CALIBRATION_POINTS (float[2][2]){{PI, 0.101},{PI/2, 0.059}}

#define MAX_PWM 0.125
#define MIN_PWM 0.025

//setup the servo pins
void setupArmServos();

/**
 * Move the arm to a cartesian cordinate. The origin is at the pivot point of the shoulder. 
 * 
 * Param: a valid cordinate. An expected value might be (35,11)
 * Return: true if the arm is able to move there. False otherwise
 */

bool moveToXY(float x, float y);

/**
 * Send a raw duty cycle value to the servos.
 * 
 * Param: a valid PWM duty cycle.
 * Return: False if the input duty cycle is out of range.
 */
bool moveRaw(float shoulderDutyCycle, float elbowDutyCycle);

#endif