#ifndef DEFINITIONS_H
#define DEFINITIONS_H

const int ROBOT_ID = 1; //CHANGE FOR EACH ROBOT. 0 is scooper robot, 1 is vacuum robot

//all the following are in inches
const double TAPE_WIDTH = 0.75;

//+- 0.25 on the rest
const double FIELD_LENGTH = 96;
const double TOMATOES = 6;
const double CUTTING = 29.75;
const double COOKTOP = 72.5;
const double PLATES = 89.75;

//for robot id 0:
const double PATTIES = 12;
const double BUNS = 48;
const double POTATOES = 84;

//for robot id 1:
const double CHEESE = 6;
const double SERVING = 48;
const double LETTUCE = 90;

enum State {
  MOVE,
  ADJUST,
  SPIN,
  ARM,
  WAITING
};

enum Instruction {
  GO,
  GRAB,
  PLACE,
  TURN,
  WAIT,
  END,
  SEND_CHECKPOINT,
  RECEIVE_CHECKPOINT
};

enum Recipe {
  BURGER,
  DELUX_BURGER,
  SALAD,
  CHEESE_PLATE,
  TEST,
  END_RECIPE
};

enum Ingredient {
  patty,
  bun,
  potato,
  tomato,
  cheese,
  lettuce,
  plate
};

#define LOOP_INTERVAL 10 //in millis

//right motor pwm pins
#define PWM_RIGHT_1 26
#define PWM_RIGHT_2 32

//left motor pwm pins
#define PWM_LEFT_1 4
#define PWM_LEFT_2 25

//right encoder i2c
#define I2C_SDA0 21
#define I2C_SCL0 22

//left encoder i2c
#define I2C_SDA1 13
#define I2C_SCL1 15

//front right tcrt
#define FR_TCRT 35
//front left tcrt
#define FL_TCRT 34
//back right tcrt
#define BR_TCRT 38
//back left tcrt
#define BL_TCRT 37

//right station-sensing tcrt
#define RS_TCRT 5
//left station-sensing tcrt
#define LS_TCRT 7

#define PUMP 20
#define VALVE 27
#define PUMP_SENSE 33 //current sense 2
//#define VALVE_SENSE 33 //current sense 1

#define CLAW_SERVO 2
#define SHOULDER_SERVO 10
#define ELBOW_SERVO 9

#define MICRO_SWITCH_1 19
#define MICRO_SWITCH_2 8

//wheel radius in inch
#define WHEEL_RADIUS 1.285

#endif