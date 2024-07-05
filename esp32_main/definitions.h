#ifndef DEFINITIONS_H
#define DEFINITIONS_H

const int ROBOT_ID = 0; //CHANGE FOR EACH ROBOT. 0 is scooper robot, 1 is vacuum robot

//right motor pwm pins
#define PWM_RIGHT_1 26
#define PWM_RIGHT_2 32

//left motor pwm pins
#define PWM_LEFT_1 20 //39  //temporary for testing
#define PWM_LEFT_2 25

//right encoder i2c
#define I2C_SDA0 21
#define I2C_SCL0 22

//left encoder i2c
#define I2C_SDA1 13
#define I2C_SCL1 15

//testing i2c pins
//#define I2C_SDA2 19
//#define I2C_SCL2 8

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

// steering constant for corrections in cm/s

//wheel radius in cm
#define WHEEL_RADIUS 3.35

#define GAIN_P 0.15675
#define GAIN_I 0.005
#define GAIN_D 0

#endif