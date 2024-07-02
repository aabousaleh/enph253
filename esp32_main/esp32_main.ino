//
//    FILE: AS5600_demo.ino
//  AUTHOR: Rob Tillaart
// PURPOSE: demo
//     URL: https://github.com/RobTillaart/AS5600
//
//  Examples may use AS5600 or AS5600L devices.
//  Check if your sensor matches the one used in the example.
//  Optionally adjust the code.


#include "AS5600.h"
#include "pid.h"
#include "motor.h"

//right motor pwm pins
#define PWM_RIGHT_1 26
#define PWM_RIGHT_2 32

//left motor pwm pins
#define PWM_LEFT_1 39
#define PWM_LEFT_2 25

//right encoder i2c
#define I2C_SDA0 21
#define I2C_SCL0 22

//left encoder i2c
#define I2C_SDA1 19
#define I2C_SCL1 8

#define GAIN_0P 1
#define GAIN_0I 0
#define GAIN_0D 2

AS5600 as5600_0(&Wire);
AS5600 as5600_1(&Wire1);

double setpoint = 0; //in cm
double dt = 0.01; //in s
unsigned long timeStart = 0;
unsigned long timeEnd = dt*1000; //convert to ms

Error right_encoder;
Motor right(PWM_RIGHT_1, PWM_RIGHT_2);

Error left_encoder;
Motor left(PWM_LEFT_1, PWM_LEFT_2);

Error line_sensing;

void setup()
{
  
  Serial.begin(115200);

  // pinMode(PWM_RIGHT_1, OUTPUT);
  // pinMode(PWM_RIGHT_2, OUTPUT);

  // analogWrite(PWM_RIGHT_1, 0);
  // analogWrite(PWM_RIGHT_2, 0);

  Wire.begin(I2C_SDA0, I2C_SCL0);
  Wire1.begin(I2C_SDA1, I2C_SCL1);

  as5600_0.begin();  //  set direction pin.
  as5600_0.setDirection(AS5600_CLOCK_WISE);  //  default, just be explicit.
  Serial.print("Connect device 0: ");
  Serial.println(as5600_0.isConnected() ? "true" : "false");
  delay(1000);

  as5600_1.begin();  //  set direction pin.
  as5600_1.setDirection(AS5600_CLOCK_WISE);  //  default, just be explicit.
  Serial.print("Connect device 1: ");
  Serial.println(as5600_1.isConnected() ? "true" : "false");
  delay(1000);
}


void loop()
{
  //  Serial.print(millis());
  //  Serial.print("\t");
  Serial.print(as5600_0.readAngle());
  Serial.print("\t");
  Serial.println(as5600_1.readAngle());
  //  Serial.println(as5600.rawAngle() * AS5600_RAW_TO_DEGREES);

  // if (Serial.available() > 0) setpoint = Serial.parseFloat(SKIP_ALL);
  timeStart = millis();
  
  delay(100);
  //Serial.println(as5600.detectMagnet());
}