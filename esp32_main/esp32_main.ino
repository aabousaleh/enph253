#include "Arduino.h"
#include "AS5600.h"
#include "pid.h"
#include "motor.h"
#include "definitions.h"
#include "arduino-timer.h"

auto timer = timer_create_default();

//initialize i2c bus for right encoder
AS5600 as5600_0(&Wire);

//initialize i2c bus for left encoder
AS5600 as5600_1(&Wire1);

double setpoint = 0; //in cm
double dt = 0.01; //in s
unsigned long timeStart = 0;
unsigned long timeEnd = dt*1000; //convert to ms

volatile double rightSpeedSetpoint = 0; // cm/s
Error right_encoder;
Motor right(PWM_RIGHT_1, PWM_RIGHT_2);

volatile double leftSpeedSetpoint = 0;
Error left_encoder;
Motor left(PWM_LEFT_1, PWM_LEFT_2);

Error line_sensing;

void setup()
{
  
  Serial.begin(115200);

  pinMode(FR_TCRT, INPUT);

  //initialize the i2c busses
  Wire.begin(I2C_SDA0, I2C_SCL0);
  Wire1.begin(I2C_SDA1, I2C_SCL1);

  //initialize right encoder
  as5600_0.begin();
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


void loop() {
  // Serial.print(as5600_0.readAngle());
  // Serial.print("\t");
  // Serial.println(as5600_1.readAngle());

  // // if (Serial.available() > 0) setpoint = Serial.parseFloat(SKIP_ALL);
  // timeStart = millis();
  
  // timer.tick();
  //Serial.println(as5600.detectMagnet());
  Serial.println(analogRead(FR_TCRT));
  delay(50);
}


void line_sensing_method(int speed) {
  //tape is higher value
  double fr = analogRead(FR_TCRT);
  double fl = analogRead(FL_TCRT);
  double br = analogRead(BR_TCRT);
  double bl = analogRead(BL_TCRT);

  int front_correction = (fr - fl);
  int back_correction = (bl - br);

  //TODO add driving direction check. it flips which variable to use in switch case
  
  //if (state.driveDirection == forward) {}

  if (front_correction > 0) {
    rightSpeedSetpoint -= STEERING_CONSTANT;
  }
  else if (front_correction < 0) {
    leftSpeedSetpoint -= STEERING_CONSTANT;
  }

}

void move_forward(int speed, double seconds) {
  right.setSpeed(speed);
  left.setSpeed(speed);
  timer.at(seconds / 1000.0, brake);
}

bool brake(void *) {
  right.setSpeed(0); 
  left.setSpeed(0);
  return true;
}