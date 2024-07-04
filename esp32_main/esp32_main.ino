#include "Arduino.h"
#include "AS5600.h"
#include "pid.h"
#include "motor.h"
#include "definitions.h"
#include "arduino-timer.h"
#include "map.h"
#include "movement.ino"

auto timer = timer_create_default();

//initialize i2c bus for right encoder
AS5600 as5600_0(&Wire);

//initialize i2c bus for left encoder
AS5600 as5600_1(&Wire1);

double setpoint = 0; //in cm
double dt = 0.01; //in s
unsigned long timeStart = 0;
unsigned long timeEnd = dt*1000; //convert to ms

float lastAngle = 0; //for getAngularSpeed

Map m;

volatile double rightSpeedSetpoint = 0; // cm/s
Error rightSpeedError;
Error rightPositionError;
Motor right(PWM_RIGHT_1, PWM_RIGHT_2, 3300); //3300 is totally random value that is """"SOMEWHAT"""" close to real. adjust as needed

volatile double leftSpeedSetpoint = 0;
Error leftSpeedError;
Error leftPositionError;
Motor left(PWM_LEFT_1, PWM_LEFT_2, 3300);

void setup()
{
  
  Serial.begin(115200);

  pinMode(FR_TCRT, INPUT);
  pinMode(FL_TCRT, INPUT);
  pinMode(BR_TCRT, INPUT);
  pinMode(BL_TCRT, INPUT);

  pinMode(RS_TCRT, INPUT);
  pinMode(LS_TCRT, INPUT);

  attachInterrupt(RS_TCRT, m.updateLocationRight, RISING);
  attachInterrupt(LS_TCRT, m.updateLocationLeft, RISING);

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

  equalSpeedSet(1500);
}


void loop() {
  // Serial.print(as5600_0.readAngle());
  // Serial.print("\t");
  // Serial.println(as5600_1.readAngle());

  if (Serial.available() > 0) rightSpeedSetpoint = Serial.parseFloat();
  // timeStart = millis();
  
    // timer.tick();
    //Serial.println(as5600.detectMagnet());
    //Serial.println(analogRead(FR_TCRT));
    //line_sensing_correction();
  // if (as5600_0.detectMagnet()) rightSpeedError.updateError(rightSpeedSetpoint, 1.0 * getAngularSpeed(&as5600_0) /* * WHEEL_RADIUS / 360.0 */, dt);
  // if (as5600_1.detectMagnet()) leftSpeedError.updateError(leftSpeedSetpoint, 1.0 * getAngularSpeed(&as5600_1) * WHEEL_RADIUS / 360.0, dt);
  // right.setSpeed(rightSpeedSetpoint + GAIN_P*rightSpeedError.p + GAIN_I*rightSpeedError.i + GAIN_D*rightSpeedError.d);
  // left.setSpeed(rightSpeedSetpoint + GAIN_P*leftSpeedError.p + GAIN_I*leftSpeedError.i + GAIN_D*leftSpeedError.d);

  if (as5600_0.detectMagnet()) rightPositionError.updateError(30.0, as5600_0.readAngle()/4096.0 * 360/* * WHEEL_RADIUS / 360.0 */, dt);
  right.setSpeed(GAIN_P*rightPositionError.p + GAIN_D*rightPositionError.d);

  Serial.print("Setpoint: ");
  Serial.println(rightSpeedSetpoint);
  Serial.print("Speed (raw): ");
  Serial.println(getAngularSpeed(&as5600_0));
  Serial.print("Error: ");
  Serial.println(rightSpeedError.p);
  //Serial.println("\n");
  // Serial.println(as5600_0.detectMagnet());
  delay(10);
}


void updateEncoderPosition(Map *m) {
  
}


//returns angular speed in degrees/second
//positive for forward rotation, negative for backward rotation
float getAngularSpeed(AS5600 *a) {

  if (!a->detectMagnet()) return 0;
  //long now = millis();
  float angle = a->readAngle()/4096.0 * 360; //in degrees

  //long deltaT = now - lastTime;
  float deltaA = angle - lastAngle;

  lastAngle = angle;
  
  float speed = deltaA / dt;

  return speed;
}

void line_sensing_correction() {
  //tape is higher value
  double fr = analogRead(FR_TCRT);
  double fl = analogRead(FL_TCRT);
  double br = analogRead(BR_TCRT);
  double bl = analogRead(BL_TCRT);

  int front_correction = (fr - fl);
  int back_correction = (br - bl);

  if (m.getDrivingDirection() == 1) {
    if (front_correction > 0) {
      rightSpeedSetpoint -= STEERING_CONSTANT;
    }
    else if (front_correction < 0) {
      leftSpeedSetpoint -= STEERING_CONSTANT;
    }
  } else {
    if (back_correction > 0) {
      rightSpeedSetpoint -= STEERING_CONSTANT;
    }
    else if (back_correction < 0) {
      leftSpeedSetpoint -= STEERING_CONSTANT;
    }
  }
}

void equalSpeedSet(double speed) {
  rightSpeedSetpoint = speed;
  leftSpeedSetpoint = speed;
}

void move() {
  right.setSpeed(rightSpeedSetpoint);
  left.setSpeed(leftSpeedSetpoint);
  //timer.at(seconds / 1000.0, brake);
}

bool brake(void *) {
  right.setSpeed(0); 
  left.setSpeed(0);
  return true;
}

void turn180(Map *m) {
  //TODO write this function
  m->flipFacingDirection();
}