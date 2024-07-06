#include "Arduino.h"
#include "AS5600.h"
#include "pid.h"
#include "motor.h"
#include "definitions.h"
#include "arduino-timer.h"
#include "map.h"
#include "movement.ino"

// auto timer = timer_create_default();

//initialize i2c bus for right encoder
AS5600 as5600_0(&Wire);

//initialize i2c bus for left encoder
AS5600 as5600_1(&Wire1);

const int MAX_SPEED = 4000;
const int BASE_SPEED = 2500;
const double STEERING_CONSTANT = 0.35 * BASE_SPEED;
const double TURNING_CONSTANT = 0.10 * BASE_SPEED;

double setpoint = 0; //in cm
double dt = 0.01; //in s
unsigned long timeStart = 0;
unsigned long timeEnd = dt*1000; //convert to ms

float lastAngle = 0; //for getAngularSpeed

Map m;

volatile double rightSpeedSetpoint = 0; // cm/s
Error rightSpeedError;
Error rightPositionError;
Motor right(PWM_RIGHT_1, PWM_RIGHT_2, MAX_SPEED); //3300 is totally random value that is """"SOMEWHAT"""" close to real. adjust as needed

volatile double leftSpeedSetpoint = 0;
Error leftSpeedError;
Error leftPositionError;
Motor left(PWM_LEFT_1, PWM_LEFT_2, MAX_SPEED);

volatile double position = 0;

long int lastTime = 0;

void setup()
{
  
  Serial.begin(115200);

  pinMode(FR_TCRT, INPUT);
  pinMode(FL_TCRT, INPUT);
  pinMode(BR_TCRT, INPUT);
  pinMode(BL_TCRT, INPUT);

  pinMode(RS_TCRT, INPUT);
  pinMode(LS_TCRT, INPUT);

  pinMode(PUMP_SENSE, INPUT);
  pinMode(PUMP, OUTPUT);
  pinMode(VALVE, OUTPUT);

  digitalWrite(PUMP, LOW);

  // attachInterrupt(RS_TCRT, m.updateLocationRight, RISING); broken
  // attachInterrupt(LS_TCRT, m.updateLocationLeft, RISING);

  //initialize the i2c busses
  Wire.begin(I2C_SDA0, I2C_SCL0);
  Wire1.begin(I2C_SDA1, I2C_SCL1);

  //initialize right encoder
  as5600_0.begin();
  as5600_0.setDirection(AS5600_CLOCK_WISE);  //  default, just be explicit.
  Serial.print("Connect device 0: ");
  Serial.println(as5600_0.isConnected() ? "true" : "false");
  delay(1000);

  as5600_1.begin();
  as5600_1.setDirection(AS5600_CLOCK_WISE);  //  default, just be explicit.
  Serial.print("Connect device 1: ");
  Serial.println(as5600_1.isConnected() ? "true" : "false");
  delay(1000);

  equalSpeedSet(BASE_SPEED);
}


void loop() {
  // Serial.print(as5600_0.readAngle());
  // Serial.print("\t");
  // Serial.println(as5600_1.readAngle());

  //if (Serial.available() > 0) rightSpeedSetpoint = Serial.parseFloat();
  timeStart = millis();
  
    // timer.tick();
    //Serial.println(as5600.detectMagnet());
    //Serial.println(analogRead(FR_TCRT));
    //line_sensing_correction();
  //if (as5600_0.detectMagnet()) rightSpeedError.updateError(rightSpeedSetpoint, 1.0 * getAngularSpeed(&as5600_0) /* * WHEEL_RADIUS / 360.0 */, dt);
  // if (as5600_1.detectMagnet()) leftSpeedError.updateError(leftSpeedSetpoint, 1.0 * getAngularSpeed(&as5600_1) * WHEEL_RADIUS / 360.0, dt);
  //right.setSpeed(rightSpeedSetpoint + GAIN_P*rightSpeedError.p + GAIN_I*rightSpeedError.i + GAIN_D*rightSpeedError.d);
  // left.setSpeed(rightSpeedSetpoint + GAIN_P*leftSpeedError.p + GAIN_I*leftSpeedError.i + GAIN_D*leftSpeedError.d);

  // if (as5600_0.detectMagnet()) rightPositionError.updateError(30.0, as5600_0.readAngle()/4096.0 * 360/* * WHEEL_RADIUS / 360.0 */, dt);
  // right.setSpeed(GAIN_P*rightPositionError.p + GAIN_D*rightPositionError.d);

  // Serial.print("Setpoint: ");
  // Serial.println(rightSpeedSetpoint);
  // Serial.print("Speed (raw): ");
  // Serial.println(getAngularSpeed(&as5600_0));
  // // Serial.print("Error: ");
  // // Serial.println(rightSpeedError.p);
  // timeEnd = millis();
  // Serial.println(timeEnd - timeStart);

  // //Serial.println(as5600_0.detectMagnet());
  // int d = timeEnd - timeStart
  // int d1 = d > dt ? 0 : d;
  // delay(d1*1000);
      // line_sensing_correction();
      // right.setSpeed(rightSpeedSetpoint);
      // left.setSpeed(leftSpeedSetpoint);

  //turn180(&m, 1);
 // Serial.println(m.TAPE_WIDTH);
  Serial.println(analogRead(PUMP_SENSE));
  // if (timeStart - lastTime > 2000) {
    // lastTime = timeStart;
    digitalWrite(PUMP, HIGH);
  // }
  delay(50);
}


void updateEncoderPosition(Map *m, AS5600 *a1, AS5600 *a2) {
  position += m->getMovingDirection() * (getAngularSpeed(a1) + getAngularSpeed(a2))/2 * dt;
}


//returns angular speed in degrees/second
//positive for forward rotation, negative for backward rotation
//TODO: this method is scuffed af, results in huge negative speed when rotation resets. FIX ASAP
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
  //higher value means on black tape
  double fr = analogRead(FR_TCRT);
  double fl = analogRead(FL_TCRT);
  double br = analogRead(BR_TCRT);
  double bl = analogRead(BL_TCRT);

  int front_correction = (fr - fl);
  int back_correction = (br - bl);

  if (m.getDrivingDirection() == 1) {
    if (front_correction > 0) {
      rightSpeedSetpoint = BASE_SPEED - STEERING_CONSTANT;
      leftSpeedSetpoint = BASE_SPEED;
    }
    else if (front_correction < 0) {
      leftSpeedSetpoint =  BASE_SPEED - STEERING_CONSTANT;
      rightSpeedSetpoint = BASE_SPEED;
    } else {
      leftSpeedSetpoint = BASE_SPEED;
      rightSpeedSetpoint = BASE_SPEED;
    }
  } else {
    if (back_correction > 0) {
      leftSpeedSetpoint = BASE_SPEED - STEERING_CONSTANT;
      rightSpeedSetpoint = BASE_SPEED;
    }
    else if (back_correction < 0) {
      rightSpeedSetpoint = BASE_SPEED - STEERING_CONSTANT;
      leftSpeedSetpoint = BASE_SPEED;
    } else {
      leftSpeedSetpoint = BASE_SPEED;
      rightSpeedSetpoint = BASE_SPEED;
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

void brake() {
  right.setSpeed(0); 
  left.setSpeed(0);
}

//dir: 1 CW, -1 CCW
void turn180(Map *m, int dir) {
  //TODO write this function
  if (dir == 1) {
    right.setSpeed(TURNING_CONSTANT);
    left.setSpeed(-TURNING_CONSTANT);
    delay(1000); //change
    brake();
    delay(1000);
  } else if (dir == -1) {
    right.setSpeed(-TURNING_CONSTANT);
    left.setSpeed(TURNING_CONSTANT);
    delay(1000);
    brake();
    delay(1000);
  }
  m->flipFacingDirection();
}