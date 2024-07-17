#include "Arduino.h"
#include "AS5600.h"
#include "pid.h"
#include "motor.h"
#include "definitions.h"
// #include "arduino-timer.h"
#include "map.h"
//#include "movement.ino"

// #include <WiFi.h>
// #include <ESPmDNS.h>
// #include <WiFiUdp.h>
// #include <ArduinoOTA.h>

const char* ssid = "Kirby ESP32 Testing Server";
const char* password = "KirbySucc";

// WiFiServer server(80);
// WiFiClient RemoteClient;

// auto timer = timer_create_default();

//initialize i2c bus for right encoder
AS5600 as5600_0(&Wire);

//initialize i2c bus for left encoder
AS5600 as5600_1(&Wire1);

const int MAX_SPEED = 4000;
int BASE_SPEED = 2000;
double STEERING_CONSTANT = 0.4 * BASE_SPEED;
double TURNING_CONSTANT = 0.10 * BASE_SPEED;

double dt = PID_LOOP_INTERVAL / 1000.0; //in s
unsigned long timeStart = 0;
unsigned long timeEnd = dt*1000; //convert to ms
unsigned long lastTime = 0;
unsigned long lastTime2 = 0;
double lastAngle_0 = 0;
double lastAngle_1 = 0;

Map m;

volatile double rightSpeedSetpoint = 0; //in degrees/sec
Error rightSpeedError;
Error rightPositionError;
Motor right(PWM_RIGHT_1, PWM_RIGHT_2, MAX_SPEED);
double rightAngularSpeed;

volatile double leftSpeedSetpoint = 0;
Error leftSpeedError;
Error leftPositionError;
Motor left(PWM_LEFT_1, PWM_LEFT_2, MAX_SPEED);
double leftAngularSpeed;

volatile double position = 0;

double GAIN_P = 0;

void setup()
{
  
  Serial.begin(115200);

  pinMode(FR_TCRT, INPUT);
  pinMode(FL_TCRT, INPUT);
  pinMode(BR_TCRT, INPUT);
  pinMode(BL_TCRT, INPUT);

  pinMode(RS_TCRT, INPUT);
  pinMode(LS_TCRT, INPUT);

  pinMode(MICRO_SWITCH_1, INPUT);
  pinMode(MICRO_SWITCH_2, INPUT);

  pinMode(PUMP_SENSE, INPUT);
  pinMode(PUMP, OUTPUT);
  pinMode(VALVE, OUTPUT);

  pinMode(CLAW_SERVO, OUTPUT);

  digitalWrite(PUMP, LOW);

  attachInterrupt(digitalPinToInterrupt(RS_TCRT), updateLocationRight, RISING);
  attachInterrupt(digitalPinToInterrupt(LS_TCRT), updateLocationLeft, RISING);

  //initialize the i2c busses
  Wire.begin(I2C_SDA0, I2C_SCL0);
  Wire1.begin(I2C_SDA1, I2C_SCL1);


  //initialize right encoder
  as5600_0.begin();
  as5600_0.setDirection(AS5600_COUNTERCLOCK_WISE);  //  check encoder/magnet to make sure this is correct
  Serial.print("Connect device 0: ");
  Serial.println(as5600_0.isConnected() ? "true" : "false");
  delay(1000);

  as5600_1.begin();
  as5600_1.setDirection(AS5600_CLOCK_WISE);  //  default, just be explicit.
  Serial.print("Connect device 1: ");
  Serial.println(as5600_1.isConnected() ? "true" : "false");
  delay(1000);

  equalSpeedSet(BASE_SPEED);

  // WiFi.mode(WIFI_STA);
  // WiFi.begin(ssid, password);
  // while (WiFi.waitForConnectResult() != WL_CONNECTED) {
  //   delay(5000);
  //   ESP.restart();
  // }

  // server.begin();
  lastAngle_0 = as5600_0.readAngle() / 4096.0 * 360;
  lastAngle_1 = as5600_1.readAngle() / 4096.0 * 360;
}


void loop() {
  timeStart = millis();
  if (Serial.available() > 0) GAIN_P = Serial.parseFloat();
  if (timeStart - lastTime > PID_LOOP_INTERVAL) {
    rightAngularSpeed = getAngularSpeed(&as5600_0, 0);
    leftAngularSpeed = getAngularSpeed(&as5600_1, 1);
    lineSensingCorrection();
    if (as5600_0.detectMagnet()) rightSpeedError.updateError(rightSpeedSetpoint, 1.0 * rightAngularSpeed, dt);
    if (as5600_1.detectMagnet()) leftSpeedError.updateError(leftSpeedSetpoint, 1.0 * leftAngularSpeed, dt);
    right.setSpeed(rightSpeedSetpoint + GAIN_P*rightSpeedError.p + GAIN_I*rightSpeedError.i + GAIN_D*rightSpeedError.d);
    left.setSpeed(rightSpeedSetpoint + GAIN_P*leftSpeedError.p + GAIN_I*leftSpeedError.i + GAIN_D*leftSpeedError.d);

    
    lastTime = timeStart;
  }
  if (timeStart - lastTime2 > 100) {
    Serial.print("Right_Speed:");
    Serial.print(rightAngularSpeed);
    Serial.print(",");
    Serial.print("Left_Speed:");
    Serial.println(leftAngularSpeed);
    lastTime2 = timeStart;
  }
  // if (server.hasClient()) {
  //   if (RemoteClient.connected()) {
  //     Serial.println("Connection rejected");
  //     server.available().stop();
  //   } else {
  //     Serial.println("Connection accepted");
  //     RemoteClient = server.available();
  //   }
  // }
  
    // timer.tick();
  // if (as5600_0.detectMagnet()) rightSpeedError.updateError(rightSpeedSetpoint, 1.0 * getAngularSpeed(&as5600_0), dt);
  // if (as5600_1.detectMagnet()) leftSpeedError.updateError(leftSpeedSetpoint, 1.0 * getAngularSpeed(&as5600_1), dt);
  // right.setSpeed(rightSpeedSetpoint + GAIN_P*rightSpeedError.p + GAIN_I*rightSpeedError.i + GAIN_D*rightSpeedError.d);
  // left.setSpeed(rightSpeedSetpoint + GAIN_P*leftSpeedError.p + GAIN_I*leftSpeedError.i + GAIN_D*leftSpeedError.d);

  // if (as5600_0.detectMagnet()) rightPositionError.updateError(30.0, as5600_0.readAngle()/4096.0 * 360/* * WHEEL_RADIUS / 360.0 */, dt);
  // right.setSpeed(GAIN_P*rightPositionError.p + GAIN_D*rightPositionError.d);

  // Serial.print("Setpoint: ");
  // Serial.println(rightSpeedSetpoint);
  // Serial.print("Speed (raw): ");
  // Serial.println(getAngularSpeed(&as5600_0));
  // // Serial.print("Error: ");
  // // Serial.println(rightSpeedError.p);
  // Serial.println(timeEnd - timeStart);

  // //Serial.println(as5600_0.detectMagnet());
  // int d = timeEnd - timeStart
  // int d1 = d > dt ? 0 : d;
  // delay(d1*1000);

  // right.setSpeed(rightSpeedSetpoint);
  // left.setSpeed(leftSpeedSetpoint - (leftSpeedSetpoint < 0 ? 650 : 0));

  //turn180(&m, 1);
 // Serial.println(m.TAPE_WIDTH);
  // Serial.println(analogRead(PUMP_SENSE));
  // if (timeStart - lastTime > 2000) {
  //   lastTime = timeStart;
  //   digitalWrite(CLAW_SERVO, !digitalRead(CLAW_SERVO));
  // // ledcWrite(CLAW_SERVO, 25);
  // }
  // digitalWrite()
  // if (RemoteClient.connected()) {
  //   if (RemoteClient.available() > 0) {
  //     BASE_SPEED = RemoteClient.parseFloat();
  //     STEERING_CONSTANT = 0.385 * BASE_SPEED;
  //     TURNING_CONSTANT = 0.10 * BASE_SPEED;
  //     equalSpeedSet(BASE_SPEED);
  //     // delay(5);
  //     // RemoteClient.println(rightSpeedSetpoint);

  //   }

  //   RemoteClient.print("Right Speed: ");
  //   RemoteClient.print(rightAngularSpeed);

  //   RemoteClient.print("  |  Left Speed: ");
  //   RemoteClient.println(leftAngularSpeed);

  //   RemoteClient.println(BASE_SPEED);
  // }
  timeEnd = millis();
  //if (timeEnd - timeStart < dt*1000 - 2) delay(dt*1000 - 2 - (timeEnd - timeStart));
}


void updateEncoderPosition(Map *m, AS5600 *a0, AS5600 *a1) {
  position += m->getMovingDirection() * (getAngularSpeed(a0, 0) + getAngularSpeed(a1, 1))/2 * dt * WHEEL_RADIUS; //dt here is update rate (period)
}


//returns angular speed in degrees/second
//positive for forward rotation, negative for backward rotation
float getAngularSpeed(AS5600 *a, int encoderNum) {

  if (!a->detectMagnet()) return 0;
  //long now = millis();
  //float angleStart = a->readAngle()/4096.0 * 360; //in degrees
  //delayMicroseconds(250);
  float angleEnd = a->readAngle()/4096.0 * 360;
  float deltaA = 0;
  if (encoderNum == 0) {
    deltaA = angleEnd - lastAngle_0;
    lastAngle_0 = angleEnd;
  } else {
    deltaA = angleEnd - lastAngle_1;
    lastAngle_1 = angleEnd;
  }

  if (abs(deltaA) > 180) deltaA -= sign(deltaA) * 360; //fix for when it goes from 0 -> 360 or vice versa
  float speed = deltaA / dt;
  Serial.println(deltaA);

  return speed;
}

int sign(float a) {
  return a >= 0 ? 1 : -1;
}

void lineSensingCorrection() {
  //higher value means on black tape
  double fr = analogRead(FR_TCRT);
  double fl = analogRead(FL_TCRT);
  double br = analogRead(BR_TCRT);
  double bl = analogRead(BL_TCRT);

  int front_correction = (fr - fl);
  int back_correction = (bl - br);

  if (BASE_SPEED != 0) {
    if (m.getDrivingDirection() == 1 && BASE_SPEED > 0) {
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
        leftSpeedSetpoint = BASE_SPEED + STEERING_CONSTANT;
        rightSpeedSetpoint = BASE_SPEED;
      }
      else if (back_correction < 0) {
        rightSpeedSetpoint = BASE_SPEED + STEERING_CONSTANT;
        leftSpeedSetpoint = BASE_SPEED;
      } else {
        leftSpeedSetpoint = BASE_SPEED;
        rightSpeedSetpoint = BASE_SPEED;
      }
    }
  } else {
    rightSpeedSetpoint = 0;
    leftSpeedSetpoint = 0;
  }
}

void equalSpeedSet(double speed) {
  rightSpeedSetpoint = speed;
  leftSpeedSetpoint = speed;
}

void move() {
  right.setSpeed(rightSpeedSetpoint);
  left.setSpeed(leftSpeedSetpoint);
}

void brake() {
  right.setSpeed(0); 
  left.setSpeed(0);
}

//dir: 1 CW, -1 CCW
void turn180(Map *m, int dir) {
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

//these may not be useful at all
void updateLocationRight() {
  if (m.getFacingDirection() == (1 - ROBOT_ID*2)) { // 1 - 0*2 = 1, 1 - 1*2 = -1
    m.location += m.getDrivingDirection();
  }
}

void updateLocationLeft() {
  if (m.getFacingDirection() == (-1 + ROBOT_ID*2)) {
    m.location -= m.getDrivingDirection();
  }
}

/*

switch (m.state) {
  case SPEED:
    //set speed according to distance from desired position
    //interrupt on the destination station tape line: full brake + go to ADJUST

    //attachInterrupt (in interrupt: brake() + m.state = ADJUST)

    lineSensingCorrection();
    right.setSpeed(rightSpeedSetpoint);
    left.setSpeed(leftSpeedSetpoint);

    break;

  case ADJUST:
    //roll backwards slowly until you find edge of tape
    //move precisely one tape-width's distance further
    //go to ARM

    if (!tapeSensed()) {
      equalSetSpeed(SLOW);
    } else {
      final = position +- TAPE_WIDTH / 2.0;
      while (position != final) {
        pid();
      }
      m.state = ARM;
    }

    break;

  case SPIN:

    break;
  case ARM:
    if (isReady) { //isReady to check that the other robot completed their task
      if (hasObject) { 
        place();
        hasObject = false;
      } else {
        grab();
        hasObject = true;
      }
    }

    //update state now? depends on recipe tho, figure that out gl bro
    break;

  case WAIT:

    break;
  default:

    break;
}
*/