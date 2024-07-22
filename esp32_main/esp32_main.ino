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

const int MAX_SPEED = 2200;
int BASE_SPEED = 600;
double STEERING_CONSTANT = 0.065 * MAX_SPEED;
double TURNING_CONSTANT = 0.1 * MAX_SPEED;

double dt = PID_LOOP_INTERVAL / 1000.0; //in s
unsigned long timeStart = 0;
unsigned long timeEnd = dt*1000; //convert to ms
unsigned long lastTime = 0;
unsigned long lastTime2 = 0;
double lastAngle_0 = 0;
double lastAngle_1 = 0;

Map m;

bool BRAKE_OFF = true; //turns off motors when false

volatile double rightSpeedSetpoint = 0; //in degrees/sec
Error rightSpeedError(MAX_SPEED*2);
Error rightPositionError(4096);
Motor right(PWM_RIGHT_1, PWM_RIGHT_2, MAX_SPEED);
double rightAngularSpeed;

volatile double leftSpeedSetpoint = 0;
Error leftSpeedError(MAX_SPEED*2);
Error leftPositionError(4096);
Motor left(PWM_LEFT_1, PWM_LEFT_2, MAX_SPEED);
double leftAngularSpeed;

volatile double position = 5.75;

double GAIN_P = 0.45;
double GAIN_I = 0.35;
double GAIN_D = 0.001;

int LAST_TURN = 0;


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

bool startTimer = true;
unsigned long timerStart1 = 0;
bool endTimer = true;

double intendedPosition = COOKTOP;

void loop() {
  timeStart = millis();
  if (Serial.available() > 0){
    // char c = Serial.read();
    // switch (c) {
    //   case 'P':

    // }
    // GAIN_P = Serial.parseFloat();
    // Read the incoming byte (assuming the data format is 'P I D')
    String input = Serial.readStringUntil('\n');
    // Split the input string into three parts (P, I, D)
    int spaceIndex1 = input.indexOf(' ');
    int spaceIndex2 = input.lastIndexOf(' ');
    if (spaceIndex1 != -1 && spaceIndex2 != -1 && spaceIndex1 != spaceIndex2) {
      // Extract P, I, D from the input string
      String strP = input.substring(0, spaceIndex1);
      String strI = input.substring(spaceIndex1 + 1, spaceIndex2);
      String strD = input.substring(spaceIndex2 + 1);
      
      // Convert strings to doubles
      GAIN_P = strP.toDouble();
      GAIN_I = strI.toDouble();
      GAIN_D = strD.toDouble();
    }
    else {
      // If the input format is incorrect
      Serial.println("Invalid input format. Please enter 'P I D'.");
    }
  }
  if (timeStart - lastTime > PID_LOOP_INTERVAL) {
    if (abs(intendedPosition - position) > 0.5) {
      BASE_SPEED = 600 * sign(intendedPosition - position);
      equalSpeedSet(BASE_SPEED);
    } else {
      if (intendedPosition == PATTIES) {
        brake();
        BRAKE_OFF = false;
      } else {
        intendedPosition = PATTIES;
        brake();
        delay(1000);
        BASE_SPEED = 600 * sign(intendedPosition - position);
      equalSpeedSet(BASE_SPEED);
      }
    }
    dt = (timeStart - lastTime)/1000.0;
    rightAngularSpeed = getAngularSpeed(&as5600_0, 0);
    leftAngularSpeed = getAngularSpeed(&as5600_1, 1);
    right.updateSpeeds(rightAngularSpeed);
    left.updateSpeeds(leftAngularSpeed);
    double rightAverageSpeed = right.averageSpeed();
    double leftAverageSpeed = left.averageSpeed();

    // if (as5600_0.detectMagnet()) rightSpeedError.updateError(rightSpeedSetpoint, rightAverageSpeed, dt);
    // if (as5600_1.detectMagnet()) leftSpeedError.updateError(leftSpeedSetpoint, leftAverageSpeed, dt);
    lineSensingCorrection();
    right.setSpeed(rightSpeedSetpoint * BRAKE_OFF);
    //double oglss = leftSpeedSetpoint;
    //if (leftSpeedSetpoint < 0) leftSpeedSetpoint -= 200;
    left.setSpeed((leftSpeedSetpoint) * BRAKE_OFF);
    updateEncoderPosition();
    //leftSpeedSetpoint = oglss;
    //right.setSpeed(MAX_SPEED);
    //right.setSpeed(BASE_SPEED);
    //right.setSpeed(rightSpeedSetpoint + GAIN_P*rightSpeedError.p + GAIN_I*rightSpeedError.i);
    // Serial.print("Setpoint:");
    // Serial.print(COOKTOP);
    // Serial.print(",");
    // Serial.print("Right_Avg_Speed:");
    // Serial.print(rightAverageSpeed);
    // Serial.print(",");
    // Serial.print("Left_Avg_Speed:");
    // Serial.print(leftAverageSpeed);
    // Serial.print(",");
    // Serial.print("Position:");
    // Serial.println(position);
    // Serial.print(",");
    // Serial.print("Left_Angle:");
    // Serial.println(as5600_1.readAngle());
    // Serial.print(",");
    // Serial.print("D:");
    // Serial.println(leftSpeedError.d);
    
    lastTime = timeStart;
  } else {
    dt = PID_LOOP_INTERVAL / 1000.0;
  }
  //Serial.println(millis()-timeStart);
  //if (timeStart - lastTime2 > 20) {
    
  //   lastTime2 = timeStart;
  // }
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
  delay(1);
  //timeEnd = millis();
  //if (timeEnd - timeStart < dt*1000 - 2) delay(dt*1000 - 2 - (timeEnd - timeStart));
  //Serial.print("Full loop: ");
  //Serial.println(millis() - timeStart);
}

void updateEncoderPosition() {
  position += (right.currentAverageSpeed + left.currentAverageSpeed)/720.0 * dt * WHEEL_RADIUS * 6.28; //dt here is update rate (period)
}


//returns angular speed in degrees/second
//positive for forward rotation, negative for backward rotation
float getAngularSpeed(AS5600 *a, int encoderNum) {
  if (!a->detectMagnet()) return 0;
  float angleEnd = a->readAngle()/4096.0 * 360;
  float deltaA = 0;
  if (encoderNum == 0) {
    deltaA = angleEnd - lastAngle_0;
    lastAngle_0 = angleEnd;
  } 
  else {
    deltaA = angleEnd - lastAngle_1;
    lastAngle_1 = angleEnd;
  }

  if (abs(deltaA) > 180) deltaA -= sign(deltaA) * 360; //this is a fix for when it goes from 0 -> 360 or vice versa
  float speed = deltaA / dt;

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

  // Serial.println(fr);
  // Serial.println(fl);
  // Serial.println(br);
  // Serial.println(bl);

  Serial.print("Frontright:");
  Serial.print(fr);
  Serial.print(",");
  Serial.print("Frontleft:");
  Serial.print(fl);
  Serial.print(",");
  Serial.print("Backright:");
  Serial.print(br);
  Serial.print(",");
  Serial.print("Backleft:");
  Serial.println(bl);

  int front_correction = (fr - fl);
  int back_correction = (bl - br);

  bool OFF_THE_LINE = false;

  //1 means right, -1 means left, 0 means straight
  if (BASE_SPEED != 0) {
    if (m.getDrivingDirection() == 1 && BASE_SPEED > 0) {
      if (fr < 4000 && fl < 4000) OFF_THE_LINE = true;
      if (front_correction > 0 || (OFF_THE_LINE && (LAST_TURN == 1))) {
        if (!OFF_THE_LINE) LAST_TURN = 1;
        rightSpeedSetpoint = BASE_SPEED - STEERING_CONSTANT;
        leftSpeedSetpoint = BASE_SPEED;
      }
      else if (front_correction < 0 || (OFF_THE_LINE && (LAST_TURN == -1))) {
        if (!OFF_THE_LINE) LAST_TURN = -1;
        leftSpeedSetpoint =  BASE_SPEED - STEERING_CONSTANT;
        rightSpeedSetpoint = BASE_SPEED;
      } else {
        LAST_TURN = 0;
        OFF_THE_LINE = false;
        leftSpeedSetpoint = BASE_SPEED;
        rightSpeedSetpoint = BASE_SPEED;
      }
    } else {
      if (br < 4000 && bl < 4000) OFF_THE_LINE = true;
      if (back_correction > 0 || (OFF_THE_LINE && (LAST_TURN == -1))) {
        if (!OFF_THE_LINE) LAST_TURN = -1;
        leftSpeedSetpoint = BASE_SPEED + STEERING_CONSTANT*2;
        rightSpeedSetpoint = BASE_SPEED;
      }
      else if (back_correction < 0 || (OFF_THE_LINE && (LAST_TURN == 1))) {
        if (!OFF_THE_LINE) LAST_TURN = 1;
        rightSpeedSetpoint = BASE_SPEED + STEERING_CONSTANT*2;
        leftSpeedSetpoint = BASE_SPEED;
      } else {
        LAST_TURN = 0;
        OFF_THE_LINE = false;
        leftSpeedSetpoint = BASE_SPEED;
        rightSpeedSetpoint = BASE_SPEED;
      }
    }
  } else {
    rightSpeedSetpoint = 0;
    leftSpeedSetpoint = 0;
  }

  Serial.println(OFF_THE_LINE);
  Serial.println(LAST_TURN);
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
  if (BRAKE_OFF) {
    equalSpeedSet(-BASE_SPEED / 3.0);
    right.setSpeed(rightSpeedSetpoint);
  //if (BASE_SPEED > 0) leftSpeedSetpoint = -2250;
    left.setSpeed(leftSpeedSetpoint);
    delay(25);
  }
  right.setSpeed(0); 
  left.setSpeed(0);
  right.clearSpeeds();
  left.clearSpeeds();
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