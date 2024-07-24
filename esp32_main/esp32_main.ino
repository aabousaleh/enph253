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

//initialize i2c bus for right encoder
AS5600 as5600_0(&Wire);

//initialize i2c bus for left encoder
AS5600 as5600_1(&Wire1);

//Speed constants
const int MAX_SPEED = 1000;
int BASE_SPEED = 600;
double STEERING_CONSTANT = 0.272;
double TURNING_SPEED = 0.1 * MAX_SPEED;
double ADJUSTING_SPEED = 0.015 * MAX_SPEED;
int TURNING_DELAY = 855;

double dt = LOOP_INTERVAL / 1000.0; //in s
unsigned long timeStart = 0; //for loop timing
unsigned long timeEnd = dt*1000;
unsigned long lastTime = 0; //for loop timing
double lastAngle_0 = 0;
double lastAngle_1 = 0;

Map m;

bool DRIVING = true; //turns off motors when false

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

volatile double position = 5.70;

double GAIN_P = 0.45;
double GAIN_I = 0.35;
double GAIN_D = 0.001;

int LAST_TURN = 0;

double intendedPosition;
Instruction currentInstruction;

volatile bool stationInterrupt = false;

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

  // attachInterrupt(digitalPinToInterrupt(RS_TCRT), updateLocationRight, RISING);
  // attachInterrupt(digitalPinToInterrupt(LS_TCRT), updateLocationLeft, RISING);

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

  currentInstruction = m.getNextInstruction();
  updateInstruction();
}

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
  //TODO: alot
  /*
  1. Add interrupts for microswitches to freeze the robot
  2. Communication
  3. create log of recipe stack (to use for isReady)
  4. figure out blocking
  */

  //receive status from other robot
  if (timeStart - lastTime > LOOP_INTERVAL) {
    double rightCurrentAngle = as5600_0.readAngle() / 4096.0 * 360;
    double leftCurrentAngle = as5600_1.readAngle() / 4096.0 * 360;
    double rightDelta = (rightCurrentAngle - lastAngle_0);
    double leftDelta = (leftCurrentAngle - lastAngle_1);
    if (abs(rightDelta) > 180) rightDelta -= sign(rightDelta) * 360;
    if (abs(leftDelta) > 180) leftDelta -= sign(leftDelta) * 360;
    double deltaAngleAverage = (rightDelta + leftDelta)/2;
    position += deltaAngleAverage/360.0 * 6.28 * WHEEL_RADIUS * m.getFacingDirection();
    lastAngle_0 = rightCurrentAngle;
    lastAngle_1 = leftCurrentAngle;
    switch (m.state) {
      case MOVE: {
        double distance = intendedPosition - position;
        if (abs(distance) > 0.15 && !stationInterrupt) {
          BASE_SPEED = abs(distance) > 10 ? 240 * sign(distance) * m.getFacingDirection() : 100 * sign(distance);
          equalSpeedSet(BASE_SPEED);
          // rightAngularSpeed = getAngularSpeed(&as5600_0, 0);
          // leftAngularSpeed = getAngularSpeed(&as5600_1, 1);
          // right.updateSpeeds(rightAngularSpeed);
          // left.updateSpeeds(leftAngularSpeed);
          // double rightAverageSpeed = right.averageSpeed();
          // double leftAverageSpeed = left.averageSpeed();

          lineSensingCorrection();
          move();
          //updateEncoderPosition();

          //Serial.print("Position:");
          //Serial.print(position);
          //Serial.print(",");
          //Serial.print("Setpoint:");
          //Serial.print(intendedPosition);
          //Serial.print(",");
          //Serial.print("Right_Angle:");
          //Serial.print(rightCurrentAngle);
          //Serial.print(",");
          //Serial.print("Left_Angle:");
          //Serial.print(leftCurrentAngle);
          //Serial.print(",");
          //Serial.print("deltaAngle:");
          //Serial.println(deltaAngleAverage);

        } else {
          brake(true);
          delay(1000);
          m.state = ADJUST;
        }
        
        lastTime = timeStart;
        break;
      }
      case ADJUST: {
        int stationToRead = m.getFacingDirection() == 1 ? LS_TCRT : RS_TCRT;

        if (!digitalRead(stationToRead)) {
          equalSpeedSet(ADJUSTING_SPEED * sign(intendedPosition - position) * m.getFacingDirection());
          //lineSensingCorrection();
          move();
          updateEncoderPosition();
        } else {
          // double final = position + sign(rightSpeedSetpoint) * TAPE_WIDTH / 2.0;
          // while (position != final) {
          //   equalSpeedSet(ADJUSTING_SPEED / 2.0 * sign(rightSpeedSetpoint) * m.getFacingDirection());
          //   move();
          //   updateEncoderPosition();
          // }
          // currentInstruction = m.getNextInstruction();
          // updateInstruction();
          brake(false);
          delay(2000);
          currentInstruction = m.getNextInstruction();
          updateInstruction();
        }

        break;
      }
      case SPIN: {
        //Serial.println("spin");
        spin180Encoder(1);
        currentInstruction = m.getNextInstruction();
        updateInstruction();
        break;
      }
      case ARM: {
        if (/*isReady*/ true) { //isReady to check that the other robot completed their task
          if (currentInstruction == PLACE) {
            //place();
            if (abs(position - COOKTOP) < 1) {
              //start timer
              //broadcast it
            }
          } else if (currentInstruction == GRAB) {
            //grab();
            if (abs(position - COOKTOP) < 1) {
              //stop timer
              //broadcast it
            }
          }
          //send status to other robot
          currentInstruction = m.getNextInstruction();
          updateInstruction();
        }

        break;
      }
      case WAITING: {

        break;
      }
      default: {

        break;
      }
    }
  } else {
    dt = LOOP_INTERVAL / 1000.0;
  }
  delay(1);
}

void rightStationInterrupt() {

}

void leftStationInterrupt() {

}
void updateEncoderPosition() {
  position += m.getFacingDirection() * (right.currentAverageSpeed + left.currentAverageSpeed)/720.0 * dt * WHEEL_RADIUS * 6.28; //dt here is update rate (period)
}

void updateInstruction() {
  switch (currentInstruction) {
    case GO: {
      intendedPosition = m.getNextLocation();
      m.state = MOVE;
      attachInterrupt(digitalPinToInterrupt(RS_TCRT), rightStationInterrupt, RISING);
      attachInterrupt(digitalPinToInterrupt(LS_TCRT), leftStationInterrupt, RISING);
      break;
    }
    case GRAB: {
      m.state = ARM;
      break;
    }
    case PLACE: {
      m.state = ARM;
      break;
    }
    case TURN: {
      m.state = SPIN;
      break;
    }
    case WAIT: {
      m.state = WAITING;
      break;
    }
    case END: {
      //TODO: fix this case
      // m.nextRecipe();
      DRIVING = false;
      m.state = WAITING;
      break;
    }
    default: {
      //this should never happen
      //if it does... die...?
    }
  }
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

  // Serial.print("Frontright:");
  // Serial.print(fr);
  // Serial.print(",");
  // Serial.print("Frontleft:");
  // Serial.print(fl);
  // Serial.print(",");
  // Serial.print("Backright:");
  // Serial.print(br);
  // Serial.print(",");
  // Serial.print("Backleft:");
  // Serial.println(bl);

  int front_correction = (fr - fl);
  int back_correction = (bl - br);

  bool OFF_THE_LINE = false;

  //1 means right, -1 means left, 0 means straight
  if (BASE_SPEED != 0) {
    if (m.getDrivingDirection() == 1 && BASE_SPEED > 0) {
      if (fr < 4000 && fl < 4000) OFF_THE_LINE = true;
      if (front_correction > 0 || (OFF_THE_LINE && (LAST_TURN == 1))) {
        Serial.println("F right");
        rightSpeedSetpoint = BASE_SPEED * (1.0 - (STEERING_CONSTANT + LAST_TURN*0.15));
        leftSpeedSetpoint = BASE_SPEED;
        if (!OFF_THE_LINE) LAST_TURN = 1;
      }
      else if (front_correction < 0 || (OFF_THE_LINE && (LAST_TURN == -1))) {
        //Serial.println("F left");
        leftSpeedSetpoint =  BASE_SPEED * (1.0 - (STEERING_CONSTANT + LAST_TURN*0.15));
        rightSpeedSetpoint = BASE_SPEED;
        if (!OFF_THE_LINE) LAST_TURN = -1;
      } else {
        //Serial.println("F straight");
        LAST_TURN = 0;
        OFF_THE_LINE = false;
        leftSpeedSetpoint = BASE_SPEED;
        rightSpeedSetpoint = BASE_SPEED;
      }
    } else {
      if (br < 4000 && bl < 4000) OFF_THE_LINE = true;
      if (back_correction > 0 || (OFF_THE_LINE && (LAST_TURN == -1))) {
        if (!OFF_THE_LINE) LAST_TURN = -1;
        //Serial.println("B right");
        leftSpeedSetpoint = BASE_SPEED * (1.0 - STEERING_CONSTANT*2.3);
        rightSpeedSetpoint = BASE_SPEED;
      }
      else if (back_correction < 0 || (OFF_THE_LINE && (LAST_TURN == 1))) {
        if (!OFF_THE_LINE) LAST_TURN = 1;
        //Serial.println("B left");
        rightSpeedSetpoint = BASE_SPEED * (1.0 - STEERING_CONSTANT*2.3);
        leftSpeedSetpoint = BASE_SPEED;
      } else {
        LAST_TURN = 0;
        //Serial.println("B straight");
        OFF_THE_LINE = false;
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
  right.setSpeed(rightSpeedSetpoint * DRIVING);
  left.setSpeed(leftSpeedSetpoint * DRIVING);
}

void brake(bool useBackdrive) {
  if (useBackdrive) {
    equalSpeedSet(-BASE_SPEED / 3.0);
    right.setSpeed(rightSpeedSetpoint);
    left.setSpeed(leftSpeedSetpoint);
    delay(20);
  }
  right.setSpeed(0); 
  left.setSpeed(0);
  right.clearSpeeds();
  left.clearSpeeds();
}

void spin180Encoder(int dir) {
  double lastAngle = as5600_1.readAngle() / 4096.0 * 360;
  double finalAnglePosition = 12.4;
  double currentPosition = 0;
  right.setSpeed(dir * -TURNING_SPEED);
  left.setSpeed(dir * TURNING_SPEED*1.10);
  while (finalAnglePosition - currentPosition > 0) {
    double currentAngle = as5600_1.readAngle() / 4096.0 * 360;
    double deltaA = currentAngle - lastAngle;
    if (abs(deltaA) > 180) deltaA -= sign(deltaA) * 360;
    currentPosition += deltaA/360.0 * 6.28 * WHEEL_RADIUS;
    lastAngle = currentAngle;
    //delayMicroseconds(200);
  }
  spinBrake(1);
  // DRIVING = false;
  // brake();
  // DRIVING = true;
  m.flipFacingDirection();
}

//dir: 1 CW, -1 CCW
void spin180(int dir) {
  right.setSpeed(dir * -TURNING_SPEED);
  left.setSpeed(dir * TURNING_SPEED*1.10);
  delay(TURNING_DELAY);
  spinBrake(dir);
  m.flipFacingDirection();
}

void spinBrake(int dir) {
  right.setSpeed(dir * TURNING_SPEED / 3.0);
  left.setSpeed(dir * -TURNING_SPEED / 2.0);
  delay(10);
  right.setSpeed(0);
  left.setSpeed(0);
  right.clearSpeeds();
  left.clearSpeeds();
}

//these may not be useful at all
// void updateLocationRight() {
//   if (m.getFacingDirection() == (1 - ROBOT_ID*2)) { // 1 - 0*2 = 1, 1 - 1*2 = -1
//     m.location += m.getDrivingDirection();
//   }
// }

// void updateLocationLeft() {
//   if (m.getFacingDirection() == (-1 + ROBOT_ID*2)) {
//     m.location -= m.getDrivingDirection();
//   }
// }