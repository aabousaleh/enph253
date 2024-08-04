#include "Arduino.h"
#include "AS5600.h"
#include "pid.h"
#include "motor.h"
#include "definitions.h"
// #include "arduino-timer.h"
#include "map.h"
//#include "movement.ino"
#include "arm.h"
#include "vacuum.h"

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
int BASE_SPEED = 950;
double STEERING_CONSTANT = 0.375;
double TURNING_SPEED = 0.10 * MAX_SPEED;
double ADJUSTING_SPEED = 0.15 * MAX_SPEED;
int TURNING_DELAY = 500;

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

volatile double position = CHEESE;

int LAST_TURN = 0;

double intendedPosition;
Instruction currentInstruction;
Ingredient currentIngredient;

float height = 11;
float reach = 15;
float grabbing = 0.075;

Vacuum v;

int blackTapeCounter = 0;

void setup() {
  
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

  ledcAttach(CLAW_SERVO, SERVO_PWM_FREQUENCY, SERVO_PWM_RESOLUTION);

  digitalWrite(PUMP, LOW);
  digitalWrite(VALVE, LOW);

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

  setupArmServos();

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

  //DRIVING = false;
}

void loop() {
  timeStart = millis();

  /*if (Serial.available() > 0){
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
  }*/
  //TODO: alot
  /*
  1. Add interrupts for microswitches to freeze the robot
  2. Communication
  3. create log of recipe stack (to use for isReady)
  4. figure out blocking
  */

  //receive status from other robot
  if (timeStart - lastTime > LOOP_INTERVAL) {
    lastTime = timeStart;
    switch (m.state) {
      case MOVE: {
        if (position == intendedPosition) {
          currentInstruction = m.getNextInstruction();
          updateInstruction();
        } else {
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
          double distance = intendedPosition - position;
          m.setMovingDirection(sign(distance));
          if ( abs(distance) > 1) {
            // BASE_SPEED = 650 * m.getDrivingDirection();//abs(distance) > 10 ? 240 * m.getDrivingDirection() : 100 * m.getDrivingDirection();
            equalSpeedSet(BASE_SPEED * m.getDrivingDirection());
            lineSensingCorrection();
            move(rightSpeedSetpoint, leftSpeedSetpoint);
          } else {
            // BASE_SPEED = 90 * m.getDrivingDirection();
            brake(true);
            // equalSpeedSet(ADJUSTING_SPEED);
            // move(rightSpeedSetpoint, leftSpeedSetpoint);
            // brake(false);
            // delay(250);
            delay(500);
            currentInstruction = m.getNextInstruction();
            updateInstruction();
            //delay(1000);
            //int stationToRead = stationRightOrLeft(intendedPosition, ROBOT_ID) * m.getFacingDirection() == 1 ? RS_TCRT : LS_TCRT;
            //attachInterrupt(digitalPinToInterrupt(LS_TCRT), stationInterrupt, RISING);
            //m.state = ADJUST;
            //delay(500);
            // Serial.println("going to adjust");
          }
        }
        break;
      }
      case ADJUST: {
        if (intendedPosition == SERVING) {
          brake(true);
          currentInstruction = m.getNextInstruction();
          updateInstruction();
        } else {
          int ls = digitalRead(LS_TCRT);
          if (ls == 1) blackTapeCounter++;
          if (ls == 0) blackTapeCounter = 0;
          //Serial.print("leftstation in adjust: ");
          //Serial.println(ls);
          equalSpeedSet(ADJUSTING_SPEED * m.getDrivingDirection());
          lineSensingCorrection();
          move(rightSpeedSetpoint, leftSpeedSetpoint);
          if (blackTapeCounter >= 2) {
            //Serial.println("on tape");
          //   equalSpeedSet(ADJUSTING_SPEED * sign(BASE_SPEED));
          //   //lineSensingCorrection();
          //   move(rightSpeedSetpoint, leftSpeedSetpoint);
          //   //Serial.println("adjusting");
          //   //Serial.println(rightSpeedSetpoint);
          // } else {
          // //   // double final = position + sign(rightSpeedSetpoint) * TAPE_WIDTH / 2.0;
          // //   // while (position != final) {
          // //   //   equalSpeedSet(ADJUSTING_SPEED / 2.0 * sign(rightSpeedSetpoint) * m.getFacingDirection());
          // //   //   move();
          // //   // }
          // //   // currentInstruction = m.getNextInstruction();
          // //   // updateInstruction();
            brake(true);
            position = intendedPosition;
            currentInstruction = m.getNextInstruction();
            updateInstruction();
            blackTapeCounter = 0;
          //   delay(1000);
          //   // Serial.println("reached");
          }
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
        delay(500);
        currentInstruction = m.getNextInstruction();
        updateInstruction();
        break;
        if (true) { //isReady to check that the other robot completed their task
          if (currentInstruction == PLACE) {
            place();
            // if (abs(position - COOKTOP) < 1) {
            //   //start timer
            //   //broadcast it
            // }
          } else if (currentInstruction == GRAB) {
            currentIngredient = m.getNextIngredient();
            grab(currentIngredient);
            // if (abs(position - COOKTOP) < 1) {
            //   //stop timer
            //   //broadcast it
            // }
          }
          //send status to other robot
          delay(1000);
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

void grab(Ingredient i) {
  switch (i) {
    case cheese: {
      moveToXY(20, 17);
      delay(750);
      moveToXY(33.8, 17);
      delay(1000);
      moveToXY(33.8, 7.8);
      digitalWrite(PUMP, HIGH);
      delay(1000);
      moveToXY(33.8, 18);
      delay(500);
      moveToXY(15, 12);
      break;
    }
    case patty: {

      break;
    }
    case bun: {

      break;
    }
    case plate: {
      for (int i = 0; i <= 5; i++) {
        moveToXY(20, 15 - i);
        delay(25);
      }
      delay(500);
      for (int i = 0; i <= 16; i++) {
        moveToXY(20 + i, 10);
        delay(25);
      }
      moveToXY(37, 15.5);
      delay(1000);
      moveToXY(20, 15.5);
      delay(500);
      moveToXY(16, 12.5);
      break;
    }
    case tomato: {

      break;
    }
    case lettuce: {

      break;
    }
    case potato: {
      //stupid fries
      break;
    }
  }
}

void place() {
  //move arm to correct location
  //turn off pump
  //switch solenoid
  //bring arm back
  if (currentIngredient == plate) {
    moveToXY(19, 16);
    delay(500);
    for (int i = 0; i <= 15; i++) {
      moveToXY(20 + i, 16);
      delay(25);
    }
    delay(500);
    for (double i = 0; i <= 6.5; i = i + 0.5) {
      moveToXY(35, 16 - i);
      delay(25);
    }
    delay(500);
    moveToXY(16, 9.5);
    delay(500);
    moveToXY(15, 12);
  } else {
    moveToXY(20,16);
    delay(500);
    moveToXY(41, 16);
    delay(500);
    digitalWrite(VALVE, HIGH);
    digitalWrite(PUMP, LOW);
    delay(100); //TODO: MIGHT NEED TO CHANGE THIS DELAY VALUE
    digitalWrite(VALVE, LOW);
    delay(1000);
    moveToXY(15, 12);
  }
}

void updateInstruction() {
  switch (currentInstruction) {
    case GO: {
      intendedPosition = m.getNextLocation();
      m.state = MOVE;
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
      DRIVING = false;
      brake(false);
      m.state = WAITING;
      break;
    }
    case END: {
      //TODO: fix this case
      // m.nextRecipe();
      // DRIVING = false;
      brake(false);
      m.nextRecipe();
      currentInstruction = m.getNextInstruction();
      updateInstruction();
      //m.state = WAITING;
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

  int front_correction = (fr - fl);
  int back_correction = (bl - br);

  bool OFF_THE_LINE = false;

  //1 means right, -1 means left, 0 means straight
  if (BASE_SPEED != 0) {
    if (m.getDrivingDirection() == 1) {
      if (fr < 4000 && fl < 4000) OFF_THE_LINE = true;
      if (front_correction > 0 || (OFF_THE_LINE && (LAST_TURN == 1))) {
        rightSpeedSetpoint = BASE_SPEED * (1.0 - (STEERING_CONSTANT));
        leftSpeedSetpoint = BASE_SPEED;
        if (!OFF_THE_LINE) LAST_TURN = 1;
      }
      else if (front_correction < 0 || (OFF_THE_LINE && (LAST_TURN == -1))) {
        leftSpeedSetpoint =  BASE_SPEED * (1.0 - (STEERING_CONSTANT));
        rightSpeedSetpoint = BASE_SPEED;
        if (!OFF_THE_LINE) LAST_TURN = -1;
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
        leftSpeedSetpoint = BASE_SPEED * (1.0 - STEERING_CONSTANT*2.25);
        rightSpeedSetpoint = BASE_SPEED;
      }
      else if (back_correction < 0 || (OFF_THE_LINE && (LAST_TURN == 1))) {
        if (!OFF_THE_LINE) LAST_TURN = 1;
        rightSpeedSetpoint = BASE_SPEED * (1.0 - STEERING_CONSTANT*2.25);
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

}

void equalSpeedSet(double speed) {
  rightSpeedSetpoint = speed;
  leftSpeedSetpoint = speed;
}

void move(double rss, double lss) {
  left.setSpeed(lss * DRIVING);
  right.setSpeed(rss * DRIVING);
}

void brake(bool useBackdrive) {
  if (useBackdrive) {
    move(0,0);
    delay(10);
    //equalSpeedSet(-BASE_SPEED / 2);
    move(-rightSpeedSetpoint, -leftSpeedSetpoint);
    // right.setSpeed(rightSpeedSetpoint);
    // left.setSpeed(leftSpeedSetpoint);
    delay(20);
  }
  move(0,0);
}

void spin180Encoder(int dir) {
  double lastAngle = as5600_1.readAngle() / 4096.0 * 360;
  double finalAnglePosition = 10.15;//12.4;
  double currentPosition = 0;
  move(dir * -TURNING_SPEED, dir * TURNING_SPEED * 1.15);
  int jitter = 0;
  unsigned long currentMillis = millis();
  while (finalAnglePosition - currentPosition > 0) {
    double currentAngle = as5600_1.readAngle() / 4096.0 * 360;
    double deltaA = currentAngle - lastAngle;
    if (abs(deltaA) > 180) deltaA -= sign(deltaA) * 360;
    currentPosition += deltaA/360.0 * 6.28 * WHEEL_RADIUS * dir;
    lastAngle = currentAngle;
    // if (millis() - currentMillis > 50) {
    //   currentMillis = millis();
    //   jitter *= -1;
    // }
    // if (jitter == 1) {
    //   move(dir * -TURNING_SPEED, dir * TURNING_SPEED);
    // }
    // if (jitter == -1) {
    //   brake(false);
    // }
  }
  // double fr = analogRead(FR_TCRT);
  // while (fr < 4090) {
  //   move(dir * -TURNING_SPEED * 0.65, dir * TURNING_SPEED * 0.65);
  //   fr = analogRead(FR_TCRT);
  // }
  // //spinBrake(1);
  // DRIVING = false;
  // brake();
  // DRIVING = true;
  brake(false);
  delay(1000);
  m.flipFacingDirection();
  lastAngle_0 = as5600_0.readAngle() / 4096.0 * 360;
  lastAngle_1 = as5600_1.readAngle() / 4096.0 * 360;
}

//dir: 1 CW, -1 CCW
void spin180(int dir) {
  move(dir * -TURNING_SPEED, dir * TURNING_SPEED * 1.15);
  delay(TURNING_DELAY);
  int ls = digitalRead(LS_TCRT);
  while (ls) {
    move(dir * -TURNING_SPEED * 0.65, dir * TURNING_SPEED * 1.15 * 0.65);
  }
  brake(true);
  m.flipFacingDirection();
}

void spinBrake(int dir) {
  right.setSpeed(dir * TURNING_SPEED / 7.5);
  left.setSpeed(dir * -TURNING_SPEED / 5.0);
  delay(10);
  right.setSpeed(0);
  left.setSpeed(0);
  right.clearSpeeds();
  left.clearSpeeds();
}


//1 is right, -1 is left, 0 is error
int stationRightOrLeft(double station, int robotID) {
  if (robotID == 0) {
    if (station == TOMATOES || station == CUTTING || station == COOKTOP || station == PLATES) return 1;
    if (station == PATTIES || station == BUNS || station == POTATOES) return -1;
    return 0;
  } else if (robotID == 1) {
    if (station == TOMATOES || station == CUTTING || station == COOKTOP || station == PLATES) return -1;
    if (station == CHEESE || station == SERVING || station == LETTUCE) return 1;
    return 0;
  }
}
