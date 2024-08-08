#include "Arduino.h"
#include "AS5600.h"
#include "motor.h"
#include "definitions.h"
#include "map.h"
#include "arm.h"

#include <esp_now.h>
#include <WiFi.h>

uint8_t broadcastAddress[] = {0x64, 0xB7, 0x08, 0x9C, 0x65, 0x90};

esp_now_peer_info_t peerInfo;

typedef struct Message{
  int checkpoint;
  bool occupyingPlate;
}Message;

Message dataReceived = {100, false};
Message dataToBeSent = {0, false};

int selfCheckpoint = 0;
int otherCheckpoint = 0;
int checkpointToWaitFor = 1;
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
double STEERING_CONSTANT = 0.31;
double TURNING_SPEED = 0.25 * MAX_SPEED;//0.15 * MAX_SPEED;
double ADJUSTING_SPEED = 0.25 * MAX_SPEED;//0.085 * MAX_SPEED;
int TURNING_DELAY = 500;

double dt = LOOP_INTERVAL / 1000.0; //in s
unsigned long timeStart = 0; //for loop timing
unsigned long timeEnd = dt*1000;
unsigned long lastTime = 0; //for loop timing
double lastAngle_0 = 0;
double lastAngle_1 = 0;

Map m;

int DRIVING = 1; //turns off motors when 0

volatile double rightSpeedSetpoint = 0; //in degrees/sec
Motor right(PWM_RIGHT_1, PWM_RIGHT_2, MAX_SPEED);
double rightAngularSpeed;

volatile double leftSpeedSetpoint = 0;
Motor left(PWM_LEFT_1, PWM_LEFT_2, MAX_SPEED);
double leftAngularSpeed;

volatile double position = CHEESE;

int LAST_TURN = 0;

double intendedPosition;
Instruction currentInstruction;
Ingredient currentIngredient;

const float returnY = 10.5;
const float returnX = 14.5;

int blackTapeCounter = 0;

double rightLineSensingCorrection = 1;
double leftLineSensingCorrection = 1;

void setup() {
  
  Serial.begin(115200);

  //pins

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

    setupArmServos();


  //encoders
  
    //initialize the i2c busses
    Wire.begin(I2C_SDA0, I2C_SCL0);
    Wire1.begin(I2C_SDA1, I2C_SCL1);

    //initialize right encoder
    as5600_0.begin();
    as5600_0.setDirection(AS5600_COUNTERCLOCK_WISE);  //  check encoder/magnet to make sure this is correct
    Serial.print("Connect device 0: ");
    Serial.println(as5600_0.isConnected() ? "true" : "false");
    delay(50);

    as5600_1.begin();
    as5600_1.setDirection(AS5600_CLOCK_WISE);  //  default, just be explicit.
    Serial.print("Connect device 1: ");
    Serial.println(as5600_1.isConnected() ? "true" : "false");
    delay(50);

    // server.begin();
    lastAngle_0 = as5600_0.readAngle() / 4096.0 * 360;
    lastAngle_1 = as5600_1.readAngle() / 4096.0 * 360;

  equalSpeedSet(BASE_SPEED);
  //espnnow
    WiFi.mode(WIFI_STA);
    // Init ESP-NOW
    if (esp_now_init() != ESP_OK) {
      Serial.println("Error initializing ESP-NOW");
      return;
    }
    esp_now_register_send_cb(esp_now_send_cb_t(OnDataSent));
    memcpy(peerInfo.peer_addr, broadcastAddress, 6);
    peerInfo.channel = 0;  
    peerInfo.encrypt = false;
    if (esp_now_add_peer(&peerInfo) != ESP_OK){
      Serial.println("Failed to add peer");
      return;
    }
    esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
    delay(1000);
    // WiFi.begin(ssid, password);
    // while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    //   delay(5000);
    //   ESP.restart();
    // }
  currentInstruction = m.getNextInstruction();
  updateInstruction();

  //DRIVING = 0;
}

void loop() {
  timeStart = millis();
  /*if (Serial.available() > 0) {
    // Read the incoming byte
    char incomingByte = Serial.read();
    // Check which key was pressed
    if (incomingByte == 'w') {
      height += 0.2;
      Serial.print("Height: ");
      Serial.println(height);
    } else if (incomingByte == 's') {
      height -= 0.2;
      Serial.print("Height: ");
      Serial.println(height);
    } else if (incomingByte == 'a') {
      reach += 0.2;
      Serial.print("Reach: ");
      Serial.println(reach);
    } else if (incomingByte == 'd') {
      reach -= 0.2;
      Serial.print("Reach: ");
      Serial.println(reach);
    } else if (incomingByte == 'q') {
      grabbing += 0.002;
      Serial.print("Grabbing: ");
      Serial.println(grabbing);
    } else if (incomingByte == 'e') {
      grabbing -= 0.002;
      Serial.print("Grabbing: ");
      Serial.println(grabbing);
    }
  }
  moveToXY(reach, height);
  delay(10);*/
  
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
          if (abs(distance) > 12 || (abs(distance) > 1 && intendedPosition == SERVING)) {
            //BASE_SPEED = 650 * m.getDrivingDirection();//abs(distance) > 10 ? 240 * m.getDrivingDirection() : 100 * m.getDrivingDirection();
            equalSpeedSet( BASE_SPEED * m.getDrivingDirection());
            lineSensingCorrection();
            move(rightSpeedSetpoint * rightLineSensingCorrection, leftSpeedSetpoint * leftLineSensingCorrection);
          } else {
            brake(false);
            // equalSpeedSet(ADJUSTING_SPEED);
            // move(rightSpeedSetpoint, leftSpeedSetpoint);
            // brake(false);
            delay(500);
            // currentInstruction = m.getNextInstruction();
            // updateInstruction();
            m.state = ADJUST;
          }
        }
        break;
      }
      case ADJUST: {
        if (intendedPosition == SERVING) {
          position = intendedPosition;
          brake(false);
          currentInstruction = m.getNextInstruction();
          updateInstruction();
        } else {
          int ls = digitalRead(LS_TCRT);
          if (ls == 1) blackTapeCounter++;
          if (ls == 0) blackTapeCounter = 0;
          //equalSpeedSet(ADJUSTING_SPEED * m.getDrivingDirection());
          rightSpeedSetpoint = ADJUSTING_SPEED * m.getDrivingDirection();
          leftSpeedSetpoint = ADJUSTING_SPEED * m.getDrivingDirection() * 1.5;
          lineSensingCorrection();
          move(rightSpeedSetpoint * rightLineSensingCorrection, leftSpeedSetpoint * leftLineSensingCorrection);
          if (blackTapeCounter >= 1) {
            brake(true);
            position = intendedPosition;
            currentInstruction = m.getNextInstruction();
            updateInstruction();
            blackTapeCounter = 0;
            delay(500);
          }
        }
        break;
      }
      case SPIN: {
        spin180Encoder(1);
        currentInstruction = m.getNextInstruction();
        updateInstruction();
        break;
      }
      case ARM: {
        //if (stationRightOrLeft(position) != m.getFacingDirection()) spin180Encoder(1);
        if (currentInstruction == PLACE) {
        //   // if (position == PLATES) {
        //   //   if (dataReceived.occupyingPlate) break;
        //   //   dataToBeSent.occupyingPlate = true;
        //   //   esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &dataToBeSent, sizeof(dataToBeSent));
        //   // }
          place();
        } else if (currentInstruction == GRAB) {
        //   currentIngredient = m.getNextIngredient();
        //   // if (position == PLATES) {
        //   //   if (dataReceived.occupyingPlate) break;
        //   //   dataToBeSent.occupyingPlate = true;
        //   //   esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &dataToBeSent, sizeof(dataToBeSent));
        //   // }
          grab(currentIngredient);
        }
        // dataToBeSent.occupyingPlate = false;
        // esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &dataToBeSent, sizeof(dataToBeSent));
        //send status to other robot
        delay(500);
        currentInstruction = m.getNextInstruction();
        updateInstruction();

        break;
      }
      case WAITING: {
        if (dataReceived.checkpoint >= checkpointToWaitFor) {
          currentInstruction = m.getNextInstruction();
          updateInstruction();
          checkpointToWaitFor++;
        }
        break;
      }
      case SPECIAL_STATE: {
        for (int i = 0; i < 8; i++) {
          move(-(100 + 100*i), 100 + 100*i);
          delay(250);
        }
        move(-1000, 1000);
        delay(1500);
        setVac(false);
        delay(400);
        moveToXY(44, 20);
        delay(1500);
        currentInstruction = m.getNextInstruction();
        updateInstruction();
        moveToXY(returnX, returnY);
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

void updateInstruction() {
  switch (currentInstruction) {
    case GO: {
      Serial.println("Go");
      intendedPosition = m.getNextLocation();
      m.state = MOVE;
      break;
    }
    case GRAB: {
      Serial.println("Grab");
      m.state = ARM;
      break;
    }
    case PLACE: {
      Serial.println("Place");
      m.state = ARM;
      break;
    }
    case TURN: {
      Serial.println("Turn");
      m.state = SPIN;
      break;
    }
    case WAIT: {
      Serial.println("Wait");
      DRIVING = false;
      brake(false);
      checkpointToWaitFor = 100;
      m.state = WAITING;
      break;
    }
    case END: {
      brake(false);
      m.nextRecipe();
      int selfCheckpoint = 0;
      int otherCheckpoint = 0;
      int checkpointToWaitFor = 1;
      currentInstruction = m.getNextInstruction();
      updateInstruction();
      break;
    }
    case SEND_CHECKPOINT: {
      Serial.println("Send");
      dataToBeSent.checkpoint = ++selfCheckpoint;
      esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &dataToBeSent, sizeof(dataToBeSent));
      break;
    }
    case RECEIVE_CHECKPOINT: {
      Serial.println("Receive");
      m.state = WAITING;
    }
    case SPECIAL: {
      m.state = SPECIAL_STATE;
    }
    default: {
      //this should never happen
      //if it does... die...?
    }
  }
}

void grab(Ingredient i) {
  switch (i) {
    case plate: {
      for (int i = 0; i <= 5; i++) {
        moveToXY(20, 15.3 - i);
        delay(25);
      }
      delay(300);
      for (int i = 0; i <= 18; i++) {
        moveToXY(20 + i, 10.3);
        delay(25);
      }
      moveToXY(38.5, 18);
      delay(1000);
      moveToXY(25, 18);
      delay(300);
      moveToXY(returnX, returnY);
      break;
    }
    case tomato:
    case lettuce:
    case cheese: {
      moveToXY(20, 17);
      delay(550);
      moveToXY(41.5, 17);
      delay(550);
      //turn on vacuum
        setVac(true);
      moveToXY(43.5, 10.6);
      delay(550);
      moveToXY(41.5, 15);
      delay(300);
      moveToXY(returnX, returnY);
      break;
    }
    case patty: {
      moveToXY(20, 17);
      delay(550);
      moveToXY(41.5, 17);
      delay(750);
      //turn on vacuum
        setVac(true);
      moveToXY(43.5, 11.6);
      delay(750);
      moveToXY(41.5, 16);
      delay(500);
      moveToXY(returnX, returnY);
      break;
    }
    case top_bun: {
      moveToXY(20, 17);
      delay(550);
      moveToXY(38, 18);
      delay(750);
      //turn on vacuum
        setVac(true);
      moveToXY(43.5, 13);
      delay(750);
      moveToXY(35, 18);
      delay(750);
      moveToXY(returnX, returnY);
      break;
    }
    case bottom_bun: {
      moveToXY(20, 17);
      delay(550);
      moveToXY(42.5, 17);
      delay(750);
      //turn on vacuum
        setVac(true);
      moveToXY(43.5, 11.5);
      delay(750);
      moveToXY(41.5, 17);
      delay(500);
      moveToXY(returnX, returnY);
      break;
    }
    default: {
      break;
    }
  }
}

void place() {
  if (currentIngredient == plate) {
    moveToXY(19, 16);
    delay(300);
    for (int i = 0; i <= 15; i++) {
      moveToXY(20 + i, 16);
      delay(25);
    }
    delay(300);
    for (double i = 0; i <= 6.5; i = i + 0.5) {
      moveToXY(35, 16 - i);
      delay(25);
    }
    delay(300);
    moveToXY(16, 9.5);
    delay(300);
    moveToXY(returnX, returnY);
  } else {
    for (int i = 0; i < 23; i++) {
      moveToXY(20 + i, 12 + i*0.35);
      delay(50);
      if (i == 15) {
        setVac(false);
      }
    }
    moveToXY(44.5, 17.5);
    delay(300);
    moveToXY(44.5, 16.5);
    delay(1500);
    for (int i = 0; i < 23; i++) {
      moveToXY(43 - i, 20 - i*0.35);
      delay(50);
    }
    moveToXY(returnX, returnY);
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
      if (fr < 4050 && fl < 4050) OFF_THE_LINE = true;
      if (front_correction > 0 || (OFF_THE_LINE && (LAST_TURN == 1))) {
        rightLineSensingCorrection = (1.0 - (STEERING_CONSTANT));
        leftLineSensingCorrection = 1;
        if (!OFF_THE_LINE) LAST_TURN = 1;
      }
      else if (front_correction < 0 || (OFF_THE_LINE && (LAST_TURN == -1))) {
        leftLineSensingCorrection = (1.0 - (STEERING_CONSTANT));
        rightLineSensingCorrection = 1;
        if (!OFF_THE_LINE) LAST_TURN = -1;
      } else {
        LAST_TURN = 0;
        OFF_THE_LINE = false;
        leftLineSensingCorrection = 1;
        rightLineSensingCorrection = 1;
      }
    } else {
      if (br < 4000 && bl < 4000) OFF_THE_LINE = true;
      if (back_correction > 0 || (OFF_THE_LINE && (LAST_TURN == -1))) {
        leftLineSensingCorrection = (1.0 - STEERING_CONSTANT*2.25);
        rightLineSensingCorrection = 1;
        if (!OFF_THE_LINE) LAST_TURN = -1;
      }
      else if (back_correction < 0 || (OFF_THE_LINE && (LAST_TURN == 1))) {
        rightLineSensingCorrection = (1.0 - STEERING_CONSTANT*2.25);
        leftLineSensingCorrection = 1;
        if (!OFF_THE_LINE) LAST_TURN = 1;
      } else {
        LAST_TURN = 0;
        OFF_THE_LINE = false;
        leftLineSensingCorrection = 1;
        rightLineSensingCorrection = 1;
      }
    }
  } else {
    rightLineSensingCorrection = 0;
    leftLineSensingCorrection = 0;
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
    delay(20);
    // left.setSpeed(-leftSpeedSetpoint * DRIVING);
    // delay(10);
    move(-rightSpeedSetpoint*1.15, -leftSpeedSetpoint*1.25);
    delay(65);
  }
  move(0,0);
}

void spin180Encoder(int dir) {
  double lastAngle = as5600_1.readAngle() / 4096.0 * 360;
  double finalAnglePosition = 8;//12.4;
  double currentPosition = 0;
  if (currentIngredient == plate) {
    TURNING_SPEED = 0.20 * MAX_SPEED;
  } else {
    TURNING_SPEED = 0.30 * MAX_SPEED;
  }
  move(dir * -TURNING_SPEED, dir * TURNING_SPEED * 1.5);
  unsigned long currentMillis = millis();
  while (finalAnglePosition - currentPosition > 0) {
    double currentAngle = as5600_1.readAngle() / 4096.0 * 360;
    double deltaA = currentAngle - lastAngle;
    if (abs(deltaA) > 180) deltaA -= sign(deltaA) * 360;
    currentPosition += deltaA/360.0 * 6.28 * WHEEL_RADIUS * dir;
    lastAngle = currentAngle;
  }
  int fr = analogRead(FR_TCRT);
  int bl = analogRead(BL_TCRT);
  while (blackTapeCounter < 2) {
    fr = analogRead(FR_TCRT);
    bl = analogRead(BL_TCRT);
    if (fr > 4000 && bl > 4000) {
      blackTapeCounter++;
    } else {
      blackTapeCounter = 0;
    }
    //equalSpeedSet(dir * 0.075 * MAX_SPEED);
    rightSpeedSetpoint = -dir * 0.19 * MAX_SPEED;
    leftSpeedSetpoint = dir * 0.19 * 1.5 * MAX_SPEED;
    move(rightSpeedSetpoint, leftSpeedSetpoint);
  }
  blackTapeCounter = 0;
  brake(true);
  delay(750);
  m.flipFacingDirection();
  lastAngle_0 = as5600_0.readAngle() / 4096.0 * 360;
  lastAngle_1 = as5600_1.readAngle() / 4096.0 * 360;
}

//-1 is right, 1 is left, 0 is error
int stationRightOrLeft(double station) {
  if (ROBOT_ID == 0) {
    if (station == TOMATOES || station == CUTTING || station == COOKTOP || station == PLATES) return -1;
    if (station == PATTIES || station == BUNS || station == POTATOES) return 1;
    return 0;
  } else if (ROBOT_ID == 1) {
    if (station == TOMATOES || station == CUTTING || station == COOKTOP || station == PLATES) return 1;
    if (station == CHEESE || station == SERVING || station == LETTUCE) return -1;
    return 0;
  }
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  //Serial.print("\r\nLast Packet Send Status:\t");
  //Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
};

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&dataReceived, incomingData, sizeof(dataReceived));

};

void setVac(bool on) {
  if (on) {
    digitalWrite(PUMP, HIGH);
    delay(1650);
    digitalWrite(PUMP, LOW);
    delay(100);
    digitalWrite(PUMP, HIGH);
    delay(100);
    digitalWrite(PUMP, LOW);
  } else {
    digitalWrite(PUMP, HIGH);
    delay(100);
    digitalWrite(PUMP, LOW);
  }
}