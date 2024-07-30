/**
 * Line following seems consistent across different battery voltages (not tested extensively just a vibe check)
 * Started tuning spin function with battery at 16.1 V, ended at 15.8 V
 * Need right wheel to turn more to have centre of rotation be in the middle for CCW turn
 * Need to punch in at max speed at the start of each turn to overcome wonky motor response
 */

#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

WiFiServer server(80);
WiFiClient RemoteClient;

const char* ssid = "Kirby ESP32 Testing Server";
const char* password = "KirbySucc";

//right motor pwm pins
#define PWM_RIGHT_1 25  //forward: 26       back: 25
#define PWM_RIGHT_2 4   //forward: 32       back: 4

//left motor pwm pins
#define PWM_LEFT_1 32   //forward: 4        back: 32
#define PWM_LEFT_2 26   //forward: 25       back: 26

//front right tcrt
#define FR_TCRT 37      //forward: 35       back: 37
//front left tcrt
#define FL_TCRT 38      //forward: 34       back: 38
//back right tcrt
#define BR_TCRT 34      //forward: 38       back: 34
//back left tcrt
#define BL_TCRT 35      //forward: 37       back: 35

unsigned long timeStart = 0;
unsigned long lastTime = 0;

const double MAX_SPEED = 1000.0;
double TURNING_SPEED = 0.045 * MAX_SPEED;
int TURNING_DELAY = 925;    //CCW turn: 910
int rightMotorOffset = 10;  //CW turn: 16
int leftMotorOffset = 0;  //CCW turn: 10

void setup() {
  Serial.begin(115200);
  // put your setup code here, to run once:

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    delay(5000);
    ESP.restart();
  }

  ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";

      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });

  ArduinoOTA.begin();

  server.begin();

  pinMode(FR_TCRT, INPUT);
  pinMode(FL_TCRT, INPUT);
  pinMode(BR_TCRT, INPUT);
  pinMode(BL_TCRT, INPUT);

  pinMode(PWM_RIGHT_1, OUTPUT);
  pinMode(PWM_RIGHT_2, OUTPUT);
  pinMode(PWM_LEFT_1, OUTPUT);
  pinMode(PWM_LEFT_2, OUTPUT);

  ledcAttach(PWM_RIGHT_1, 250, 8);
  ledcAttach(PWM_RIGHT_2, 250, 8);
  ledcAttach(PWM_LEFT_1, 250, 8);
  ledcAttach(PWM_LEFT_2, 250, 8);

  delay(3000);
}

void loop() {
  ArduinoOTA.handle();

  if (server.hasClient()) {
    if (RemoteClient.connected()) {
      Serial.println("Connection rejected");
      server.available().stop();
    } else {
      Serial.println("Connection accepted");
      RemoteClient = server.available();
    }
  }

  if (RemoteClient.available() > 0) {
    rightMotorOffset = RemoteClient.parseInt();
  }

  RemoteClient.println(rightMotorOffset);

  setSpeed(MAX_SPEED, -MAX_SPEED);
  delay(50);
  setSpeed(TURNING_SPEED + rightMotorOffset, -TURNING_SPEED);
  delay(TURNING_DELAY);     //CW turn: 900          CCW turn: 910
  setSpeed(-MAX_SPEED, MAX_SPEED);
  delay(10);
  stop();
  delay(3000);

  // timeStart = millis();
  // if (timeStart < 6000) {
  //   int rightSpeed = 240; //1000 max
  //   int leftSpeed = 240;
  //   int correction = 150; //65 forward, 150 back
  //   //line sensing:
  //   double fr = analogRead(FR_TCRT);
  //   double fl = analogRead(FL_TCRT);
  //   double br = analogRead(BR_TCRT);
  //   double bl = analogRead(BL_TCRT);

  //   int front_correction = (fr - fl);
  //   int back_correction = (bl - br);

  //   if (front_correction > 0) {
  //     rightSpeed -= correction;
  //   }
  //   else if (front_correction < 0) {
  //     leftSpeed -= correction;
  //   }

  //   setSpeed(rightSpeed, leftSpeed);

  // } else {
  //   stop();
  // }
  // // printTCRT(fr, fl, br, bl);
}

void setRightSpeed(int rightSpeed) {
  int pwm = (abs(rightSpeed) < MAX_SPEED) ? (rightSpeed / MAX_SPEED) * 255.0 : 255 * rightSpeed / abs(rightSpeed);
  if (pwm >= 0) {
    ledcWrite(PWM_RIGHT_1, pwm);
    ledcWrite(PWM_RIGHT_2, 0);
  } else {
    ledcWrite(PWM_RIGHT_1, 0);
    ledcWrite(PWM_RIGHT_2, -pwm);
  }
}

void setLeftSpeed(int leftSpeed) {
  int pwm = (abs(leftSpeed) < MAX_SPEED) ? (leftSpeed / MAX_SPEED) * 255.0 : 255 * leftSpeed / abs(leftSpeed);
  if (pwm >= 0) {
    ledcWrite(PWM_LEFT_1, pwm);
    ledcWrite(PWM_LEFT_2, 0);
  } else {
    ledcWrite(PWM_LEFT_1, 0);
    ledcWrite(PWM_LEFT_2, -pwm);
  }
}

void setSpeed(int rightSpeed, int leftSpeed) {
  setRightSpeed(rightSpeed);
  setLeftSpeed(leftSpeed);
}

void stop() {
  ledcWrite(PWM_RIGHT_1, 0);
  ledcWrite(PWM_RIGHT_2, 0);
  ledcWrite(PWM_LEFT_1, 0);
  ledcWrite(PWM_LEFT_2, 0);
}

//dir: 1 CW, -1 CCW
void spin180(int dir) {
  setSpeed(dir * -MAX_SPEED, dir * MAX_SPEED);
  delay(10);
  setSpeed(dir * -TURNING_SPEED, dir * TURNING_SPEED);
  delay(TURNING_DELAY);
  spinBrake(dir);
}

void spinBrake(int dir) {
  setSpeed(dir * TURNING_SPEED / 3.0, dir * -TURNING_SPEED / 3.0);
  delay(10);
  stop();
}

void printTRCT(double _fr, double _fl, double _br, double _bl) {
  Serial.print("Frontright:");
  Serial.print(_fr);
  Serial.print(",");
  Serial.print("Frontleft:");
  Serial.print(_fl);
  Serial.print(",");
  Serial.print("Backright:");
  Serial.print(_br);
  Serial.print(",");
  Serial.print("Backleft:");
  Serial.println(_bl);
}
