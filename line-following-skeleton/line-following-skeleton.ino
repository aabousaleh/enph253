//right motor pwm pins
#define PWM_RIGHT_1 26
#define PWM_RIGHT_2 32

//left motor pwm pins
#define PWM_LEFT_1 4
#define PWM_LEFT_2 25

//front right tcrt
#define FR_TCRT 35
//front left tcrt
#define FL_TCRT 34
//back right tcrt
#define BR_TCRT 38
//back left tcrt
#define BL_TCRT 37

#define RS_TCRT 5
//left station-sensing tcrt
#define LS_TCRT 7

unsigned long timeStart = 0;
unsigned long lastTime = 0;

void setup() {
  Serial.begin(115200);
  // put your setup code here, to run once:
  pinMode(FR_TCRT, INPUT);
  pinMode(FL_TCRT, INPUT);
  pinMode(BR_TCRT, INPUT);
  pinMode(BL_TCRT, INPUT);

  pinMode(RS_TCRT, INPUT);
  pinMode(LS_TCRT, INPUT);

  pinMode(33, INPUT);

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
  // timeStart = millis();
  // if (timeStart < 5000) {
  //   int rightSpeed = 200; //1000 max
  //   int leftSpeed = -250;
  //   int correction = 120;
    //line sensing:
    double fr = analogRead(FR_TCRT);
    double fl = analogRead(FL_TCRT);
    double br = analogRead(BR_TCRT);
    double bl = analogRead(BL_TCRT);

    int rs = digitalRead(RS_TCRT);
    int ls = digitalRead(LS_TCRT);

    double pot = analogRead(33);

  //   int front_correction = (fr - fl);
  //   int back_correction = (bl - br);

  //   // if (front_correction > 0) {
  //   //   rightSpeed -= correction;
  //   // } 
  //   // else if (front_correction < 0) {
  //   //   leftSpeed -= correction;
  //   // }

  //   ledcWrite(PWM_RIGHT_1, rightSpeed/1000.0 * 255);
  //   ledcWrite(PWM_RIGHT_2, 0);
  //   ledcWrite(PWM_LEFT_2, -leftSpeed/1000.0 * 255);
  //   ledcWrite(PWM_LEFT_1, 0);
  // } else {
  //   ledcWrite(PWM_RIGHT_1, 0);
  //   ledcWrite(PWM_RIGHT_2, 0);
  //   ledcWrite(PWM_LEFT_1, 0);
  //   ledcWrite(PWM_LEFT_2, 0);
  // }
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
  Serial.print(bl);
  Serial.print(",");
  Serial.print("RightStation:");
  Serial.print(rs);
  Serial.print(",");
  Serial.print("LeftStation:");
  Serial.print(ls);
  Serial.print(",");
  Serial.print("pot:");
  Serial.println(pot);
}
