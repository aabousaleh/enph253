//right motor pwm pins
#define PWM_RIGHT_1 25//26
#define PWM_RIGHT_2 4//32

//left motor pwm pins
#define PWM_LEFT_1 32//4
#define PWM_LEFT_2 26//25

//front right tcrt
#define FR_TCRT 37//35
//front left tcrt
#define FL_TCRT 38//34
//back right tcrt
#define BR_TCRT 34//38
//back left tcrt
#define BL_TCRT 35//37

unsigned long timeStart = 0;
unsigned long lastTime = 0;

void setup() {
  // put your setup code here, to run once:
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
  timeStart = millis();
  if (timeStart < 6000) {
    int rightSpeed = 240; //1000 max
    int leftSpeed = 240;
    int correction = 165;
    //line sensing:
    double fr = analogRead(FR_TCRT);
    double fl = analogRead(FL_TCRT);
    double br = analogRead(BR_TCRT);
    double bl = analogRead(BL_TCRT);

    int front_correction = (fr - fl);
    int back_correction = (bl - br);

    if (front_correction > 0) {
      rightSpeed -= correction;
    } 
    else if (front_correction < 0) {
      leftSpeed -= correction;
    }

    ledcWrite(PWM_RIGHT_1, rightSpeed/1000.0 * 255);
    ledcWrite(PWM_RIGHT_2, 0);
    ledcWrite(PWM_LEFT_1, leftSpeed/1000.0 * 255);
    ledcWrite(PWM_LEFT_2, 0);
  } else {
    ledcWrite(PWM_RIGHT_1, 0);
    ledcWrite(PWM_RIGHT_2, 0);
    ledcWrite(PWM_LEFT_1, 0);
    ledcWrite(PWM_LEFT_2, 0);
  }

}
