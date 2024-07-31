/**
 * 
 */

#define FORWARD 1
#define BACKWARD 0

//right motor pwm pins
int PWM_RIGHT_1 = 25;  //forward: 26       back: 25
int PWM_RIGHT_2 = 4;   //forward: 32       back: 4

//left motor pwm pins
int PWM_LEFT_1 = 32;   //forward: 4        back: 32
int PWM_LEFT_2 = 26;   //forward: 25       back: 26

//front right tcrt
int FR_TCRT = 37;      //forward: 35       back: 37
//front left tcrt
int FL_TCRT = 38;      //forward: 34       back: 38
//back right tcrt
int BR_TCRT = 34;      //forward: 38       back: 34
//back left tcrt
int BL_TCRT = 35;      //forward: 37       back: 35

unsigned long timeStart = 0;
unsigned long lastTime = 0;

const double MAX_SPEED = 1000.0;
int rightSpeed = 240;   //max: 1000
int leftSpeed = 240;
int drivingTime = 6000;
int correction = 65;   //forward: 65       back: 150

void setup() {
  Serial.begin(115200);
  // put your setup code here, to run once:
  physicalDrivingDirection(FORWARD);

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
  if (timeStart < drivingTime) {
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

    setSpeed(rightSpeed, leftSpeed);

    // printTCRT(fr, fl, br, bl);
  } else {
    stop();
  }
}

/**
 * Sets speeds of right motor (maps speed to PWM).
 * 
 * @param: 'rightSpeed' - desired speed of right motor.
 */
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

/**
 * Sets speeds of left motor (maps speed to PWM).
 * 
 * @param: 'leftSpeed' - desired speed of left motor.
 */
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

/**
 * Sets speeds of right and left motors (maps speed to PWM).
 * 
 * @param: 'rightSpeed' - desired speed of right motor.
 * @param: 'leftSpeed' - desired speed of left motor.
 */
void setSpeed(int rightSpeed, int leftSpeed) {
  setRightSpeed(rightSpeed);
  setLeftSpeed(leftSpeed);
}

/**
 * Sets both motors to 0 PWM (no braking).
 */
void stop() {
  ledcWrite(PWM_RIGHT_1, 0);
  ledcWrite(PWM_RIGHT_2, 0);
  ledcWrite(PWM_LEFT_1, 0);
  ledcWrite(PWM_LEFT_2, 0);
}

/**
 * Sets motor and TCRT pins for physical forward or backward direction.
 * 
 * @param: 'direction' - desired physical direction of robot (1 is forward, 0 is backward).
 */
void physicalDrivingDirection(bool direction) {
  if (direction) {
    int PWM_RIGHT_1 = 26;  //forward: 26       back: 25
    int PWM_RIGHT_2 = 32;   //forward: 32       back: 4

    //left motor pwm pins
    int PWM_LEFT_1 = 4;   //forward: 4        back: 32
    int PWM_LEFT_2 = 25;   //forward: 25       back: 26

    //front right tcrt
    int FR_TCRT = 35;      //forward: 35       back: 37
    //front left tcrt
    int FL_TCRT = 34;      //forward: 34       back: 38
    //back right tcrt
    int BR_TCRT = 38;      //forward: 38       back: 34
    //back left tcrt
    int BL_TCRT = 37;      //forward: 37       back: 35
  } else {
    int PWM_RIGHT_1 = 25;  //forward: 26       back: 25
    int PWM_RIGHT_2 = 4;   //forward: 32       back: 4

    //left motor pwm pins
    int PWM_LEFT_1 = 32;   //forward: 4        back: 32
    int PWM_LEFT_2 = 26;   //forward: 25       back: 26

    //front right tcrt
    int FR_TCRT = 37;      //forward: 35       back: 37
    //front left tcrt
    int FL_TCRT = 38;      //forward: 34       back: 38
    //back right tcrt
    int BR_TCRT = 34;      //forward: 38       back: 34
    //back left tcrt
    int BL_TCRT = 35;      //forward: 37       back: 35
  }
}

/**
 * Prints raw analog read values of TCRTs.
 * 
 * @param: '_fr' - front right TCRT output.
 * @param: '_fl' - front left TCRT output.
 * @param: '_br' - back right TCRT output.
 * @param: '_bl' - back left TCRT output.
 */
void printTCRT(double _fr, double _fl, double _br, double _bl) {
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
