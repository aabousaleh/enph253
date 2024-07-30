#include "arm.h"

// Rudy was here

//Don't edit unless you're Michael
const float shoulderServoCoefficients[] = {
   7.13931823618486,-0.476879724058869,-0.504761425391325,0.0178792568111378,0.0573179013163423,-0.0174092462381695,-0.000137790695475793,
   -0.00283794406158651,0.00143945711157218,-1.23590156313981e-05,-4.33773775067365e-06,6.25852495562458e-05,-3.96148785958450e-05,-1.09756321446616e-06,
   4.22377191605358e-06,6.78905775309119e-08,-5.17074332241146e-07,4.13972322106801e-07,-7.65520675768900e-08,5.13228641415541e-08,-8.31112902967723e-08
};
//Don't edit unless you're Michael
const float elbowServoCoefficients[] = {
    30.2084076666509,-2.33381377971387,-4.08625074010892,0.0843909443876355,0.328225397724002,0.174236665363448,-0.00167028557454934, 
    -0.0100178423528661,-0.0121245438078198,-0.00312177482266156,1.86364342770356e-05,0.000137254035174178,	0.000272113394667061,0.000160839095421658,	
    2.80099832795895e-05,-9.72127292660652e-08,-7.21674614466160e-07,-1.97376685449567e-06,-2.03969091317573e-06,-6.39475626855562e-07,-2.15058138209155e-07
};

float calculateFifthDegreePoly(float x, float y, const float* coefficients);

float shoulderServoAngle(float x, float y);

float elbowServoAngle(float x, float y);

float shoulderServoAnglePWMConverter(float angle);

float elbowServoAnglePWMConverter(float angle);

void setupArmServos(){
    pinMode(SHOULDER_SERVO, OUTPUT);
    pinMode(ELBOW_SERVO, OUTPUT);

    ledcAttach(SHOULDER_SERVO, SERVO_PWM_FREQUENCY, SERVO_PWM_RESOLUTION);
    ledcAttach(ELBOW_SERVO, SERVO_PWM_FREQUENCY, SERVO_PWM_RESOLUTION);
}

bool moveToXY(float x, float y){
    float shoulderServoDutyCycle = shoulderServoAnglePWMConverter(shoulderServoAngle(x,y));
    float elbowServoDutyCycle = elbowServoAnglePWMConverter(elbowServoAngle(x,y));
    
    return moveRaw(shoulderServoDutyCycle, elbowServoDutyCycle);
}

bool moveRaw(float shoulderDutyCycle, float elbowDutyCycle){
    if(shoulderDutyCycle > MAX_PWM || shoulderDutyCycle < MIN_PWM)
        return false;
    if(elbowDutyCycle > MAX_PWM || elbowDutyCycle < MIN_PWM)
        return false;

    ledcWrite(SHOULDER_SERVO, (1<<SERVO_PWM_RESOLUTION) * shoulderDutyCycle);
    ledcWrite(ELBOW_SERVO, (1<<SERVO_PWM_RESOLUTION) * elbowDutyCycle);

    return true;
}

float shoulderServoAnglePWMConverter(float angle){
    float slope = (SHOULDER_SERVO_CALIBRATION_POINTS[1][1]-SHOULDER_SERVO_CALIBRATION_POINTS[0][1])/(SHOULDER_SERVO_CALIBRATION_POINTS[1][0]-SHOULDER_SERVO_CALIBRATION_POINTS[0][0]);
    return slope * (angle - SHOULDER_SERVO_CALIBRATION_POINTS[1][0]) + SHOULDER_SERVO_CALIBRATION_POINTS[1][1];
}

float elbowServoAnglePWMConverter(float angle){
    float slope = (ELBOW_SERVO_CALIBRATION_POINTS[1][1]-ELBOW_SERVO_CALIBRATION_POINTS[0][1])/(ELBOW_SERVO_CALIBRATION_POINTS[1][0]-ELBOW_SERVO_CALIBRATION_POINTS[0][0]);
    return slope * (angle - ELBOW_SERVO_CALIBRATION_POINTS[1][0]) + ELBOW_SERVO_CALIBRATION_POINTS[1][1];
}

float elbowServoAngle(float x, float y) {
    return calculateFifthDegreePoly(x, y, elbowServoCoefficients);
}

float shoulderServoAngle(float x, float y) {
    return calculateFifthDegreePoly(x, y, shoulderServoCoefficients);
}

float calculateFifthDegreePoly(float x, float y, const float * coefficients) {
    float output = 0;
    int index = 0;
    for (int power = 0; power <= POLYNOMIAL_DEGREE; power++) {
        for (int xPower = power; xPower >= 0; xPower--) {
            int yPower = power - xPower;
            output += coefficients[index] * pow(x, xPower) * pow(y, yPower);
            index++;
        }
    }
    return output;
}