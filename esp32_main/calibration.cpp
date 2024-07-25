#include "calibration.h"


float RIGHT_MOTOR_CURVE[41] {
    -0.999920643,
    -0.961701935,
    -0.953359249,
    -0.938619819,
    -0.928886685,
    -0.912637583,
    -0.897382146,
    -0.875054866,
    -0.86424871,
    -0.838981442,
    -0.807953802,
    -0.774303979,
    -0.722418863,
    -0.673672316,
    -0.586031582,
    -0.498629296,
    -0.391561782,
    -0.391243976,
    -0.31345598,
    -0.196416503,
    0,
    0.258154357,
    0.301656539,
    0.376623771,
    0.452862606,
    0.540860635,
    0.600452826,
    0.666203311,
    0.720988547,
    0.770291349,
    0.798458737,
    0.818005121,
    0.852369912,
    0.879861821,
    0.890667977,
    0.905168957,
    0.923324514,
    0.929720878,
    0.933415702,
    0.949902874,
    0.995788789
};

float LEFT_MOTOR_CURVE[41] {
   -0.990107773,
    -0.944777733,
    -0.936991302,
    -0.925350574,
    -0.918438203,
    -0.896031187,
    -0.891224232,
    -0.86178562,
    -0.844543613,
    -0.811926564,
    -0.780024863,
    -0.730562968,
    -0.662150812,
    -0.607127507,
    -0.51908961,
    -0.392356486,
    -0.231615866,
    -0.003257794,
    0.000159093,
    0,
    0,
    0.105200548,
    0.201859449,
    0.330976305,
    0.434825894,
    0.520956576,
    0.590560599,
    0.658893398,
    0.71510895,
    0.759564549,
    0.796432678,
    0.817091192,
    0.858567695,
    0.880179627,
    0.893965259,
    0.901593357,
    0.920940781,
    0.934408988,
    0.953677055,
    0.95602092,
    1
};

float getCalibratedPWM(float PWM, char channel);

void setupMotors(){
    ledcAttach(PWM_LEFT_1, PWM_FREQ, PWM_RESOLUTION); //FWD
    ledcAttach(PWM_LEFT_2, PWM_FREQ, PWM_RESOLUTION); //REV
    ledcAttach(PWM_RIGHT_1, PWM_FREQ, PWM_RESOLUTION); //FWD
    ledcAttach(PWM_RIGHT_2, PWM_FREQ, PWM_RESOLUTION); //REV

    ledcWrite(PWM_LEFT_1, LOW);
    ledcWrite(PWM_LEFT_2, LOW);
    ledcWrite(PWM_RIGHT_1, LOW);
    ledcWrite(PWM_RIGHT_2, LOW);
}

bool sendPWM(float dutyCycle, char channel) {
    int motorPin1;
    int motorPin2;
    if(channel == MOTOR_LEFT){
        motorPin1 = PWM_LEFT_1;
        motorPin2 = PWM_LEFT_2;
    } else if(channel == MOTOR_RIGHT){
        motorPin1 = PWM_RIGHT_1;
        motorPin2 = PWM_RIGHT_2;
    } else
        return false;

    /*if(dutyCycle < 0){
      ledcWrite(motorPin1, 0);
      ledcWrite(motorPin2, (1<<PWM_RESOLUTION) * -dutyCycle);
    } else if(dutyCycle > 0){
        ledcWrite(motorPin1, (1<<PWM_RESOLUTION) * dutyCycle);
        ledcWrite(motorPin2, 0);
    } else {
      ledcWrite(motorPin1, 0);
      ledcWrite(motorPin2, 0);
    }
    return true;*/

    if(dutyCycle > 1){
        ledcWrite(motorPin1, (1<<PWM_RESOLUTION));
    } else if(dutyCycle < 1){
        ledcWrite(motorPin2, (1<<PWM_RESOLUTION));
    }

    if(abs(dutyCycle) < 0.00001){
        ledcWrite(motorPin1, 0);
        ledcWrite(motorPin2, 0);
        return true;
    }

    float calibratedDutyCycle = getCalibratedPWM(dutyCycle, channel);

    if(calibratedDutyCycle < 0){
      ledcWrite(motorPin1, 0);
      ledcWrite(motorPin2, (1<<PWM_RESOLUTION) * -calibratedDutyCycle);
    } else if(calibratedDutyCycle > 0){
        ledcWrite(motorPin1, (1<<PWM_RESOLUTION) * calibratedDutyCycle);
        ledcWrite(motorPin2, 0);
    } else {
      ledcWrite(motorPin1, 0);
      ledcWrite(motorPin2, 0);
    }
    return true;
}

float getCalibratedPWM(float PWM, char channel) {
    float * data;
    if(channel == MOTOR_LEFT)
        data = LEFT_MOTOR_CURVE;
    if(channel == MOTOR_RIGHT)
        data = RIGHT_MOTOR_CURVE;
    
    float initialPWM1;
    float initialPWM2;
    float finalPWM1;
    float finalPWM2;

    for(int i = 0; i < NUMBER_OF_CALIBRATION_POINTS + 1; i++){
        if(i == NUMBER_OF_CALIBRATION_POINTS){
            return 1;
        }
        if(PWM < data[i]){
            if(i == 0){
                return -1;
            }

            float dutyCycleStep = 2.0 / (NUMBER_OF_CALIBRATION_POINTS - 1);
            finalPWM1 = ((i - 1) * dutyCycleStep) - 1; //y-axis
            finalPWM2 = finalPWM1 - dutyCycleStep;

            initialPWM1 = data[i - 1]; //x-axis
            initialPWM2 = data[i];

            float slope = (finalPWM1 - finalPWM2) / (initialPWM2 - initialPWM1);

            return slope * (PWM - initialPWM1) + finalPWM1;
        }
    }
    return 0;
}