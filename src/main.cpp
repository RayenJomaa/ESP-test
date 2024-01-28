#include "Arduino.h"

#define PWM_Res 8
#define PWM_Freq 1200

#define PWM_FWD_MOTOR_L 0
#define PWM_BWD_MOTOR_L 1
#define PWM_FWD_MOTOR_R 2
#define PWM_BWD_MOTOR_R 3

#define PIN_FWD_MOTOR_R 26
#define PIN_BWD_MOTOR_R 25
#define PIN_encR_A 35

#define PIN_FWD_MOTOR_L 33
#define PIN_BWD_MOTOR_L 32
#define PIN_encL_A 39

#define SENSOR_COUNT 8
#define WHITE_LINE 1
#define BLACK_LINE 0

const byte posSensor[SENSOR_COUNT] = {15, 23, 22, 21, 19, 18, 5, 14};

bool s[8] = {0};
int dataSensor = 0b00000000;
float somme = 0;
int cnt = 0;

bool lineColor = BLACK_LINE;


void setMotor(int LL, int RR);
void printCapteur(int cnt, float somme);
void PID_1();
void PID_2();
int readSensor();