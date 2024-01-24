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

#define PIN_FWD_MOTOR_L 32
#define PIN_BWD_MOTOR_L 33
#define PIN_encL_A 39

#define SENSOR_COUNT 8
#define WHITE_LINE 1
#define BLACK_LINE 0 
const byte posSensor[SENSOR_COUNT] = {15,23, 22, 21, 19, 18, 5,14};

byte lineColor = BLACK_LINE; 

void setMotor(int LL, int RR);
int readSensor();
// Variables to store encoder ticks
volatile unsigned int encR_ticks = 0;
volatile unsigned int encL_ticks = 0;

// Function to increment right encoder tick count (Do not use directly)
void read_rightEncoder()
{
    encR_ticks++;
}

// Function to increment left encoder tick count (Do not use directly)
void read_leftEncoder()
{
    encL_ticks++;
}

// Function to reset right encoder tick count
void reset_encR()
{
    encR_ticks = 0;
}

// Function to reset left encoder tick count
void reset_encL()
{
    encL_ticks = 0;
}

// Function to get the current right encoder tick count (equivalent to millis())
unsigned int get_encR()
{
    return encR_ticks;
}

// Function to get the current left encoder tick count (equivalent to millis())
unsigned int get_encL()
{
    return encL_ticks;
}

// Setup function to configure pins and attach interrupts
void setupEncoders()
{
    // Configure right encoder pins
    pinMode(PIN_encR_A, INPUT_PULLUP);
    // pinMode(PIN_encR_B, INPUT_PULLUP);

    // Configure left encoder pins
    pinMode(PIN_encL_A, INPUT_PULLUP);
    // pinMode(PIN_encL_B, INPUT_PULLUP);

    // Attach interrupts for both encoders on rising edges
    attachInterrupt(digitalPinToInterrupt(PIN_encR_A), read_rightEncoder, RISING);
    attachInterrupt(digitalPinToInterrupt(PIN_encL_A), read_leftEncoder, RISING);
}

void setupLEDC()
{
 
    // pinMode(hbridgeA,OUTPUT); pinMode(hbridgeB,OUTPUT);
    // pinMode(encoderPinA,INPUT);pinMode(encoderPinB,INPUT);
    pinMode(PIN_FWD_MOTOR_L, OUTPUT);
    ledcSetup(PWM_FWD_MOTOR_L, PWM_Freq, PWM_Res);
    ledcAttachPin(PIN_FWD_MOTOR_L, PWM_FWD_MOTOR_L);

    pinMode(PIN_BWD_MOTOR_L, OUTPUT);
    ledcSetup(PWM_BWD_MOTOR_L, PWM_Freq, PWM_Res);
    ledcAttachPin(PIN_BWD_MOTOR_L, PWM_BWD_MOTOR_L);

    pinMode(PIN_FWD_MOTOR_R, OUTPUT);
    ledcSetup(PWM_FWD_MOTOR_R, PWM_Freq, PWM_Res);
    ledcAttachPin(PIN_FWD_MOTOR_R, PWM_FWD_MOTOR_R);

    pinMode(PIN_BWD_MOTOR_R, OUTPUT);
    ledcSetup(PWM_BWD_MOTOR_R, PWM_Freq, PWM_Res);
    ledcAttachPin(PIN_BWD_MOTOR_R, PWM_BWD_MOTOR_R);

    
    setMotor(0, 0);

}
// Setup function to initialize encoders
void setup()
{
    Serial.begin(115200);
    setupEncoders();
    setupLEDC();
}

// Loop function (empty in this case)
unsigned int lastEncL = 0;
unsigned int lastEncR = 0;
int i = 0;

void loop()
{
    unsigned int currentEncL = get_encL();
    unsigned int currentEncR = get_encR();

    // setMotor(120, 0);
    // delay(2000);
    // setMotor(120, 120);
    // delay(2000);
    // setMotor(0, 120);
    // delay(2000);
    // setMotor(0, 0);
    // delay(2000);

    readSensor();
    
    if (currentEncL>226){
        Serial.println("left wheel made 1 round");
        reset_encL();
    }
    if (currentEncR>226){
        Serial.println("right wheel made 1 round");
        reset_encR();
    }
    // Serial.print("Left Encoder: "+String(currentEncL)+" Right Encoder: "+ String(currentEncR)+"\n");
}

void setMotor(int LL, int RR)
{
    LL = constrain(LL, -255, 255);
    RR = constrain(RR, -255, 255);

    if (RR > 0)
    {
        ledcWrite(PWM_FWD_MOTOR_R, RR);
        ledcWrite(PWM_BWD_MOTOR_R, 0);
    }
    else
    {
        ledcWrite(PWM_FWD_MOTOR_R, 0);
        ledcWrite(PWM_BWD_MOTOR_R, -RR);
    }

    if (LL > 0)
    {
        ledcWrite(PWM_FWD_MOTOR_L, LL);
        ledcWrite(PWM_BWD_MOTOR_L, 0);
    }
    else
    {
        ledcWrite(PWM_FWD_MOTOR_L, 0);
        ledcWrite(PWM_BWD_MOTOR_L, -LL);
    }
}


int readSensor()
{
    int dataSensorBit = 0b00000000;

    for (int x = 0; x < SENSOR_COUNT; x++)
    {
        if (digitalRead(posSensor[x]))
        {
            dataSensorBit = dataSensorBit + (0b10000000 >> x);
            Serial.print("1") ;
        }
        else{
            Serial.print("0") ;

        }
    }
    Serial.println() ;
    
    int bufBitSensor = 0b11111111;
    if (lineColor == WHITE_LINE)
    {
        bufBitSensor = 0b11111111 - dataSensorBit;
    }
    else
    {
        bufBitSensor = dataSensorBit;
    };
    
    return bufBitSensor;
}