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

int basespeedm1 = 120;
int maxspeedm1 = 180;
int minspeedm1 = -50;

int basespeedm2 = 120;
int maxspeedm2 = 180;
int minspeedm2 = -50;

const byte posSensor[SENSOR_COUNT] = {15, 23, 22, 21, 19, 18, 5, 14};

bool s[8] = {0};
int dataSensor = 0b00000000;
byte lineColor = BLACK_LINE;

void setMotor(int LL, int RR);
void printCapteur(int cnt, float somme);
void PID_1();
void PID_2();

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
int cnt = 0;
float somme = 0;

// Loop function (empty in this case)
unsigned int lastTime = millis();
unsigned int lastEncL = 0;
unsigned int lastEncR = 0;

// int n = -1;
int n = 11;

void loop()
{
    unsigned int currentTime = millis();
    unsigned int currentEncL = get_encL();
    unsigned int currentEncR = get_encR();
    dataSensor = readSensor();

    if (n == -1)
    {
        setMotor(120, 120);
        if ((dataSensor != 0b00000000) && (currentTime - lastTime > 200))
        {
            n++;
            lastTime = millis();
        }
    }
    else if (n == 0)
    {
        // if (s[0] && s[1] && s[2] && s[3])
        if ((somme <= 2.5 && cnt >= 4) && (currentTime - lastTime > 200))
        {
            reset_encL();
            reset_encR();
            n++;
        }
        else
        {
            PID_1();
        }
    }

    else if (n == 1)
    {
        // Left Encoder: 151 Right Encoder: 190
        if ((currentEncR > 200) && (cnt >= 2 && abs(somme - 3.5) <= 3))
        {
            // setMotor(0,0);
            // delay(2000);
            n++;
            lastTime = millis();
            reset_encL();
            reset_encR();
        }
        else
        {   
            setMotor(-80, 120);
        }
    }
    else if (n == 2)
    {
        if ((s[7] || s[6]) && (currentEncL > 80 && currentEncR > 80))
        {
            // Left Encoder: 885 Right Encoder: 925
            // setMotor(-30,-30);
            // delay(50);
            reset_encL();
            reset_encR();
            setMotor(120, 120);
            n++;
        }
        else
        {
            PID_1();
        }
    }

    else if (n == 3)
    {
        // Left Encoder: 885 Right Encoder: 925
        if ((currentEncL > 226 * 3 / 4) && (currentEncR > 226 * 3 / 4) && (s[0] || s[1]))
        {
            reset_encL();
            reset_encR();
            n++;
        }
        else
        {

            s[5] = 0;
            s[6] = 0;
            s[7] = 0;
            PID_1();
        }
    }
    else if (n == 4)
    {
        // Left Encoder: 885 Right Encoder: 925
        if ((currentEncL > 226 * 0.65) && (currentEncR > 226 * 0.65)) // kammelna ezzouz khtout l verticaux
        {
            n++;
        }
        else
        {

            s[0] = 0;
            s[1] = 0;
            s[2] = 0;
            PID_1();
        }
    }

    else if (n == 5)
    {

        if (s[4] && s[5] && s[6] && s[7]) // wsol l khat l horizental loul
        {

            reset_encL();
            reset_encR();
            while ((get_encR() < 56) && (get_encL() < 56) )
            {
                setMotor(120, 120);
                dataSensor = readSensor();
            }
            n++;
        }
        else
        {
            PID_1();
        }
    }
    else if (n == 6)
    {

        if (s[0] && (s[1] && s[2] && s[3])) // wsol l khat l horizental etheni
        {

            reset_encL();
            reset_encR();
            while ((get_encR() < 56) && (get_encL() < 56))
            {
                setMotor(120, 120);
            }
            n++;
        }
        else
        {
            PID_1();
        }
    }
    else if (n == 7)
    {

        if (s[4] && s[5] && s[6] && s[7])
        {
            reset_encR();
            reset_encL();
            while ((get_encL() < 255) || !( cnt>=2 &&abs(somme - 3.5) <= 3))
            {
                setMotor(120, -70);
                dataSensor = readSensor();
            }
            n++;
        }
        else
        {
            PID_1();
        }
    }
    else if (n == 8)
    {
        if ((s[0] && s[7]) && (currentTime - lastTime > 500))
        {
            lineColor = WHITE_LINE;
            n++;
        }
        else
        {
            PID_1();
        }
    }
    else if (n == 9)
    {
        if ((s[0] && s[7]) && (currentTime - lastTime > 500))
        {
            setMotor(-30, -30);
            delay(500);
            reset_encL();
            reset_encR();
            while ((get_encR() < 56) && (get_encL() < 56))
            {
                setMotor(120, 120);
            }
            n++;
        }
        else
        {
            PID_1();
        }
    }
    else if (n == 10)
    {
        if ((s[0] && s[7]) && (currentTime - lastTime > 500))
        {
            lineColor = BLACK_LINE;
            n++;
        }
        else
        {
            PID_1();
        }
    }
    else if (n == 11)
    {
        if (s[4] && s[5] && s[6] && s[7])
        {
            reset_encR();
            reset_encL();
            while ((get_encL() < 255))
            {
                setMotor(120, -70);
            }
            n++;
            lastTime = millis();
            reset_encL();
            reset_encR();
        }
        else
        {
            PID_1();
        }
    }
    else if (n == 12)
    {
        // Left Encoder: 221 Right Encoder: 364
        //  d5alna le zigzag
        if ((s[7]) && (currentEncL > 56) && (currentEncR > 56))
        {
            reset_encL();
            while (get_encL() < 280)
            {
                setMotor(120, -30);
            }
            n++;
            reset_encR();
            reset_encL();
        }
        else
            PID_1();
    }
    else if (n == 13)
    {   //zigzag 1
        if ((cnt == 0))
        {
            // Left Encoder: 218 Right Encoder: 304
            reset_encR();
            reset_encL();
            while (!(cnt >= 2 && abs(somme - 3.5) <= 3))
            {
                setMotor(-100, 120);
                dataSensor = readSensor();
            }
            reset_encR();
            reset_encL();
            n++;
        }
        else
        {
            PID_1();
        }
    }
    else if (n == 14)
    {   //zigzag2
        if ((cnt == 0))
        {
            // Left Encoder: 218 Right Encoder: 304
            reset_encR();
            reset_encL();
            while (!(cnt >= 2 && abs(somme - 3.5) <= 3))
            {
                setMotor(120,-100);
                dataSensor = readSensor();
            }
            reset_encR();
            reset_encL();
            n++;
        }
        else
        {
            PID_1();
        }
    }
    else if (n == 15){
        if ((s[0]) && (currentEncL > 56) && (currentEncR > 56)){
            n++;
        }else {
            PID_1();
        }
    }
    else if (n == 16)
    {   //zigzag3
        if ((cnt == 0))
        {   
            // Left Encoder: 218 Right Encoder: 304
            reset_encR();
            reset_encL();
            while (!(cnt >= 2 && abs(somme - 3.5) <= 3))
            {
                setMotor(-100, 120);
                dataSensor = readSensor();
            }
            reset_encR();
            reset_encL();
            n++;
        }
        else
        {
            setMotor(80,80);
        }
    }
    else if (n == 17)
    {   //zigzag4 (final)
        if (s[6] && (get_encL() > 50) && (get_encR() > 50))
        {
            n++;
            reset_encL();
            reset_encR();
        }
        else
            PID_1();
    }
    else if (n == 18)
    {
        if ((!s[1] && (!s[5] || !s[2]) && !s[6]) && (get_encL() > 50) && (get_encR() > 50))
        {
            // Left Encoder: 214 Right Encoder: 200
            //  setMotor(-30, -30);
            //  delay(50);
            reset_encL();
            reset_encR();
            while ((get_encR() < 200 * 1.6) && (get_encL() < 200 * 1.6))//||!((abs(somme - 5) <= 2) && cnt >=2))
            {
                setMotor(120, -120);
                dataSensor = readSensor();
            
            }
            n++;
            reset_encR();
            reset_encL();
            lastTime = millis();
        }
        else
        {
            s[7] = 0;
            s[6] = 0;
            s[5] = 0;
            PID_1();
        }
    }
    else if (n == 19)
    {     

        if (s[4] && s[5] && s[6] && s[7] && (currentEncL > 100 && currentEncR > 100))
        {
            reset_encR();
            reset_encL();
            while ((get_encL() < 292))
            {
                setMotor(120, -30);
            }
            reset_encR();
            reset_encL();
            while ((get_encR() < 616-130))
            {
                setMotor(-30, 120);
            }
            setMotor(0,0);
            n=100;
        }
        else
        {
            if (currentTime-lastTime <800){

            s[7] = 0;
            s[6] = 0;
            s[5] = 0;
            }
            PID_1();
        }
    }
    else if (n==20){}

    /*------------------------------------------------*/
    /*------------------------------------------------*/
    /*------------------------------------------------*/

    if (n == 100)
    {
        setMotor(-30, -30);
        delay(50);
        setMotor(0, 0);
        lastTime = millis();
        while ((encL_ticks || encR_ticks))
        {
            reset_encL();
            reset_encR();
            delay(500);
        }

        n++;
    }
    if (n == 101)
    {
        Serial.print("Left Encoder: " + String(currentEncL) + " Right Encoder: " + String(currentEncR) + "\n");
    }
    if (n == 1000)
    {
        setMotor(0, 0);
        n++;
    }
    /*------------------------------------------------*/
    /*------------------------------------------------*/
    /*--------------------tests-----------------------*/

    // test pid

    if (n == 690)
    {
        PID_1();
    }
    // test capteurs
    if (n == 691)
    {
        printCapteur(cnt, somme);
        delay(500);
    }
    // test Moteurs
    if (n == 692)
    {
        setMotor(120, 120);
        delay(1000);
        setMotor(0, 0);
        delay(1000);
        setMotor(-80, -80);
        delay(1000);
        setMotor(0, 0);
        delay(2000);
    }

    // test Encodeur
    if (n == 693)
    {
        if (currentEncL > 226)
        {
            Serial.println("left wheel made 1 round");
            reset_encL();
        }
        if (currentEncR > 226)
        {
            Serial.println("right wheel made 1 round");
            reset_encR();
        }
        Serial.print("Left Encoder: " + String(currentEncL) + " Right Encoder: " + String(currentEncR) + "\n");
    }
    if (n == 694)
    {
        PID_2();
    }
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
    memset(&s, 0, sizeof(s)); //  s[8]= {0};

    for (int x = 0; x < SENSOR_COUNT; x++)
    {
        if (digitalRead(posSensor[x]))
        {
            dataSensorBit = dataSensorBit + (0b10000000 >> x);
            s[x] = 1;
        }
    }

    if (lineColor == WHITE_LINE)
    {
        for (int i = 0; i < SENSOR_COUNT; i++)
            s[i] = !s[i];
        dataSensorBit = 0b11111111 - dataSensorBit;
    }
    cnt = 0;
    somme = 0;
    for (int i = 0; i < SENSOR_COUNT; i++)
    {
        if (s[i])
        {
            cnt++;
            somme += i;
        }
    }
    if (cnt)
        somme = somme / cnt;

    return dataSensorBit;
}

void printCapteur(int cnt, float somme)
{
    for (int i = 0; i < SENSOR_COUNT; i++)
    {
        Serial.print(s[i]);
    }
    Serial.println(" // somme: " + String(somme) + " cnt: " + String(cnt));
}

int values[8] = {-100, -50, -25, -10, 10, 25, 50, 100};

float kp = 1.2;
// float ki = 0.05;
float kd = 0.2;

int P, D;
int I = 0;

int lastProcess = 0;
int lasterror = 0;

void PID_1()
{

    int currenttime = millis();
    double deltaTime = (micros() - lastProcess) / 1000000.0;
    int error = 0;

    for (int i = 0; i < 8; i++)
    {
        error += s[i] * values[i];
    }

    P = error;

    // D = error - lasterror;

    D = (error - lasterror) * (double)kd / deltaTime;
    lasterror = error;
    lastProcess = micros();

    int motorspeed = P * kp + D * kd;

    int speedm1 = basespeedm1 - motorspeed;
    int speedm2 = basespeedm2 + motorspeed;

    speedm1 = constrain(speedm1, minspeedm1, maxspeedm1);
    speedm2 = constrain(speedm2, minspeedm2, maxspeedm2);
    setMotor(speedm2, speedm1);
}

int lastOnLineError = 0;
void PID_2()
{

    int currenttime = millis();
    double deltaTime = (micros() - lastProcess) / 1000000.0;
    int error = 0;

    for (int i = 0; i < 8; i++)
    {
        error += s[i] * values[i];
    }
    if (error == 0 && dataSensor == 0)
    {
        error = lastOnLineError * (-2);
    }
    else
    {
        lastOnLineError = error;
    }
    P = error;

    // D = error - lasterror;

    D = (error - lasterror) * (double)kd / deltaTime;
    lasterror = error;
    lastProcess = micros();

    int motorspeed = P * kp + D * kd;

    int speedm1 = basespeedm1 - motorspeed;
    int speedm2 = basespeedm2 + motorspeed;

    speedm1 = constrain(speedm1, minspeedm1, maxspeedm1);
    speedm2 = constrain(speedm2, minspeedm2, maxspeedm2);
    setMotor(speedm2, speedm1);
}
