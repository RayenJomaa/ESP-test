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
/*__________________________________________ENCODER_______________________________________________*/
// Variables to store encoder ticks
volatile unsigned int encR_ticks = 0;
volatile unsigned int encL_ticks = 0;
// Function to increment right encoder tick count (Do not use directly)
void read_rightEncoder() { encR_ticks++; }
// Function to increment left encoder tick count (Do not use directly)
void read_leftEncoder() { encL_ticks++; }
// Function to reset right encoder tick count
void reset_encR() { encR_ticks = 0; }
// Function to reset left encoder tick count
void reset_encL() { encL_ticks = 0; }
// Function to get the current right encoder tick count (equivalent to millis())
unsigned int get_encR() { return encR_ticks; }
// Function to get the current left encoder tick count (equivalent to millis())
unsigned int get_encL() { return encL_ticks; }
void reset_encoders()
{
    reset_encL();
    reset_encR();
}

/*______________________________________SENSORS______________________________________________________*/
const byte posSensor[SENSOR_COUNT] = {15, 23, 22, 21, 19, 18, 5, 14};
bool s[8] = {0};
int dataSensor = 0b00000000;
float lastOnLineSomme = 0;
float somme = 0;
int cnt = 0;
bool onLine = 0;

bool lineColor = BLACK_LINE;

/*______________________________________PROTOTYPES___________________________________________________*/
class PID_1;

void setMotor(int LL, int RR);
void printCapteur();
/**
 * @brief detect line position and update s,datasensor,cnt,somme;
 * @note  takes into account the line color black/white using the global "lineColor" variable
 */
void readSensor();

/*______________________________________PID CLASSES___________________________________________________*/

class PID_1_cls
{
public:
    int values[8] = {-100, -50, -25, -10, 10, 25, 50, 100};
    float kp = 1.2;
    float kd = 0.2;
    int P, D;
    int lastProcess = 0;
    int lasterror = 0;

    int basespeed = 120;
    int maxspeed = 180;
    int minspeed = -50;

    void Compute()
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

        int speedm1 = basespeed - motorspeed;
        int speedm2 = basespeed + motorspeed;

        speedm1 = constrain(speedm1, minspeed, maxspeed);
        speedm2 = constrain(speedm2, minspeed, maxspeed);
        setMotor(speedm2, speedm1);
    }
};
class PID_5_cls
{
public:
    int values[8] = {-100, -50, -25, -10, 10, 25, 50, 100};
    float kp = 1;
    float kd = 0;
    int P, D;
    int lastProcess = 0;
    int lasterror = 0;

    int basespeed = 180;
    int maxspeed = 255;
    int minspeed = 0;

    int DistanceAcc = 500;

    void Compute()
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

        int speedm1 = basespeed - motorspeed;
        int speedm2 = basespeed + motorspeed;

        speedm1 = constrain(speedm1, minspeed, maxspeed);
        speedm2 = constrain(speedm2, minspeed, maxspeed);
        setMotor(speedm2, speedm1);
    }
};
class PID_4_cls
{
public:
    int values[8] = {-100, -50, -25, -10, 10, 25, 50, 100};
    float kp = 1;
    float kd = 0.2;
    int P, D;
    int lastProcess = 0;
    int lasterror = 0;

    int basespeed = 120;
    int maxspeed = 180;
    int minspeed = -50;

    void Compute()
    {

        int currenttime = millis();
        double deltaTime = (micros() - lastProcess) / 1000000.0;
        int error = 0;

        for (int i = 0; i < 8; i++)
        {
            error += s[i] * values[i];
        }

        P = error;

        if (error == 0)
        {
            if (lastOnLineSomme < 3.5)
            {
                error = -100;
            }
            else
            {
                error = 100;
            }
        }
        // D = error - lasterror;

        D = (error - lasterror) * (double)kd / deltaTime;
        lasterror = error;
        lastProcess = micros();

        int motorspeed = P * kp + D * kd;

        int speedm1 = basespeed - motorspeed;
        int speedm2 = basespeed + motorspeed;

        speedm1 = constrain(speedm1, minspeed, maxspeed);
        speedm2 = constrain(speedm2, minspeed, maxspeed);
        setMotor(speedm2, speedm1);
    }
};
class PID_2_cls
{
public:
    int error = 0;
    unsigned int lastProcess = 0;
    int lastError = 0;
    int lastOnLineError = 0;
    int P, D;
    int kp = 35;
    int kd = 5;
    int speed = 120;

    int maxspeed = 140;
    int minspeed = -120;
    void resetPID()
    {
        error = 0;
        lastProcess = 0;
        lastError = 0;
        lastOnLineError = 0;
        lastOnLineError = 0;
        kp = 35;
        kd = 5;
        speed = 120;
        maxspeed = 140;
        minspeed = -120;
    }
    void setPIDMestwi()
    {
        error = 0;
        lastProcess = 0;
        lastError = 0;
        lastOnLineError = 0;
        lastOnLineError = 0;
        kp = 30;
        kd = 2;
        speed = 120;
        maxspeed = 150;
        minspeed = -50;
    }
    void Compute(int shift = 0)
    {
        double deltaTime = (micros() - lastProcess) / 1000000.0;
        lastProcess = micros();
        int dataSensorLocal = dataSensor;
        if (shift != 0)
        {
            dataSensorLocal = dataSensorLocal >> shift;
        }
        // clang-format off
        switch (dataSensorLocal) {
            case 0b00011000: error = 0;    break;

            case 0b00110000: error = 1;    break;
            case 0b00100000: error = 2;    break;
            case 0b01100000: error = 3;    break;
            case 0b01000000: error = 4;    break;
            case 0b11000000: error = 5;    break;
            case 0b10000000: error = 6;    break;

            case 0b00001100: error = -1;   break;
            case 0b00000100: error = -2;   break;
            case 0b00000110: error = -3;   break;
            case 0b00000010: error = -4;   break;
            case 0b00000011: error = -5;   break;
            case 0b00000001: error = -6;   break;
        }
        // clang-format on
        // debugSerial->println("--> error: " + String(error));
        // displaySensor(dataSensorLocal);

        // 0b101100; 0b100100; 0b101111
        // if ((dataSensorLocal & 0b100000) &&
        //     (dataSensorLocal & 0b001111))
        // if ((dataSensorLocal & 0b10000000) &&
        //     (dataSensorLocal & 0b00011111))
        // {
        //     error = lastOnLineError;
        // }
        if (dataSensorLocal != 0b00000000)
        {
            lastOnLineError = error;
        }
        if (dataSensorLocal == 0b00000000)
        {
            error = lastOnLineError;
        }
        P = error * (double)kp;
        D = (error - lastError) * (double)kd / deltaTime;

        double rateError = error - lastError;
        lastError = error;

        double mv = P + D;
        int moveVal = (int)(P + D);
        int moveLeft = speed - moveVal;
        int moveRight = speed + moveVal;

        moveLeft = constrain(moveLeft, minspeed, maxspeed);
        moveRight = constrain(moveRight, minspeed, maxspeed);

        setMotor(moveLeft, moveRight);
    }
};
class PID_3_cls
{
public:
    int error = 0;
    unsigned int lastProcess = 0;
    int lastError = 0;
    int lastOnLineError = 6;
    int lastNonZeroError = 0;
    int P, D;
    int kp = 28;
    int kd = 0;
    int speed = 90;

    int maxspeed = 120;
    int minspeed = -90;

    void Compute(int shift = 0)
    {
        double deltaTime = (micros() - lastProcess) / 1000000.0;
        lastProcess = micros();
        int dataSensorLocal = dataSensor;
        if (shift != 0)
        {
            dataSensorLocal = dataSensorLocal >> shift;
        }
        // clang-format off
        switch (dataSensorLocal) {
            case 0b00011000: error = 0;    break;

            case 0b00110000: error = 1;    break;
            case 0b00100000: error = 2;    break;
            case 0b01100000: error = 3;    break;
            case 0b01000000: error = 4;    break;
            case 0b11000000: error = 5;    break;
            case 0b10000000: error = 6;    break;

            case 0b00001100: error = -1;   break;
            case 0b00000100: error = -2;   break;
            case 0b00000110: error = -3;   break;
            case 0b00000010: error = -4;   break;
            case 0b00000011: error = -5;   break;
            case 0b00000001: error = -6;   break;
        }
        // clang-format on
        // debugSerial->println("--> error: " + String(error));
        // displaySensor(dataSensorLocal);

        // 0b101100; 0b100100; 0b101111
        // if ((dataSensorLocal & 0b100000) &&
        //     (dataSensorLocal & 0b001111))
        // if ((dataSensorLocal & 0b10000000) &&
        //     (dataSensorLocal & 0b00011111))
        // {
        //     error = lastOnLineError;
        // }
        if (dataSensorLocal != 0b00000000)
        {
            if (error != 0)
                lastOnLineError = error;
        }
        if (dataSensorLocal == 0b00000000)
        {
            if (lastOnLineSomme < 3.5)
            {
                error = 6;
            }
            else
            {
                error = -6;
            }
            // error = constrain(lastOnLineError * 100, -6, 6);
        }
        P = error * (double)kp;
        D = (error - lastError) * (double)kd / deltaTime;

        double rateError = error - lastError;
        lastError = error;

        double mv = P + D;
        int moveVal = (int)(P + D);
        int moveLeft = speed - moveVal;
        int moveRight = speed + moveVal;

        moveLeft = constrain(moveLeft, minspeed, maxspeed);
        moveRight = constrain(moveRight, minspeed, maxspeed);
        Serial.println("LL: " + String(moveLeft) + " RR: " + String(moveRight));
        setMotor(moveLeft, moveRight);
    }
};
class FORWARD_WITH_ENCODERS_cls
{
public:
    unsigned int startEncL = get_encL();
    unsigned int startEncR = get_encR();
    unsigned long startMillis = millis();
    unsigned long lastTimer = 0;
    int powerL = 120;
    int powerR = 120;
    int speed = 120;

    bool initialized = 0;
    void init()
    {
        startEncL = get_encL();
        startEncR = get_encR();
        startMillis = millis();
        lastTimer = 0;
        initialized = 1;
    }
    void reset()
    {
        initialized = 0;
    }
    void step()
    {
        if (!initialized)
        {
            init();
        }
        const int motor_offset = 1;
        const int max_offset = 5;

        unsigned long diff_l = get_encL() - startEncL;
        unsigned long diff_r = get_encR() - startEncR;
        if (millis() - lastTimer > 0)
        {
            lastTimer = millis();
            if (diff_l > diff_r)
            {
                powerL = powerL - motor_offset;
                powerR = powerR + motor_offset;
            }
            if (diff_l < diff_r)
            {
                powerL = powerL + motor_offset;
                powerR = powerR - motor_offset;
            }
            powerL = constrain(powerL, speed - max_offset, speed + max_offset);
            powerR = constrain(powerR, speed - max_offset, speed + max_offset);
        }
        setMotor(powerL, powerR);
    }
};
/*______________________________________GLOBALs______________________________________________________*/
unsigned int lastTime = millis();
unsigned int lastEncL = 0;
unsigned int lastEncR = 0;
PID_1_cls PID_1;
PID_2_cls PID_2;
PID_3_cls PID_3;
PID_4_cls PID_4;
PID_5_cls PID_5;
FORWARD_WITH_ENCODERS_cls FORWARD_WITH_ENCODERS;

/*______________________________________SETUPs______________________________________________________*/
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

void setup()
{
    Serial.begin(115200);
    setupEncoders();
    setupLEDC();
}

/*______________________________________LOOP___________________________________________________*/
int n = -1;
// int n = 18;
// int n = 696;
//  int n = 1000;

void loop()
{
    unsigned int currentTime = millis();
    unsigned int currentEncL = get_encL();
    unsigned int currentEncR = get_encR();
    readSensor();

    /*__________________________________MAQUETTE______________________________________________*/

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
            reset_encoders();
            n++;
        }
        else
        {
            PID_1.Compute();
        }
    }
    else if (n == 1)
    {
        // Left Encoder: 151 Right Encoder: 190
        if ((currentEncR > 200) && (cnt >= 2 && abs(somme - 3.5) <= 3))
        {
            n++;
            lastTime = millis();
            reset_encoders();
        }
        else
        {
            setMotor(-100, 120);
        }
    }
    else if (n == 2)
    {
        if ((currentEncL > 80 && currentEncR > 80))
        {
            // debut ligne vertical loul
            reset_encoders();
            n++;
        }
        else
        {
            PID_1.Compute();
        }
    }

    else if (n == 3)
    {
        // Left Encoder: 885 Right Encoder: 925
        if ((currentEncL > 226 * 3 / 4) && (currentEncR > 226 * 3 / 4) && (s[0] || s[1]))
        {
            // debut ligne vertical theni
            reset_encL();
            reset_encR();
            n++;
        }
        else
        {
            s[5] = 0;
            s[6] = 0;
            s[7] = 0;
            PID_1.Compute();
        }
    }
    else if (n == 4)
    {
        // Left Encoder: 885 Right Encoder: 925
        if ((currentEncL > 226 * 0.65) && (currentEncR > 226 * 0.65))
        {
            // kammelna ezzouz khtout l verticaux
            n++;
        }
        else
        {

            s[0] = 0;
            s[1] = 0;
            s[2] = 0;
            PID_1.Compute();
        }
    }

    else if (n == 5)
    {

        if (s[4] && s[5] && s[6] && s[7])
        {
            // debut ligne horizontal 1
            reset_encL();
            reset_encR();
            while ((get_encR() < 56) && (get_encL() < 56))
            {
                setMotor(120, 120);
                readSensor();
            }
            n++;
            // fin ligne horizontal 1
        }
        else
        {
            PID_1.Compute();
        }
    }
    else if (n == 6)
    {

        if (s[0] && (s[1] && s[2] && s[3]))
        {
            // debut ligne horizental 2
            reset_encL();
            reset_encR();
            while ((get_encR() < 56) && (get_encL() < 56))
            {
                setMotor(120, 120);
                readSensor();
            }
            n++;
            // fin ligne horizontal 2
        }
        else
        {
            PID_1.Compute();
        }
    }
    else if (n == 7)
    {

        if (s[4] && s[5] && s[6] && s[7])
        {
            // debut angle droit ymin
            reset_encoders();
            while ((get_encL() < 255) || !(cnt >= 2 && abs(somme - 3.5) <= 3))
            {
                setMotor(120, -70);
                readSensor();
            }
            n++;
            reset_encoders();
            PID_2.setPIDMestwi();
            // fin angle droit ymin
        }
        else
        {
            PID_1.Compute();
        }
    }
    else if (n == 8)
    {
        if ((s[0] && s[7]) && currentEncL > 564 && currentEncR > 564) //(currentTime - lastTime > 500))
        {
            // debut partie noir (couleur inverse)
            PID_2.resetPID();
            lineColor = WHITE_LINE;
            n++;
        }
        else
        {
            PID_2.Compute();
        }
    }
    else if (n == 9)
    {
        if ((s[0] && s[7]) && (currentTime - lastTime > 500))
        {
            // ligne blanc horizontal ( pause 5 seconde ici)
            setMotor(-30, -30);
            delay(4800); // TODO: change this pause to 5 seconds
            reset_encL();
            reset_encR();
            while ((get_encR() < 56) && (get_encL() < 56))
            {
                setMotor(120, 120);
            }
            reset_encoders();
            n++;
        }
        else
        {

            PID_1.Compute();
        }
    }
    else if (n == 10)
    {
        if ((s[0] && s[7]) && (currentTime - lastTime > 500))
        {
            // fin partie noir (couleur inverse)
            lineColor = BLACK_LINE;
            reset_encoders();
            n++;
        }
        else
        {
            int x = 226 * 2 * 5;
            if (get_encL() + get_encR() < x)
            {
                // CHANGE ME
                readSensor();
                PID_5.basespeed = map(get_encL() + get_encR(), 0, x, 120, 255);
                // PID_5.Compute();
            }
            PID_5.Compute();
        }
    }
    // d5alna fl zigzag ==> taya7 l vitesse kima kenet
    else if (n == 11)
    {
        if (s[4] && s[5] && s[6] && s[7])
        {
            // debut angle droit avant zigzag
            reset_encoders();
            while ((get_encL() < 255) || cnt == 0)
            {
                readSensor();
                setMotor(120, -70);
            }
            n++;
            lastTime = millis();
            reset_encoders();
            // fin angle droit avant zigzag
        }
        else
        {
            int x1 = 226 * 2 * 2;
            int x2 = 226 * 2 * 2;
            // acceleration
            if (get_encL() + get_encR() > x1 && get_encL() + get_encR() < x2 + x1)
            {
                readSensor();
                // CHANGE ME

                PID_5.basespeed = map(get_encL() + get_encR(), x1, x1 + x2, 255, 120);
                // PID_5.Compute();
            }
            PID_5.Compute();
        }
    }
    else if (n == 12)
    {
        // Left Encoder: 221 Right Encoder: 364

        if ((s[7]) && (currentEncL > 56) && (currentEncR > 56))
        {
            // debut tour droite entree au zigzag
            reset_encoders();
            while (get_encL() < 280)
            {
                setMotor(120, -30);
            }
            reset_encoders();
            n++;
            // fin  tour droite entree au zigzag
        }
        else
            PID_1.Compute();
    }
    else if (n == 13)
    {
        if ((cnt == 0))
        {
            // Left Encoder: 218 Right Encoder: 304

            // debut angle aigu zigzag 1
            reset_encoders();

            while (get_encR() < 100 || !(cnt >= 2 && abs(somme - 3.5) <= 3))
            {
                setMotor(-100, 120);
                readSensor();
            }
            reset_encoders();
            // fin angle aigu zigzag 1
            n++;
        }
        else
        {
            PID_1.Compute();
        }
    }
    else if (n == 14)
    { // zigzag 2
        if ((cnt == 0))
        {
            // Left Encoder: 218 Right Encoder: 304

            // debut angle aigu zigzag 2
            reset_encoders();
            while (get_encL() < 100 || !(cnt >= 2 && abs(somme - 3.5) <= 3))

            {
                setMotor(120, -100);
                readSensor();
            }
            reset_encoders();
            n++;
            // fin angle aigu zigzag 2
        }
        else
        {
            PID_1.Compute();
        }
    }
    else if (n == 15)
    {
        if ((s[0]) && (currentEncL > 90) && (currentEncR > 90))
        {
            // debut angle aigu zigzag 3
            reset_encoders();
            while (cnt != 0)
            {
                readSensor();
                setMotor(80, 80);
            }

            reset_encoders();
            while (get_encR() < 100 || !(cnt >= 2 && abs(somme - 3.5) <= 3))
            {
                setMotor(-100, 120);
                readSensor();
            }
            reset_encoders();
            setMotor(0, 0);
            n++;
            // fin angle aigu zigzag 3
        }
        else
        {
            PID_2.Compute();
        }
    }

    else if (n == 16)
    {
        if ((s[6]) && (get_encL() > 50) && (get_encR() > 50))
        {
            // debut angle aigu zigzag 4 (final)
            reset_encoders();
            n++;
        }
        else
        {

            // arja3li
            
            PID_3.Compute();
        }
    }
    else if (n == 17)
    {
        if ((!s[1] && (!s[5] || !s[2]) && !s[6] && (!s[3] || !s[4])) && (get_encL() > 50) && (get_encR() > 50))
        {
            // Left Encoder: 214 Right Encoder: 200
            //  setMotor(-30, -30);
            //  delay(50);
            reset_encoders();
            while ((get_encR() < 200 * 1.6) && (get_encL() < 200 * 1.6)) //||!((abs(somme - 5) <= 2) && cnt >=2))
            {
                setMotor(120, -120);
                readSensor();
            }
            n++;
            reset_encR();
            reset_encL();
            lastTime = millis();
            // fin angle aigu zigzag 4 (final)
        }
        else
        {
            s[7] = 0;
            s[6] = 0;
            s[5] = 0;
            PID_1.Compute();
        }
    }
    else if (n == 18)
    {

        if (s[4] && s[5] && s[6] && s[7] && (currentEncL > 100 && currentEncR > 100))
        {
            // debut tour droite entree sinuset
            PID_1.basespeed = 120;
            reset_encR();
            reset_encL();
            while ((get_encL() < 292))
            {
                setMotor(120, -30);
            }
            // fin tour droite entree sinuset

            /*--------------------------------*/
            // reset_encR();
            // reset_encL();
            // while ((get_encR() < 616 - 130))
            // {
            //     setMotor(-30, 120);
            // }

            // setMotor(0, 0);
            PID_1.basespeed = 120;
            reset_encoders();
            n++;
        }
        else
        {
            // Left Encoder: 779 Right Encoder: 785
            int x_fin_accel = (779 + 785) / 3;
            int x_fin_vitesse_const = (779 + 785) * 2 / 3;
            int x_fin_decel = (779 + 785);
            // //acceleration
            // if (get_encL() + get_encR()  < x_fin_accel)
            // {
            //     readSensor();
            //     PID_1.basespeed = map(get_encL() + get_encR(), 0, x_fin_accel, 120, 180);
            // }
            // deceleration
            if (get_encL() + get_encR() > x_fin_vitesse_const && get_encL() + get_encR() < x_fin_decel)
            {
                readSensor();
                PID_1.basespeed = map(get_encL() + get_encR(), x_fin_vitesse_const, x_fin_decel, 120, 90);
            }

            if (currentTime - lastTime < 200)
            {

                s[7] = 0;
                s[6] = 0;
                s[5] = 0;
            }
            PID_1.Compute();
        }
    }
    else if (n == 19)
    {
        if ((currentEncR + currentEncL) > (1620 + 70 - 200) * 2)
        {

            // PID_2.speed = 90;

            // if (cnt == 0){
            //     reset_encoders();
            //     while ( ((get_encL()+ get_encR()) < 56*2 ) && cnt == 0)
            //     {
            //         readSensor();
            //         PID_2.Compute();
            //     }

            // }
            n++;
        }
        else
        {

            PID_3.Compute();
        }
    }
    else if (n == 20)
    {
        if (cnt == 0)
        {
            // REMINDER
            // setMotor(-30, -30);
            // delay(5);
            // debut angle droit ymin (fin sinuset)
            reset_encoders();
            while ((get_encL() < 150))
            {
                setMotor(90, -30);
                readSensor();
                if (cnt != 0)
                {
                    return;
                }
            }

            // REMINDER
            // setMotor(-30, -30);
            // delay(500);
            // reset_encoders();
            // while (cnt == 0)
            // {
            //     setMotor(90, 90);
            //     readSensor();
            // }
            reset_encoders();
            bool security = 0;
            while (cnt == 0)
            {
                if (get_encL() + get_encR() > 452)
                {
                    security = 1;
                    break;
                }
                setMotor(90, 90);
                readSensor();
            }
            if (security)
            {
                reset_encoders();
                while (cnt == 0)
                {
                    setMotor(-30, 90);
                    readSensor();
                }
            }
            n++;
            PID_2.resetPID();
            reset_encoders();

            // fin angle droit ymin (fin sinuset)
        }
        else
        {
            PID_2.Compute();
        }
    }
    else if (n == 21)
    {
        if ((s[7] && (s[4] || s[3])) && (currentEncL + currentEncR > 350))
        {
            // dora imin (fin Y1)
            reset_encoders();
            while ((get_encL() < 255))
            {
                setMotor(120, 0);
                readSensor();
            }
            reset_encoders();
            n++;
            // debut partie mestwiya (entre Y1 et Y2)
        }
        else
        {
            PID_2.Compute();
        }
    }
    else if (n == 22)
    {
        if ((cnt == 0 || (cnt >= 4 && (s[3] || s[4]))) && (currentEncL + currentEncR > 300))
        {
            // dora isar ( da5la lel Y2)
            reset_encoders();
            while ((get_encR() < 100) || !(cnt >= 2 && abs(somme - 3.5) <= 3))
            {
                setMotor(-70, 120);
                readSensor();
            }
            reset_encoders();
            n++;
            // debut Y2
        }
        else
        {
            PID_2.Compute();
        }
    }

    else if (n == 23)
    {
        if ((cnt == 0) && (currentEncL + currentEncR > 300))
        {
            setMotor(-30, -30);
            delay(5);
            // fel abyadh  mechi le drouj (fin Y2)
            reset_encoders();
            while ((get_encL() < 150))
            {
                setMotor(90, -30);
                readSensor();
            }

            reset_encoders();
            bool security = 0;
            while (cnt == 0)
            {
                if (get_encL() + get_encR() > 452)
                {
                    security = 1;
                    break;
                }
                setMotor(90, 90);
                readSensor();
            }
            if (security)
            {
                reset_encoders();
                while (cnt == 0)
                {
                    setMotor(-30, 90);
                    readSensor();
                }
            }

            // mas fel 5att drouj

            reset_encoders();
            while (cnt != 0)
            {
                setMotor(90, 90);
                readSensor();
            }
            // // fett e drouj (kolou fe labyath )
            while (cnt == 0)
            {
                setMotor(-90, 90);
                readSensor();
            }

            // rja3 le drouj
            PID_2.resetPID();
            reset_encoders();
            // REMINDER
            // setMotor(-30, -30);
            // delay(500);

            n++;
        }
        else
        {
            PID_2.Compute();
        }
    }

    else if (n == 24)
    {
        while (get_encL() + get_encR() < 1600)
        {
            readSensor();
            if (cnt == 0)
            {
                setMotor(-50, 90);
            }
            else
            {
                PID_4.Compute();
            }
        }
        // setMotor(0, 0);
        // delay(1000); //////// juste jarebna biha win youfa el 1600
        n++;
        reset_encoders();
    }
    else if (n == 25)
    {
        if (cnt >= 8)
        {
            setMotor(90, 90);
            delay(200);

            setMotor(-30, -30);
            delay(500);
            n = 100;
        }
        else
        {
            PID_2.Compute();
        }
    }

    /*__________________________________DEBUGGING_______________________________________________________*/
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
        setMotor(120, 120);
    }
    /*__________________________________TESTs_______________________________________________________*/

    // test pid
    if (n == 690)
    {
        PID_1.Compute();
    }
    // test capteurs
    if (n == 691)
    {
        printCapteur();
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
    // test PID_2
    if (n == 694)
    {
        PID_2.Compute();
    }
    // test PID_2
    if (n == 695)
    {
        PID_3.Compute();
    }
    if (n == 696)
    {
        PID_5.basespeed = 120;
        int x = 226 * 2 * 5;
        reset_encoders();
        while (get_encL() + get_encR() < x)
        {
            readSensor();
            PID_5.basespeed = map(get_encL() + get_encR(), 0, x, 125, 255);
            PID_5.Compute();
        }
        reset_encoders();
        while (get_encL() + get_encR() < x)
        {
            readSensor();
            PID_5.basespeed = map((get_encL() + get_encR()), 0, x, 255, 120);
            PID_5.Compute();
        }
        n = 100;
    }

    /*______________________________________________________________________________________________*/
}

/*______________________________________FUNCTIONS___________________________________________________*/

void readSensor()
{
    memset(&s, 0, sizeof(s)); //  s[8]= {0};
    dataSensor = 0b00000000;

    for (int x = 0; x < SENSOR_COUNT; x++)
    {
        if (digitalRead(posSensor[x]))
        {
            dataSensor = dataSensor + (0b10000000 >> x);
            s[x] = 1;
        }
    }
    if (lineColor == WHITE_LINE)
    {
        for (int i = 0; i < SENSOR_COUNT; i++)
            s[i] = !s[i];
        dataSensor = 0b11111111 - dataSensor;
    }

    onLine = 0;
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
    {
        somme = somme / cnt;
        onLine = 1;
        if (somme != 3.5)
            lastOnLineSomme = somme;
    }
}

void printCapteur()
{
    for (int i = 0; i < SENSOR_COUNT; i++)
    {
        Serial.print(s[i]);
    }
    Serial.println(" // somme: " + String(somme) + " cnt: " + String(cnt));
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