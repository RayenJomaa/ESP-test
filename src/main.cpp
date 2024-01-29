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
/*______________________________________GLOBALs______________________________________________________*/
unsigned int lastTime = millis();
unsigned int lastEncL = 0;
unsigned int lastEncR = 0;
PID_1_cls PID_1;

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
// int n = 690;
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
            setMotor(-80, 120);
        }
    }
    else if (n == 2)
    {
        if ((s[7] || s[6]) && (currentEncL > 80 && currentEncR > 80))
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

            reset_encR();
            reset_encL();
            while ((get_encL() < 255) || !(cnt >= 2 && abs(somme - 3.5) <= 3))
            {
                setMotor(120, -70);
                readSensor();
            }
            n++;
            // fin angle droit ymin
        }
        else
        {
            PID_1.Compute();
        }
    }
    else if (n == 8)
    {
        if ((s[0] && s[7]) && (currentTime - lastTime > 500))
        {
            // debut partie noir (couleur inverse)
            lineColor = WHITE_LINE;
            n++;
        }
        else
        {
            PID_1.Compute();
        }
    }
    else if (n == 9)
    {
        if ((s[0] && s[7]) && (currentTime - lastTime > 500))
        {
            // ligne blanc horizontal ( pause 5 seconde ici)
            setMotor(-30, -30);
            delay(500); // TODO: change this pause to 5 seconds
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
            PID_1.Compute();
        }
    }
    else if (n == 10)
    {
        if ((s[0] && s[7]) && (currentTime - lastTime > 500))
        {
            // fin partie noir (couleur inverse)
            lineColor = BLACK_LINE;
            n++;
        }
        else
        {
            PID_1.Compute();
        }
    }
    else if (n == 11)
    {
        if (s[4] && s[5] && s[6] && s[7])
        {
            // debut angle droit avant zigzag
            reset_encoders();
            while ((get_encL() < 255))
            {
                setMotor(120, -70);
            }
            n++;
            lastTime = millis();
            reset_encoders();
            // fin angle droit avant zigzag
        }
        else
        {
            PID_1.Compute();
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
            while ( get_encR()<100 || !(cnt >= 2 && abs(somme - 3.5) <= 3))
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
            while ( get_encL()<100 || !(cnt >= 2 && abs(somme - 3.5) <= 3))
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
        if ((s[0]) && (currentEncL > 56) && (currentEncR > 56))
        {
            // debut angle aigu zigzag 3
            reset_encoders();
            while ( cnt!=0 ){
                readSensor();
                setMotor(80,80);
            }
            
            reset_encoders();
            while ( get_encR()<100 || !(cnt >= 2 && abs(somme - 3.5) <= 3))
            {
                setMotor(-100, 120);
                readSensor();
            }
            reset_encoders();
            setMotor(0,0);
            n++;
            // fin angle aigu zigzag 3

        }
        else
        {
            PID_1.Compute();
        }
    }
    
    else if (n == 16)
    { 
        if (s[6] && (get_encL() > 50) && (get_encR() > 50))
        {   
            // debut angle aigu zigzag 4 (final)
            reset_encoders();
            n++;
        }
        else
            PID_1.Compute();
    }
    else if (n == 17)
    {
        if ((!s[1] && (!s[5] || !s[2]) && !s[6]) && (get_encL() > 50) && (get_encR() > 50))
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
            reset_encR();
            reset_encL();
            while ((get_encL() < 292))
            {
                setMotor(120, -30);
            }
            reset_encR();
            reset_encL();
            while ((get_encR() < 616 - 130))
            {
                setMotor(-30, 120);
            }
            setMotor(0, 0);
            n = 100;
        }
        else
        {
            if (currentTime - lastTime < 200)
            {

                s[7] = 0;
                s[6] = 0;
                s[5] = 0;
            }
            PID_1.Compute();
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
        setMotor(0, 0);
        n++;
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
    if (cnt){
        somme = somme / cnt;
        onLine = 1;
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