/******************************************************************

                        s c o u t 2 5

                                                   қuran july 2025
******************************************************************/

#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#define FASTLED_ALL_PINS_HARDWARE_SPI
#include <FastLED.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <PS4Controller.h>
#include <BluetoothSerial.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

#define TRUE                            true
#define FALSE                           false
#define H                               HIGH
#define L                               LOW

#define WAIT_ONE_SEC                    10000
#define WAIT_250_MSEC                   2500
#define WAIT_10_MSEC                    100 

#define ON_BOARD_LED                    5                  // LoLin32 
#define BATTERY_LEVEL                   A3                 // GPIO 39
#define REFV                            685.0              // factor

#define MFS                             0x1e               // magnetic field sensor

#define WHEEL_L                         2
#define WHEEL_R                         A4
#define WHEEL_L_DIRECTION               15 
#define WHEEL_R_DIRECTION               A5

#define EEPROM_SIZE                     100
#define EEPROM_ADDR                     0
#define EEPROM_SSID_ADDR                0
#define EEPROM_PASSWORD_ADDR            40
#define EEPROM_MOTOR_SYS_ADDR           80
#define EEPROM_PS4_ADDR                 82

#define TRIG_PIN                        25  
#define ECHO_PIN                        26
#define TEST_PIN_RX2                    16
#define TEST_PIN_TX2                    17

#define SCREEN_WIDTH                    128                // OLED display width, in pixels
#define SCREEN_HEIGHT                   64                 // OLED display height, in pixels
#define OLED                            0x3c

#define NUM_LEDS                        4
#define DATA_PIN                        23
#define CLOCK_PIN                       18

#define LDR1                            35 
#define LDR2                            34 
#define LDR3                            36 
#define LDR4                            39 

#define PS4_GREEN                       60
#define PS4_RED                         61
#define PS4_BLUE                        62

#define MODE_PS4                        0  
#define MODE_MENU                       1
#define MODE_BT                         2


hw_timer_t *timer = NULL;
void IRAM_ATTR myTimer(void);
volatile int watch = 0; 
volatile int oneSecFlag;
volatile int qSecFlag;
volatile int tenMSecFlag;

BluetoothSerial BT;  // SPP „Serial Port Profile“
static const char* BT_NAME = "scout25";

volatile int vL, vR, LDir, RDir;
volatile int impulsCntL;
volatile int impulsCntR;
volatile int impulsFlagL;
volatile int impulsFlagR;
float correctionRL; 
volatile int countR = 0;
volatile int prell = 0;


volatile float batteryLevel = 0.;
volatile int motorSys = 0;
String motorSysFromEEPROM = "";
String pskey = "";
String ssidWord = "";
String password = "";

int xMin = 32767, xMax = -32768;
int yMin = 32767, yMax = -32768;
int16_t  x, y, z;
int mfcInitialized = FALSE;
int isCalibrated = FALSE;
const uint8_t impulsR = 14; // sollten vertauscht werden! 
const uint8_t impulsL = 27;
float angle = 0.;
int speed, diff;
int speedL, speedR; 
int dirL = 1, dirR = 1;
int connected = FALSE;
int speedMin;
float distance;
Adafruit_SSD1306 oled(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire);
char text[20];
volatile static unsigned char ramp = 0;

int mode;
int ps;


CRGB leds[NUM_LEDS];

void preSet(void);
void storeStr2EEPROM(String word, int address);
void storeVal2EEPROM(int x, int address);

String readFromEEPROM(int address);

void rototateByCompass(int degree); 
void drive(int left, int right);
void driveC(int l, int r);

void onBoardLedOn(void);
void onBoardLedOff(void);

int  initMFC(void);
void preppareMfsCalibrationSystem(void);
int  calibrateMFC(void);
void readRawMFC(int16_t* x, int16_t* y, int16_t* z);
float getMFC_Angle();

void impuls_R_isr(void);
void impuls_L_isr(void);

void scanI2CBus();   // for tests
void printData(void);
void printImpulse(void);



void setup() 
{
    Serial.begin(115200);
    pinMode(ON_BOARD_LED, OUTPUT);
    pinMode(WHEEL_L, OUTPUT);
    pinMode(WHEEL_R, OUTPUT);
    pinMode(WHEEL_L_DIRECTION, OUTPUT);
    pinMode(WHEEL_R_DIRECTION, OUTPUT);
    pinMode(BATTERY_LEVEL, INPUT);
    pinMode(ECHO_PIN, INPUT_PULLUP);  
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(TEST_PIN_RX2, INPUT_PULLUP);
    pinMode(TEST_PIN_TX2, INPUT_PULLUP);


    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);  // deactiviere die Brownout Detection !!!

    /* restore all eeprom variables: */
    if (!EEPROM.begin(EEPROM_SIZE)) {
        Serial.println("EEPROM initialisieren fehlgeschlagen!");
        return;
    }

// Werte noch selbst speichern - das komt dann wieder weg! 
    preSet(); // store setup to EEPROM um andere Daten ins E2Prom zu schreiben


    motorSysFromEEPROM = readFromEEPROM(EEPROM_MOTOR_SYS_ADDR);
    motorSys = (char)motorSysFromEEPROM[0] - '0';
    ssidWord = readFromEEPROM(EEPROM_SSID_ADDR);
    password = readFromEEPROM(EEPROM_PASSWORD_ADDR);
    pskey = readFromEEPROM(EEPROM_PS4_ADDR);

    ps = (char) pskey[0]; 



    batteryLevel = analogRead(BATTERY_LEVEL) / REFV;

    Wire.begin(21, 22); // i2c wird hier vorbereitet

    if (digitalRead(TEST_PIN_RX2) == LOW)
    {
        mode = MODE_PS4;
    }
    else if (digitalRead(TEST_PIN_TX2) == LOW)
    {
        mode = MODE_MENU;
    }
    else 
    {
        mode = MODE_BT;
    }

    

    timer = timerBegin(0, 80, true);
    timerAttachInterrupt(timer, &myTimer, true);
    timerAlarmWrite(timer, 100, true);  // 0.1 msec -> 100
    timerAlarmEnable(timer);

    attachInterrupt(digitalPinToInterrupt(impulsR), impuls_R_isr, FALLING);
    attachInterrupt(digitalPinToInterrupt(impulsL), impuls_L_isr, FALLING);
    
    oneSecFlag = FALSE; 
    qSecFlag = FALSE;
    tenMSecFlag = FALSE;
    impulsFlagL = FALSE;  
    impulsFlagR = FALSE;  
    impulsCntL = impulsCntR = 0;
 
    preppareMfsCalibrationSystem();
    
    drive(0, 0);

    onBoardLedOff();

    sei(); // start all interrupts!  especially printf, impulsCount and timer need this ... 

    watch = 100; while (watch); // 100 ms wait time
    
    printf("\n______________________________________________________________________________________\n");
    printf("battery: %1.3f\n", batteryLevel);
    printf("motorSystem: %d\n", motorSys);
    printf("ssid: %s\n", ssidWord);
    printf("password: %s\n", password);
    printf("magneticfield-sesor: %d\n", initMFC());
    printf("ps: %d\n", ps);
    printf("______________________________________________________________________________________\n");
    printf("start!\n");
    
    oled.begin(SSD1306_SWITCHCAPVCC, OLED);
    oled.clearDisplay();
    oled.setTextSize(2);
    oled.setTextColor(WHITE);
    oled.setCursor(0, 0);
    oled.print("calibrate:");
    // oled.drawRect(10, 25, 40, 15, WHITE); // links, unten, breit, hoch
    // oled.drawLine(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT, WHITE);
    // oled.drawCircle(64, 32, 31, WHITE);
    oled.display();

    // scanI2CBus();  for tests  um herauszufinden welch devices vorhanden sind

    FastLED.addLeds<SK9822, DATA_PIN, CLOCK_PIN, RGB>(leds, NUM_LEDS);

    leds[0] = CRGB{0, 0, 0}; // R B G
    leds[1] = CRGB{0, 0, 0};
    leds[2] = CRGB{0, 0, 0};
    leds[3] = CRGB{0, 0, 0};

    FastLED.show();


    switch (mode)   // Init Phase: 
    {
        case MODE_PS4:

            connected = FALSE; 

            switch (ps)
            {
                case PS4_GREEN: PS4.begin("10:20:30:40:50:60"); break;
                case PS4_RED:   PS4.begin("10:20:30:40:50:61"); break;
                case PS4_BLUE:  PS4.begin("10:20:30:40:50:62"); break;
            }

        break; 

        case MODE_BT:
            if (!BT.begin(BT_NAME)) 
            {
                printf("Fehler beim Start von Bluetooth!\n");
            }
            else
            {
                printf("BT scout25 gestartet\n");
            }


    //  rotate for 360° - calibrate mfs-system and count impulses : 
    // das System soll sich für 3 Sekunden um die eigene Achse drehen. 
    // dabei soll Calibriert werden und es sollen die Impulse gezählt werden. 


    watch = 3000;
    oled.setCursor(0, 20);
    onBoardLedOn();

// --- derzeit kein Drehen!    drive( -250, 250); // dreh dich  --- nur calibrieren!


    while (watch)
    {
        calibrateMFC();
        printImpulse();
    }
    
    drive(0, 0);  // left , right   -250 ... 

    onBoardLedOff();

    watch = 3000; while (watch); // drei Sekunden zum Ablesen der Impulswerte! 

        break;

        case MODE_MENU:

        /*pinMode(LDR1, OUTPUT);
    pinMode(LDR2, OUTPUT);
    pinMode(LDR3, OUTPUT);
    pinMode(LDR4, OUTPUT);

    digitalWrite(LDR1, HIGH);
    digitalWrite(LDR2, HIGH);
    digitalWrite(LDR3, HIGH);
    digitalWrite(LDR4, HIGH);*/


        break; 
    }

}

void loop() 
{       
static char flon = FALSE;
static char x = 1;
static int newL1 = 0; 
static int oldL1 = 0;
static int newR1 = 0; 
static int oldR1 = 0;
static int newT = 0; 
static int oldT = 0;
int c; 
static int fled = 1; 


    int SensorWert1 = analogRead(LDR1);
    int SensorWert2 = analogRead(LDR2);
    int SensorWert3 = analogRead(LDR3);
    int SensorWert4 = analogRead(LDR4);
    batteryLevel = analogRead(BATTERY_LEVEL) / REFV;
    angle = getMFC_Angle();


    switch (mode)
    {
        case MODE_PS4:

            if (tenMSecFlag)
            {
                tenMSecFlag = FALSE;
            
                if (connected)
                {
                    speedL = PS4.L2Value(); 
                    speedR = PS4.R2Value(); 

                    newL1 = PS4.L1();
                    if((newL1 != 0) && (oldL1 == 0)) 
                    {
                        dirL = -dirL;
                    }
                    oldL1 = newL1;

                    newR1 = PS4.R1();
                    if((newR1 != 0) && (oldR1 == 0)) 
                    { 
                        dirR = -dirR; 
                    }
                    oldR1 = newR1;
            
                    newT = PS4.Triangle();
                    if((newT != 0) && (oldT == 0))
                    {

                    }
                    oldT = newT;
                }
                else
                {  
                    if(PS4.isConnected())
                    {
                        connected = TRUE; 
                        onBoardLedOn();

                        switch (ps)
                        {
                            case PS4_GREEN: leds[0] = CRGB{0, 0, 255}; leds[1] = CRGB{0, 0, 255}; break;
                            case PS4_RED:   leds[0] = CRGB{255, 0, 0}; leds[1] = CRGB{255, 0, 0};break;
                            case PS4_BLUE:  leds[0] = CRGB{0, 255, 0}; leds[1] = CRGB{0, 255, 0};break;
                        }   

                        leds[2] = CRGB{0, 0, 0};
                        leds[3] = CRGB{0, 0, 0};

                        FastLED.show();
                    }
                }
            }

            if (qSecFlag)
            {
                qSecFlag = FALSE;
            }

            if (connected == TRUE)
            {
                drive( dirL * speedL,  dirR * speedR);
            }

        break;

        case MODE_BT:

            if (tenMSecFlag)
            {
                tenMSecFlag = FALSE;
                while (BT.available()) 
                {
                    c = BT.read();
                    BT.write((uint8_t)c);
                    Serial.write((uint8_t)c);
                }
            }
            if (qSecFlag)
            {
                qSecFlag = FALSE;
        printf("prell: %d ldr1-4: %d %D %d %d\n", prell, SensorWert1, SensorWert2, SensorWert3, SensorWert4);
        BT.write((uint8_t)prell + '0');

            // distance: 

                digitalWrite(TRIG_PIN, LOW);        delay(5);
                digitalWrite(TRIG_PIN, HIGH);       delay(5);
                digitalWrite(TRIG_PIN, LOW);        
                distance = pulseIn(ECHO_PIN, HIGH) / 58.23;   // durch 58.23 
 
                printData();

            }

        break;

        case MODE_MENU:

            // do nothing! 

        break;
    }
}


// Magnetic field sensor:

int initMFC(void)
{
  Wire.beginTransmission(MFS);
  Wire.write(0x00); Wire.write(0x70); // Konfiguration A
  Wire.endTransmission();

  Wire.beginTransmission(MFS);
  Wire.write(0x01); Wire.write(0xA0); // Gain höher setzen
  Wire.endTransmission();

  Wire.beginTransmission(MFS);
  Wire.write(0x02); Wire.write(0x00); // Continuous mode
  return Wire.endTransmission();
}

void preppareMfsCalibrationSystem(void)
{
    xMin = yMin = 32767;
    xMax = yMax = -32768;
}


void readRawMFC(int16_t* x, int16_t* y, int16_t* z)
{
    Wire.beginTransmission(MFS);
    Wire.write(0x03);
    Wire.endTransmission(false); 

    Wire.requestFrom(MFS, 6); 
    if (Wire.available() == 6) 
    {
        *x = Wire.read() << 8; *x |= Wire.read(); 
        *z = Wire.read() << 8; *z |= Wire.read(); 
        *y = Wire.read() << 8; *y |= Wire.read(); 
    } 
}

int calibrateMFC(void)
// this function has to be called during rotation!
// after 42 calls - the system schould 
// have been rotated over 360° - just simple!
{
    int ret = FALSE; 
    unsigned long lastSampleTime = 0;

    readRawMFC(&x, &y, &z);

    // Ausreißer sollten eliminiert werden... (-4600...) 

    if (x > xMax) { xMax = x; }
    if (x < xMin) { xMin = x; }
    if (y > yMax) { yMax = y; }
    if (y < yMin) { yMin = y; }    

    if ( ((xMax -xMin) > 180) && ((yMax -yMin) > 200)) ret = isCalibrated = TRUE; 


    printf("|| x: %04d  %04d  xmin: %04d xmax: %04d ||  y: %04d  %04d ymin %04d ymax %04d || L: %04d R: %04d || ret: %d angle: %3.2f\n",
            x, (xMax - xMin), xMin, xMax, y, (yMax - yMin), yMin, yMax, impulsCntL, impulsCntR, ret, getMFC_Angle());


    return ret;
}

float getMFC_Angle()
{
    float xw, yw, angle;

    readRawMFC(&x, &y, &z);

    if ((xMax != xMin) && (yMax != yMin))  // avoid div by zero
    {
        xw = 2.0 * (x - xMin) / (xMax - xMin) - 1.0;
        yw = 2.0 * (y - yMin) / (yMax - yMin) - 1.0;
        angle = atan2(-yw, xw) * 180.0 / M_PI;
    }
    else 
    {
        angle = 0.;
    }

    if (angle < -90.) angle += 360.;
    angle -= 90.;

    // South = 0°
    // East = 90°
    // West = -90°
    // North = +-180°

    return angle;  
}



// OLED DISPLAY:

void printData(void)
{
    int angleInt = 0;
    oled.fillRect(0, 0, 128, 64, 0); // clear all!

    sprintf(text,"b: 0.00 V");
    text[3] = (int)(batteryLevel) %10 + '0';
    text[4] = '.';
    text[5] = (int)(batteryLevel * 10 ) %10 + '0';
    text[6] = (int)(batteryLevel * 100) %10 + '0';
    oled.setCursor(20, 16);
    oled.print(text);

    sprintf(text,"w:       ");

    if (angle >= 0) { text[2] = '+'; angleInt = (int)(angle*10.);} else { text[2] = '-'; angleInt = (int)(angle*-10.); }
    
    if (angleInt >= 1000) text[4] = (int)(angleInt/1000) % 10 + '0';
    if (angleInt >= 100)  text[5] = (int)(angleInt/100)  % 10 + '0';
    text[6] = (int)(angleInt/10)                         % 10 + '0';
    text[7] = '.';
    text[8] = (int)(angleInt)                            % 10 + '0';
    oled.setCursor(20, 32);
    oled.print(text);

    sprintf(text,"d:       ");
    if (distance >= 100.) text[2] = (int)(distance/100) % 10 + '0'; else text[2] = ' ';
    if (distance >= 10.)  text[3] = (int)(distance/10)  % 10 + '0'; else text[3] = ' ';
    text[4] = (int)(distance)    % 10 + '0';
    text[5] = '.';
    text[6] = (int)(distance*10) % 10 + '0';
    text[7] = 'c';
    text[8] = 'm';
    oled.setCursor(20, 48);
    oled.print(text);

    // oled.drawRect(10, 25, 40, 15, WHITE); // links, unten, breit, hoch
    // oled.drawLine(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT, WHITE);
    // oled.drawCircle(64, 32, 31, WHITE);
    oled.display();
}

void printImpulse(void)
{
    oled.fillRect(0, 17, 128, 64, 0); // clear rect 

    sprintf(text,"iL: 000 ");
    text[4] = (int)(impulsCntL/100) %10 + '0';
    text[5] = (int)(impulsCntL/10 ) %10 + '0';
    text[6] = (int)(impulsCntL   ) %10 + '0';

    oled.setCursor(20, 20);
    oled.print(text);

    sprintf(text,"iR: 000 ");
    text[4] = (int)(impulsCntR/100) %10 + '0';
    text[5] = (int)(impulsCntR/10 ) %10 + '0';
    text[6] = (int)(impulsCntR    ) %10 + '0';

    oled.setCursor(20, 40);
    oled.print(text);

    oled.display();
}



// EEPROM:

void storeStr2EEPROM(String word, int address)
{
    int i;

    for (i = 0; i < word.length(); i++)
    {
        EEPROM.write(address + i, word[i]);
    }
    EEPROM.write(address + word.length(), '\0');    
    EEPROM.commit();  // Änderungen speichern
}

void storeVal2EEPROM(int x, int address)
{
    EEPROM.write(address, (char)x);    
    EEPROM.commit();  // Änderungen speichern
}


String readFromEEPROM(int address)
{
  String word = "";
  char c;
  while ((c = EEPROM.read(address++)) != '\0') 
  { // Lies Zeichen bis zur Null-Terminierung
    word += c;
  }
  return word;
}

void preSet(void)
{
    storeStr2EEPROM("0", EEPROM_MOTOR_SYS_ADDR);
    storeStr2EEPROM("A1-A82861", EEPROM_SSID_ADDR);
    storeStr2EEPROM("7PMGDV96J8", EEPROM_PASSWORD_ADDR);
    storeVal2EEPROM(PS4_GREEN, EEPROM_PS4_ADDR);     
}

void scanI2CBus()  // for checks
{
    byte error, address;
    Serial.println("Scanning I2C bus...");
    for(address = 1; address < 127; address++ ) {
        Wire.beginTransmission(address);
        error = Wire.endTransmission();
        if (error == 0) {
            Serial.print("I2C device found at 0x");
            if (address < 16)
                Serial.print("0");
            Serial.println(address, HEX);
        }
    }
}



//  driving system:

void drive(int left, int right)
{
    if (left  >= 0) { LDir = 1; } else { LDir = 0; left  = -left;  }
    if (right >= 0) { RDir = 0; } else { RDir = 1; right = -right; }

    digitalWrite(WHEEL_L_DIRECTION, LDir);
    digitalWrite(WHEEL_R_DIRECTION, RDir);

    vL = left;
    vR = right;
}

void driveC(int l, int r)
{
    int lc, rc;
    lc = (int)((l) * (1 - correctionRL));
    rc = (int)(r * (1 + correctionRL));
     drive(lc , rc);
     printf("driveC lc: %03d rc %03d\n", lc, rc);
}



void rototateByCompass(int degree)
{

}

// lights:

void onBoardLedOn(void)
{
    digitalWrite(ON_BOARD_LED, LOW);  // it's inverese connected
}

void onBoardLedOff(void)
{
    digitalWrite(ON_BOARD_LED, HIGH);  
}



void impuls_L_isr(void)
{
    impulsFlagL = TRUE; 
}


void impuls_R_isr(void)
{
    static int x = 0; 
/*
    if (countR == 0)
    {
        x ^= 1;
        digitalWrite(TEST_PIN_RX2, x);
    }    
    else
    {
        prell++;
    }
        */
    impulsFlagR = TRUE; 
    countR = 40;
}


//******************************************************************

//              T i m e r   I n t e r r u  p t : 

//******************************************************************

void IRAM_ATTR myTimer(void)   // periodic timer interrupt, expires each 0.1 msec
{
    static int32_t otick  = 0;
    static int32_t qtick = 0;
    static int32_t mtick = 0;
    static int x = 0; 
    
    otick++;
    qtick++;
    mtick++;
    ramp++;


    /*
    x ^= 1;
    //digitalWrite(TEST_PIN_TX2, x);


    if (countR) 
    {   countR--;
        digitalWrite(TEST_PIN_TX2, LOW);

    }
    else
    {
        digitalWrite(TEST_PIN_TX2, HIGH);
    }
    */
    
    if (impulsFlagL) {impulsFlagL = FALSE; impulsCntL++; }
    if (impulsFlagR) {impulsFlagR = FALSE; impulsCntR++; } 


    if (otick >= WAIT_ONE_SEC) 
    {
        oneSecFlag = TRUE;
        otick = 0; 
    }

    if (qtick >= WAIT_250_MSEC) 
    {
        qSecFlag = TRUE;
        qtick = 0; 
    }

    if (mtick >= WAIT_10_MSEC) 
    {
        tenMSecFlag = TRUE;
        mtick = 0; 
        if (watch) watch -= 10;
    }

    // PWM:
    if (motorSys == 0) 
    {
        if (ramp > vL) digitalWrite(WHEEL_L, L);  else digitalWrite(WHEEL_L, H);
        if (ramp > vR) digitalWrite(WHEEL_R, L);  else digitalWrite(WHEEL_R, H);
    }
    if (motorSys == 1) // System mit Zusatzprint:
    { 
        if (LDir) if (ramp < vL) digitalWrite(WHEEL_L, L);  else digitalWrite(WHEEL_L, H);
        else      if (ramp > vL) digitalWrite(WHEEL_L, L);  else digitalWrite(WHEEL_L, H);

        if (RDir) if (ramp < vR) digitalWrite(WHEEL_R, L);  else digitalWrite(WHEEL_R, H);
        else      if (ramp > vR) digitalWrite(WHEEL_R, L);  else digitalWrite(WHEEL_R, H);
    }

}

