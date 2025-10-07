/******************************************************************

                        s c o u t 2 5

                                                   қuran july 2025
******************************************************************/
/*
#include <Arduino.h>
#include <BluetoothSerial.h>

BluetoothSerial BT;

void setup() {
  Serial.begin(115200);
  delay(200);

  // Optional: feste PIN, hilft auf manchen Android-Geräten
  // BT.setPin("1234");

  if (!BT.begin("scout25!")) {
    Serial.println("BT start FAIL");
    while (1) delay(1000);
  }
  Serial.println("BT start OK, name= scout25");
}

void loop() {
  // Echo
  while (BT.available()) {
    int c = BT.read();
    BT.write((uint8_t)c);
    Serial.write((uint8_t)c);
  }
  delay(1);
}


*/
//--------------------------------------------------------------

#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#define FASTLED_ALL_PINS_HARDWARE_SPI
#include <FastLED.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <PS4Controller.h>
#include <BluetoothSerial.h>


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

#define TRIG_PIN                        25  
#define ECHO_PIN                        26
#define TEST_PIN_RX2                    16

#define SCREEN_WIDTH                    128                // OLED display width, in pixels
#define SCREEN_HEIGHT                   64                 // OLED display height, in pixels
#define OLED                            0x3c

#define NUM_LEDS                        4
#define DATA_PIN                        23
#define CLOCK_PIN                       18


hw_timer_t *timer = NULL;
void IRAM_ATTR myTimer(void);
volatile int watch = 0; 
volatile int oneSecFlag;
volatile int qSecFlag;
volatile int tenMSecFlag;

BluetoothSerial BT;  // SPP „Serial Port Profile“
static const char* BT_NAME = "scout25NEW";

volatile int vL, vR, LDir, RDir;
volatile int impulsCntL;
volatile int impulsCntR;
volatile int impulsFlagL;
volatile int impulsFlagR;
float correctionRL; 

volatile float batteryLevel = 0.;
volatile int motorSys = 0;
String motorSysFromEEPROM = "";
String ssidWord = "";
String password = "";

int xMin = 32767, xMax = -32768;
int yMin = 32767, yMax = -32768;
int16_t  x, y, z;
int mfcInitialized = FALSE;
int isCalibrated = FALSE;
const uint8_t impulsL = 14;
const uint8_t impulsR = 27;
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


CRGB leds[NUM_LEDS];

void preSet(void);
void store2EEPROM(String word, int address);
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
    
    pinMode(TEST_PIN_RX2, OUTPUT);
   
    
    if (!EEPROM.begin(EEPROM_SIZE)) {
        Serial.println("EEPROM initialisieren fehlgeschlagen!");
        return;
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

    if (!BT.begin(BT_NAME)) 
    {
        printf("Fehler beim Start von Bluetooth!\n");
    }
    else
    {
        printf("BT scout25 gestartet\n");
    }
    // muss noch vor sei durchgefhrt werden!


    sei(); // start all interrupts!  especially printf, impulsCount and timer need this ... 


    watch = 100; while (watch); // 100 ms wait time

    preSet(); // store setup to EEPROM um andere Daten ins E2Prom zu schreiben

  
    


    batteryLevel = analogRead(BATTERY_LEVEL) / REFV;
    motorSysFromEEPROM = readFromEEPROM(EEPROM_MOTOR_SYS_ADDR);
    motorSys = (char)motorSysFromEEPROM[0] - '0';
    ssidWord = readFromEEPROM(EEPROM_SSID_ADDR);
    password = readFromEEPROM(EEPROM_PASSWORD_ADDR);

    Wire.begin(21, 22); // i2c wird hier vorbereitet
    
    
    printf("\n______________________________________________________________________________________\n");
    printf("start!\n");
    printf("battery: %1.3f\n", batteryLevel);
    printf("motorSystem: %d\n", motorSys);
    printf("ssid: %s\n", ssidWord);
    printf("password: %s\n", password);
    printf("magneticfield-sesor: %d\n", initMFC());

    
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


    //  rotate for 360° - calibrate mfs-system and count impulses : 
    // das System soll sich für 3 Sekunden um die eigene Achse drehen. 
    // dabei soll Calibriert werden und es sollen die Impulse gezählt werden. 

    drive(0,0);
    impulsCntL = impulsCntR = 0;
    preppareMfsCalibrationSystem();

    watch = 3000;
    oled.setCursor(0, 20);
    onBoardLedOn();

    FastLED.addLeds<SK9822, DATA_PIN, CLOCK_PIN, RGB>(leds, NUM_LEDS);

    leds[0] = CRGB{0, 0, 0}; // R B G
    leds[1] = CRGB{0, 0, 0};
    leds[2] = CRGB{0, 0, 0};
    leds[3] = CRGB{0, 0, 0};

    FastLED.show();


// --- derzeit kein Drehen!    drive( -250, 250); // dreh dich  --- nur calibrieren!

    while (watch)
    {
        calibrateMFC();
        printImpulse();
    }
    
    drive(0,0); 

    onBoardLedOff();


    watch = 3000; while (watch); // drei Sekunden zum Ablesen der Impulswerte! 
    PS4.begin("10:20:30:40:50:62");  // gleichzeitig wird der PS Controller initialisiert

    // hier könnte man die ImpulsCounterWerte links und rechts speichern um dann damit die beiden Motoren unterschiedlich anzusprechen.
    // .... 




}

void loop() 
{       
static char flon = FALSE;
int c; 
    batteryLevel = analogRead(BATTERY_LEVEL) / REFV;
    angle = getMFC_Angle();


    if (tenMSecFlag)
    {
        tenMSecFlag = FALSE;

        while (BT.available()) 
        {
            c = BT.read();
            BT.write((uint8_t)c);
            Serial.write((uint8_t)c);
        }
    
        if (connected)
        {
            if(PS4.L1()) { dirL = -dirL; }
            if(PS4.R1()) { dirR = -dirR; }
                
            speedL = PS4.L2Value(); 

            speedR = PS4.R2Value(); 

            speedL = (dirL == 1) ? speedL: -speedL;
            speedR = (dirR == 1) ? speedR: -speedR;

            drive(speedL, speedR); 

            if(PS4.Triangle())
            {
                flon = (flon) ? FALSE : TRUE;

            }
            if (flon)
            { 
                   leds[0] = CRGB{0, 0, 255}; // R B G
                   leds[1] = CRGB{0, 255, 255};
                   leds[2] = CRGB{255, 0, 255};
                   leds[3] = CRGB{255, 255, 0};
                   FastLED.show();
            }
            else
            { 
                   leds[0] = CRGB{0, 0, 0}; // R B G
                   leds[1] = CRGB{0, 0, 0};
                   leds[2] = CRGB{0, 0, 0};
                   leds[3] = CRGB{0, 0, 0};
                   FastLED.show();
            }

        }
        else
        {  
            if(PS4.isConnected())
            {
               onBoardLedOn();
               connected = TRUE;

                   leds[0] = CRGB{0, 0, 255}; // R B G
                   leds[1] = CRGB{0, 255, 255};
                   leds[2] = CRGB{255, 0, 255};
                   leds[3] = CRGB{255, 255, 0};
                   FastLED.show();

            }
        }
    }
        
   
    if (qSecFlag)
    {
        qSecFlag = FALSE;
   

        digitalWrite(TRIG_PIN, LOW);
        delay(5);
        digitalWrite(TRIG_PIN, HIGH);
        delay(5);
        digitalWrite(TRIG_PIN, LOW);
        distance = pulseIn(ECHO_PIN, HIGH) / 58.23;   // durch 58.23 
 
        printData();
    }

    //watch = 1000; while(watch);
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

void store2EEPROM(String word, int address)
{
    int i;

    for (i = 0; i < word.length(); i++)
    {
        EEPROM.write(address + i, word[i]);
    }
    EEPROM.write(address + word.length(), '\0');    
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
    store2EEPROM("0", EEPROM_MOTOR_SYS_ADDR);
    store2EEPROM("A1-A82861", EEPROM_SSID_ADDR);
    store2EEPROM("7PMGDV96J8", EEPROM_PASSWORD_ADDR);
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
    //digitalWrite(TEST_PIN_RX2, LOW);
    impulsFlagR = TRUE; 
}


//******************************************************************

//              T i m e r   I n t e r r u  p t : 

//******************************************************************

void IRAM_ATTR myTimer(void)   // periodic timer interrupt, expires each 0.1 msec
{
    static int32_t otick  = 0;
    static int32_t qtick = 0;
    static int32_t mtick = 0;
    
    otick++;
    qtick++;
    mtick++;
    ramp++;

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

