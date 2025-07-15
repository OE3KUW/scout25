/******************************************************************

                        s c o u t 2 5

                                                   қuran july 2025
******************************************************************/

#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>

#define TRUE                            true
#define FALSE                           false
#define H                               HIGH
#define L                               LOW

#define WAIT_ONE_SEC                    10000
#define WAIT_250_MSEC                   2500
#define WAIT_10_MSEC                    100 

#define ON_BOARD_LED                    5         // LoLin32 
#define BATTERY_LEVEL                   A3        // GPIO 39
#define REFV                            685.0     // factor

#define MFS                             0x1e      // magnetic field sensor

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

hw_timer_t *timer = NULL;
void IRAM_ATTR myTimer(void);
volatile int watch = 0; 
volatile int oneSecFlag;
volatile int qSecFlag;
volatile int tenMSecFlag;


volatile int vL, vR, LDir, RDir;
volatile float batteryLevel = 0.;
volatile int motorSys = 0;
String motorSysFromEEPROM = "";
String ssidWord = "";
String password = "";

int xMin = 32767, xMax = -32768;
int yMin = 32767, yMax = -32768;
int16_t  x, y, z;
int mfcInitialized = FALSE;
bool isCalibrated = FALSE;
const uint8_t impulsL = 14;
const uint8_t impulsR = 27;

void preSet(void);
void store2EEPROM(String word, int address);
String readFromEEPROM(int address);

void rototateByCompass(int degree); 
void drive(int left, int right);

void onBoardLedOn(void);
void onBoardLedOff(void);

int  initMFC(void);
int  calibrateMFC(void);
float getMFC_Angle();

void impuls_R_isr(void);
void impuls_L_isr(void);

void readRawMFC(int16_t* x, int16_t* y, int16_t* z);
void scanI2CBus(); 


void setup() 
{
    Serial.begin(115200);

    pinMode(ON_BOARD_LED, OUTPUT);
    pinMode(WHEEL_L, OUTPUT);
    pinMode(WHEEL_R, OUTPUT);
    pinMode(WHEEL_L_DIRECTION, OUTPUT);
    pinMode(WHEEL_R_DIRECTION, OUTPUT);
    pinMode(BATTERY_LEVEL, INPUT);
   
    pinMode(TEST_PIN_RX2, OUTPUT);
    

    /*
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT_PULLUP);  // ? PULLUP? vielleicht nicht nötig... 
    
        digitalWrite(TRIG_PIN, LOW);
        delay(5);
        digitalWrite(TRIG_PIN, HIGH);
        delay(5);
        digitalWrite(TRIG_PIN, LOW);

        distance = pulseIn(ECHO_PIN, HIGH);   // durch 58.23 
 if ((int)(distance/58.23) < 12)
        {
            leds[2] = CRGB{255, 0, 255};
            leds[3] = CRGB{255, 0, 255};
        }
        else
        {
            leds[2] = CRGB{255, 255, 255};
            leds[3] = CRGB{255, 255, 255};
        }


        */
    
    if (!EEPROM.begin(EEPROM_SIZE)) {
        Serial.println("EEPROM initialisieren fehlgeschlagen!");
        return;
    }

    timer = timerBegin(0, 80, true);
    timerAttachInterrupt(timer, &myTimer, true);
    timerAlarmWrite(timer, 100, true);  // 0.1 msec
    timerAlarmEnable(timer);

    attachInterrupt(digitalPinToInterrupt(impulsR), impuls_R_isr, FALLING);
    attachInterrupt(digitalPinToInterrupt(impulsL), impuls_L_isr, FALLING);
    
    oneSecFlag = FALSE; 
    qSecFlag = FALSE;
    tenMSecFlag = FALSE; 

    sei(); // start all interrupts!  especially printf need this ... 

    // preSet(); // store setup to EEPROM
    
    batteryLevel = analogRead(BATTERY_LEVEL) / REFV;
    motorSysFromEEPROM = readFromEEPROM(EEPROM_MOTOR_SYS_ADDR);
    motorSys = (char)motorSysFromEEPROM[0] - '0';
    ssidWord = readFromEEPROM(EEPROM_SSID_ADDR);
    password = readFromEEPROM(EEPROM_PASSWORD_ADDR);

    Wire.begin(21, 22);
    
    watch = 100;  while(watch); // wait for some 100 msec

    x = initMFC();
    
    printf("\n______________________________________________________________________________________\n");
    printf("start!\n");
    printf("battery: %1.3f\n", batteryLevel);
    printf("motorSystem: %d\n", motorSys);
    printf("ssid: %s\n", ssidWord);
    printf("password: %s\n", password);
    printf("magneticfield-sesor: %d\n", x);

    // scanI2CBus();  for tests

//  rotate for 360°: as long, as the compass system is calibrated

    drive(80, -80);

    xMin = yMin = 32767;
    xMax = yMax = -32768;

  
    while (calibrateMFC() != TRUE);
    // one second addidionall - to be sure! 

    watch = 1000;  
    onBoardLedOn();
    while(watch) // wait for some seconds 
    {
        calibrateMFC();
    }
    onBoardLedOff();

    x = (int) getMFC_Angle();

    while (((x > 3) || (x < 3))) 
    {
        x = (int) getMFC_Angle();
        printf("x = %d angle: %3.2f\n", x, getMFC_Angle());

        if (x > 0 ) drive(-60, 60); else drive(60, -60);
    }

    drive(0, 0);



    onBoardLedOff(); // funktioniert noch nicht richtig! ....
}

void loop() 
{

}

/*****************************************************************/
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

void readRawMFC(int16_t* x, int16_t* y, int16_t* z)
{
    Wire.beginTransmission(MFS);
    Wire.write(0x03);
    Wire.endTransmission(false); 

    Wire.requestFrom(MFS, 6); //delay(100);
    if (Wire.available() == 6) 
    {
        *x = Wire.read() << 8; *x |= Wire.read(); //delay(10);
        *z = Wire.read() << 8; *z |= Wire.read(); //delay(10);
        *y = Wire.read() << 8; *y |= Wire.read(); //delay(10);
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

    if ( ((xMax -xMin) > 200) && ((yMax -yMin) > 200)) ret = isCalibrated = TRUE; 


    printf("|| x: %04d  %04d ||  y: %04d  %04d|| ret: %d angle: %3.2f\n",
            x, (xMax - xMin), y, (yMax - yMin), ret, getMFC_Angle());


    return ret;
}

float getMFC_Angle()
{
    int16_t x_raw, y_raw, z_raw;
    float x, y, angle;

    readRawMFC(&x_raw, &y_raw, &z_raw);

    if ((xMax != xMin) && (yMax != yMin))  // avoid div by zero
    {
        x = 2.0 * (x_raw - xMin) / (xMax - xMin) - 1.0;
        y = 2.0 * (y_raw - yMin) / (yMax - yMin) - 1.0;
        angle = atan2(-y, x) * 180.0 / M_PI;
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


/*****************************************************************/
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



/*****************************************************************/
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

void rototateByCompass(int degree)
{

}

/*****************************************************************/
// lights:

void onBoardLedOn(void)
{
    digitalWrite(ON_BOARD_LED, LOW);  // it's inverese connected
}

void onBoardLedOff(void)
{
    digitalWrite(ON_BOARD_LED, HIGH);  
}


/******************************************************************

/******************************************************************/

void impuls_R_isr(void)
{
    digitalWrite(TEST_PIN_RX2, LOW);
}

void impuls_L_isr(void)
{
    digitalWrite(TEST_PIN_RX2, HIGH);
}
/******************************************************************

              T i m e r   I n t e r r u  p t : 

/******************************************************************/

void IRAM_ATTR myTimer(void)   // periodic timer interrupt, expires each 0.1 msec
{
    static int32_t otick  = 0;
    static int32_t qtick = 0;
    static int32_t mtick = 0;
    static unsigned char ramp = 0;
    
    otick++;
    qtick++;
    mtick++;
    ramp++;

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

/******************************************************************

Tests.... 



  #include <Arduino.h>
#include <Wire.h>

#define HMC5883L_ADDR 0x1E

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);  // Standardtakt, saubere Umgebung
  delay(200);

  Wire.beginTransmission(HMC5883L_ADDR);
  Wire.write(0x00); Wire.write(0x70); // Konfiguration A
  Wire.endTransmission();

  Wire.beginTransmission(HMC5883L_ADDR);
  Wire.write(0x01); Wire.write(0xA0); // Gain höher setzen
  Wire.endTransmission();

  Wire.beginTransmission(HMC5883L_ADDR);
  Wire.write(0x02); Wire.write(0x00); // Continuous mode
  Wire.endTransmission();

}

void loop() {
  int16_t x, y, z;

  Wire.beginTransmission(HMC5883L_ADDR);
  Wire.write(0x03);
  Wire.endTransmission(false); 

  Wire.requestFrom(HMC5883L_ADDR, 6);delay(100);
  if (Wire.available() == 6) {
    x = Wire.read() << 8 | Wire.read();delay(10);
    z = Wire.read() << 8 | Wire.read();delay(10);
    y = Wire.read() << 8 | Wire.read();delay(10);

    Serial.print("X: "); Serial.print(x);
    Serial.print(" Y: "); Serial.print(y);
    Serial.print(" Z: "); Serial.println(z);
  } else {
    Serial.println("Fehler beim Lesen");
  }

  delay(500);
}


// den MFC fragen, wer er ist... 
char id[4];
Wire.beginTransmission(0x1E);
Wire.write(0x0A);
Wire.endTransmission();

Wire.requestFrom(0x1E, 3);
id[0] = Wire.read();
id[1] = Wire.read();
id[2] = Wire.read();
id[3] = '\0';

Serial.print("HMC ID: ");
Serial.println(id);




*/ 

