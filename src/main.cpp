/******************************************************************


                          s c o u t 2 5


                                                     қuran nov 2025
******************************************************************/
#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#define FASTLED_ALL_PINS_HARDWARE_SPI
#include <FastLED.h>
// ROS:
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>    
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h> 
#include <std_msgs/msg/int32_multi_array.h> 
#include <rmw_microros/rmw_microros.h>

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <PS4Controller.h>
#include <BluetoothSerial.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include <esp_bt_main.h>
#include <esp_gap_bt_api.h>
#include <esp_bt_device.h>
#include <esp_system.h>

#define VERSION                         "jan 26"

#define TRUE                            true
#define FALSE                           false
#define H                               HIGH
#define L                               LOW

// WLAN & Agent:
/* */
/* /
#define WIFI_SSID                       "..."
#define WIFI_PASS                       "...!"
#define AGENT_IP                        "..." // IP vom Pi400 wlan0
#define AGENT_IP                        "..." // IP vom Pi400 wlan0
/**/

#define AGENT_PORT                      8888

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

#define EEPROM_SIZE                     200
#define EEPROM_ADDR                     0
#define EEPROM_ROB_NAME                 0
#define EEPROM_BRIGHTNESS_LEVEL         19
#define EEPROM_MIN_SPEED                21
#define EEPROM_MFS_MINX                 24
#define EEPROM_MFS_MAXX                 26
#define EEPROM_MFS_MINY                 28
#define EEPROM_MFS_MAXY                 30
#define EEPROM_SSID_ADDR                40
#define EEPROM_PASSWORD_ADDR            60
#define EEPROM_MOTOR_SYS_ADDR           80
#define EEPROM_PS4_ADDR                 82
#define EEPROM_WIFI_SYS_ADDR            84
#define EEPROM_WIFI_0_SSID_ADDR         100
#define EEPROM_WIFI_0_PASS_ADDR         115
#define EEPROM_WIFI_0_IP_ADDR           130
#define EEPROM_WIFI_1_SSID_ADDR         150
#define EEPROM_WIFI_1_PASS_ADDR         165
#define EEPROM_WIFI_1_IP_ADDR           180

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

#define PS4_RED                         61
#define PS4_GRAY                        62
#define PS4_BLUE                        60

#define DUAL_MODE                       0
#define JOY_STICK                       1

#define MODE_PS4                        0  
#define MODE_MENU                       1
#define MODE_BT                         2
#define DRIVE                           1   
#define ROTATE                          0

#define CMD_DRIVE                       0
#define CMD_FLED                        1


// micro-ROS Variablen:
rcl_allocator_t    allocator;
rclc_support_t     support;
rcl_node_t         node;
rcl_subscription_t subscriber;
rcl_publisher_t    publisher;
rclc_executor_t    executor;
 
std_msgs__msg__Int32 out_msg;   
std_msgs__msg__Int32MultiArray speed_msg;   // robId, speedL, speedR
static int32_t speed_data_buffer[3]; 

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
int last_impulsCntL;
int last_impulsCntR;
int circleMode;

float correctionRL; 
volatile int countR = 0;
volatile int prell = 0;

volatile float batteryLevel = 0.;
volatile int motorSys = 0;
volatile int wifiSys = 0;
int robId;
String robName;
String IntVal;
String motorSysFromEEPROM = "";
String wifiSysFromEEPROM = "";
String pskey = "";
String ssidWord = "";
String password = "";
String ipNode = "";

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
int vLSum = 0, vRSum = 0;
bool squareMode = FALSE;
int connected = FALSE;
int speedMin;
float distance;
Adafruit_SSD1306 oled(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire);
char text[20];
volatile static unsigned char ramp = 0;

int mode;
int autonomous;
int minSpeed;
int ps;
int psMode;
int msys;
int wsys;
int stateBT = DRIVE;
int isMFSavailable = FALSE;
int mfs = 0x1e;

CRGB leds[NUM_LEDS];

void preSet(void);
void getSet(void);
void storeStr2EEPROM(String word, int address);
String readFromEEPROM(int address);
void storeInt2EEPROM(int16_t x, int address);
int16_t getIntFromEEPROM(int address);
void drive(int left, int right);
void driveC(int l, int r);
void onBoardLedOn(void);
void onBoardLedOff(void);
int  initMFS(void);
void prepareMFSCalibrationSystem(void);
int  calibrateMFS(void);
void readRawMFS(int16_t* x, int16_t* y, int16_t* z);
float getMFS_Angle();
void impuls_R_isr(void);
void impuls_L_isr(void);
void scanI2CBus();   // for tests only 
void printData(void);   // Ausgabe am Display
void printImpulse(void); 
void printMotorSystem(int msys);
void printWifiSystem(int msys);
void printPs4(int ps);
void printStored(void);
void printCount(int i); 
void printSpeedMin(int i); 
void printComp(float w);
void printClearQ(void);
void printCleared(void);
void printRobName(const String& name);
void printReset(void);

void switchLedsOn(int ps, int sMode);
//void speed_callback(const void * msgin);

// Callback Funktion:

void speed_callback(const void * msgin)
{

    const std_msgs__msg__Int32MultiArray * msg = 
	    (const std_msgs__msg__Int32MultiArray *) msgin;

	if (msg->data.size < 3) {  
    	Serial.print("MultiArray zu kurz, size=");
    	Serial.println((int)msg->data.size);
    	return;
  	}

    int Id  = msg->data.data[0] >> 4;       // robId
    int cmd = msg->data.data[0] & 0x0f;   // command
    int lf  = msg->data.data[1];          // left - front
    int rb  = msg->data.data[2];          // right - back

    // commands:

    printf("Id %d cmd %d  lf %x rb %x\n", Id, cmd, lf, rb);

    if ((Id == robId) || Id == 0)  // Id == 0: command for every! rob 
    {
        switch(cmd)
        {
            case CMD_DRIVE:
                printf("drive left : %d   right : %d \n", lf,rb);
                drive(lf, rb);
            break;

            case CMD_FLED:   
                leds[0] = CRGB{ static_cast<fl::u8>((((lf >> 4) & 4) == 4) ? 255 : 0),
                                static_cast<fl::u8>((((lf >> 4) & 1) == 1) ? 255 : 0),
                                static_cast<fl::u8>((((lf >> 4) & 2) == 2) ? 255 : 0)};
                leds[1] = CRGB{ static_cast<fl::u8>((((lf     ) & 4) == 4) ? 255 : 0),
                                static_cast<fl::u8>((((lf     ) & 1) == 1) ? 255 : 0),
                                static_cast<fl::u8>((((lf     ) & 2) == 2) ? 255 : 0)};
                leds[2] = CRGB{ static_cast<fl::u8>((((rb     ) & 4) == 4) ? 255 : 0),
                                static_cast<fl::u8>((((rb     ) & 1) == 1) ? 255 : 0),
                                static_cast<fl::u8>((((rb     ) & 2) == 2) ? 255 : 0)};
                leds[3] = CRGB{ static_cast<fl::u8>((((rb >> 4) & 4) == 4) ? 255 : 0),
                                static_cast<fl::u8>((((rb >> 4) & 1) == 1) ? 255 : 0),
                                static_cast<fl::u8>((((rb >> 4) & 2) == 2) ? 255 : 0)};

                FastLED.show();

            break;
        }
    }
}

void printBtMac()
{
    const uint8_t* mac = esp_bt_dev_get_address();
     ("ESP32 BT MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}


void f(unsigned char a, unsigned char b, unsigned char c,  unsigned char d, unsigned char e, unsigned char f,
       unsigned char g, unsigned char h, unsigned char i,  unsigned char j, unsigned char k, unsigned char l)
{
    leds[0] = CRGB{a, b, c}; 
    leds[1] = CRGB{d, e, f}; 
    leds[2] = CRGB{g, h, i}; 
    leds[3] = CRGB{j, k, l}; FastLED.show();    
}


void setup() 
{
    int count; 
    bool ok = false;

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

    drive(0, 0);

    /* restore all eeprom variables: */
    if (!EEPROM.begin(EEPROM_SIZE)) {
        Serial.println("EEPROM initialisieren fehlgeschlagen!");
        return;
    }

    // Werte noch selbst speichern - das komt dann später weg !!! 
    preSet();
    getSet();
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         
    batteryLevel = analogRead(BATTERY_LEVEL) / REFV;

    Wire.begin(21, 22); // i2c wird hier vorbereitet

    if (digitalRead(TEST_PIN_RX2) == LOW)
    {
        mode = MODE_PS4;   
    }
    else if (digitalRead(TEST_PIN_TX2) == LOW)
    {
        mode = MODE_MENU;  //  Calibrieren, Motorystem setzen
    }
    else 
    {
        mode = MODE_BT;

        autonomous = TRUE;
        pinMode(TEST_PIN_RX2, OUTPUT);
        digitalWrite(TEST_PIN_RX2, HIGH);
        if (digitalRead(TEST_PIN_TX2) == LOW) autonomous = FALSE;
        digitalWrite(TEST_PIN_RX2, LOW);
        if (digitalRead(TEST_PIN_TX2) == HIGH) autonomous = FALSE;
        digitalWrite(TEST_PIN_RX2, HIGH);
        if (digitalRead(TEST_PIN_TX2) == LOW) autonomous = FALSE;
        digitalWrite(TEST_PIN_RX2, LOW);
        if (digitalRead(TEST_PIN_TX2) == HIGH) autonomous = FALSE;
    }

    // autonomous wird innerhalb MENU BT eingebaut - derzeit
   
    timer = timerBegin(0, 80, true);
    timerAttachInterrupt(timer, &myTimer, true);
    timerAlarmWrite(timer, 100, true);  // 0.1 msec -> 100
    timerAlarmEnable(timer);

    attachInterrupt(digitalPinToInterrupt(impulsR), impuls_R_isr, FALLING);
    attachInterrupt(digitalPinToInterrupt(impulsL), impuls_L_isr, FALLING);

    drive(0, 0);
    onBoardLedOff();

    oneSecFlag = FALSE; 
    qSecFlag = FALSE;
    tenMSecFlag = FALSE;
    impulsFlagL = FALSE;  
    impulsFlagR = FALSE;  
    impulsCntL = impulsCntR = 0;

    sei(); // start all interrupts!  especially printf, impulsCount and timer need this ... 

    watch = 100; while (watch){    }; // 100 ms wait time
  
    isMFSavailable = !initMFS(); // init liefert 0 falls der Baustein initialisiert werden konnte.
    
    printf("\n______________________________________________________________________________________\n");
    printf("\nversion: %s | name: %s  robId %d  | %s      mode: %s\n", 
              VERSION,      robName,  robId,
        (autonomous == TRUE) ? "autonomes System": "",   (mode == 0)? "ps4": (mode == 1)? "MENU" : "BlueTooth");
    printf("______________________________________________________________________________________\n");
    printf("battery: %1.3f\n", batteryLevel);
    printf("motorSystem: %d  wifi: %s\n", motorSys, (wifiSys == 0)? "home":"work");
    printf("minSpeed %d\n", minSpeed);
    printf("ssid: %s  password: %s ipNode %s\n", (char*) ssidWord.c_str(), password.c_str(), ipNode.c_str());
    printf("ps4-System: %d %s\n", ps, (ps == PS4_GRAY) ? "gray" : (ps == PS4_RED) ? "red" : "blue");
    printf("xMin %d xMax %d yMin %d yMax %d\n", xMin, xMax, yMin, yMax);
    printf("magneticfield-sesor: (MFS) available: %s 0x%02X\n", (isMFSavailable) ? "yes":"no", (isMFSavailable) ? mfs : 0);
    printf("______________________________________________________________________________________\n");
    printf("start!\n");
    
    oled.begin(SSD1306_SWITCHCAPVCC, OLED);
    oled.clearDisplay();
    oled.setTextSize(2);
    oled.setTextColor(WHITE);

    oled.setCursor(20, 0);
    oled.print(robName);

    oled.display();

    if (mode == MODE_MENU) scanI2CBus();  // test: um herauszufinden welch devices vorhanden sind

    FastLED.addLeds<SK9822, DATA_PIN, CLOCK_PIN, RGB>(leds, NUM_LEDS);

    leds[0] = CRGB{0, 0, 0}; // R B G
    leds[1] = CRGB{0, 0, 0};
    leds[2] = CRGB{0, 0, 0};
    leds[3] = CRGB{0, 0, 0};

    FastLED.show();

    if (autonomous)
    {
        int i = 0;
        int msg = 0; 
        int ret;
        int phase = 0;

        // zunächst nur Blinken (Funkeln) - per Zufall ! 

//        set_microros_wifi_transports((char*)WIFI_SSID, (char*)WIFI_PASS, (char*)AGENT_IP, AGENT_PORT);
        set_microros_wifi_transports((char*) ssidWord.c_str(), 
                                     (char*) password.c_str(), 
                                     (char*) ipNode.c_str(), AGENT_PORT);
        for(;phase == 0;)
        {
            i = rand()%4; 
            if (i > 3) i = 0;
            switch(i) // sobald ein die eigene robId empfangen wird - könnte hier umgeschalten werden
                      // das ist in der Funktion cmd_speed
            {
                case 0: f(0xff, 0x00, 0x00,   0x00, 0xff, 0x00,   0x00, 0x00, 0xff,   0x00, 0x00, 0x00); break;
                case 1: f(0x00, 0x00, 0x00,   0xff, 0x00, 0x00,   0x00, 0xff, 0x00,   0x00, 0x00, 0xff); break;
                case 2: f(0x00, 0x00, 0xff,   0x00, 0x00, 0x00,   0xff, 0x00, 0x00,   0x00, 0xff, 0x00); break;
                case 3: f(0x00, 0xff, 0x00,   0x00, 0x00, 0xff,   0x00, 0x00, 0x00,   0xff, 0x00, 0x00); break;
            } 
            watch = 500; 
            while(watch)
            {
                // autonomous wait - loop: - ist der ROS2 Server - aget erreichbar? 

                if (RMW_RET_OK == rmw_uros_ping_agent(50, 1)) { phase = 1;} else
                printf("."); // warte
             
            }
        }         

        printf("autonomous mode: phase == 1:  start ros2\n");

        leds[0] = CRGB{0, 0, 0}; // R B G
        leds[1] = CRGB{0, 0, 0};
        leds[2] = CRGB{0, 0, 0};
        leds[3] = CRGB{0, 0, 0};

        FastLED.show();


        // ROS 2 init: 

  	    allocator = rcl_get_default_allocator();
  	    rcl_ret_t rc;
 
        // Support
  	    rc = rclc_support_init(
    	    &support, 
		    0, 
		    NULL, 
		    &allocator);
    
	    if (rc != RCL_RET_OK) { Serial.println("support_init ERROR"); return; }
 
        // Node
        rc = rclc_node_init_default(
      	    &node,
      	    "esp32_motor_robot",
      	    "",
      	    &support);
 
  	    if (rc != RCL_RET_OK) { Serial.println("node_init ERROR"); return; }

	    // Publisher:
	    rc = rclc_publisher_init_default(
    	    &publisher,
    	    &node,
    	    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    	    "esp_value");
	    if (rc != RCL_RET_OK) { Serial.println("publisher_init ERROR"); return; }

     	// Message init + Buffer setzen (WICHTIG für MultiArray)
	    std_msgs__msg__Int32MultiArray__init(&speed_msg); //###

	    speed_msg.data.data = speed_data_buffer; 
	    speed_msg.data.capacity = 3;  
	    speed_msg.data.size = 0;  
  	
	    // Subscriber auf "cmd_speed" (std_msgs/Int32)
  	    rc = rclc_subscription_init_default(
      	    &subscriber,
      	    &node,
            //###   ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
      	    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
      	    "cmd_speed");
  
    	if (rc != RCL_RET_OK) { Serial.println("subscription_init ERROR"); return; }
 
        // Executor (1 Subscription):
        rc = rclc_executor_init(&executor, &support.context, 1, &allocator);
  
	    if (rc != RCL_RET_OK) { Serial.println("executor_init ERROR"); return; }
 
        rc = rclc_executor_add_subscription(
      	    &executor,
      	    &subscriber,
      	    &speed_msg,
      	    &speed_callback,
      	    ON_NEW_DATA);
  	    if (rc != RCL_RET_OK) { Serial.println("executor_add_sub ERROR"); return; }

        printf("ros2 init abgeschlossen!\n");

        for(;;)
        {
            watch = 2000; 
            while(watch)
            {
                // autonomous loop:
                
  	            rclc_executor_spin_some(&executor, RCL_MS_TO_NS(50)); // 50

                if (oneSecFlag) // falls schon jemand etwas senden möchte
	            {
		            oneSecFlag = FALSE;
		            out_msg.data = (int32_t)msg; msg++;
		            ret = rcl_publish(&publisher, &out_msg, NULL);
		            if (ret) printf("publishing returns %d\n", ret);
	            }
            }
        }         
    }

    switch (mode)   // Init Phase: 
    {
        case MODE_PS4:

            printf("ps4 mode!\n");

            watch = 200; while(watch); // wait 0.2 sec

            connected = FALSE; 
            psMode = DUAL_MODE;

            switch (ps)
            {
                case PS4_RED:   ok = PS4.begin("10:20:30:40:50:61"); break;
                case PS4_GRAY:  ok = PS4.begin("10:20:30:40:50:62"); break;
                case PS4_BLUE:  ok = PS4.begin("10:20:30:40:50:60"); break;  
            }

            if (!ok) printf("ps4 failed!\n"); else printf("ps4 correct initialized and started\n");

            printBtMac();

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
            printf("BT mode\n");
            drive(0,0);

            if (autonomous)
            {
                leds[0] = CRGB{255, 255, 0}; // R B G
                leds[1] = CRGB{255, 255, 0};
                leds[2] = CRGB{0, 0, 0};
                leds[3] = CRGB{0, 0, 0};
                FastLED.show();
            }

        break;

/*******************/
/*     M E N U     */
/*******************/

        case MODE_MENU:


            leds[0] = CRGB{255, 255, 255}; // R B G
            leds[1] = CRGB{255, 255, 255};
            leds[2] = CRGB{255, 255, 255};
            leds[3] = CRGB{255, 255, 255};
            FastLED.show();

        /*  Motorsystem wählen: */

            msys = 0; 

            while (digitalRead(TEST_PIN_TX2) == LOW)
            {
                msys = (msys) ? 0:1; 
                printMotorSystem(msys);
                printf("motsys: %d\n", msys);
                watch = 1000; while (watch); // 100 ms wait time
            }

            if (msys == 0)
                storeStr2EEPROM("0", EEPROM_MOTOR_SYS_ADDR);
            else
                storeStr2EEPROM("1", EEPROM_MOTOR_SYS_ADDR);

            motorSys = msys;    
            drive(0,0);    
            printStored();
            printf("stored motor system: %d\n", msys);

            /* TX2 ist noch immer low - dadurch wurde ja gespeichert - darum erst warten, wis der TX2 sicher high ist */

            while (digitalRead(TEST_PIN_TX2) == HIGH); // warten bis Kabel wieder gesteckt ist.
            while (digitalRead(TEST_PIN_TX2) == HIGH); // warten bis Kabel wieder gesteckt ist.
            while (digitalRead(TEST_PIN_TX2) == HIGH); // warten bis Kabel wieder gesteckt ist. // PRELLSCHUTZ


        /*  wifisystem wählen: */

            wsys = 0; 

            while (digitalRead(TEST_PIN_TX2) == LOW)
            {
                wsys = (wsys) ? 0:1; 
                printWifiSystem(wsys);
                printf("wifi sys: %d\n", wsys);
                watch = 1000; while (watch); // 100 ms wait time
            }

            if (wsys == 0)
                storeStr2EEPROM("0", EEPROM_WIFI_SYS_ADDR);
            else
                storeStr2EEPROM("1", EEPROM_WIFI_SYS_ADDR);

            wifiSys = wsys;    
            printStored();
            printf("stored wifi: %s\n", (wsys == 0)? "home" : "work");

            while (digitalRead(TEST_PIN_TX2) == HIGH); // warten bis Kabel wieder gesteckt ist.
            while (digitalRead(TEST_PIN_TX2) == HIGH); // warten bis Kabel wieder gesteckt ist.
            while (digitalRead(TEST_PIN_TX2) == HIGH); // warten bis Kabel wieder gesteckt ist. // PRELLSCHUTZ

            // Controller wählen: 

            ps  = PS4_GRAY; 

            while (digitalRead(TEST_PIN_TX2) == LOW)
            {
                ps++; 
                if (ps > PS4_GRAY) ps = PS4_BLUE;
                printPs4(ps);
                watch = 1000; while (watch); // 100 ms wait time
            }

            storeInt2EEPROM(ps, EEPROM_PS4_ADDR);     

            printStored();


            while (digitalRead(TEST_PIN_TX2) == HIGH); // warten bis Kabel wieder gesteckt ist.
            while (digitalRead(TEST_PIN_TX2) == HIGH); // warten bis Kabel wieder gesteckt ist.
            while (digitalRead(TEST_PIN_TX2) == HIGH); // warten bis Kabel wieder gesteckt ist. // PRELLSCHUTZ


            msys = 0;  // here: "msys" =  my system name - msys als Hilsvariable verwendet - statt robId

            while (digitalRead(TEST_PIN_TX2) == LOW)
            {
                msys++; if (msys > 7) msys = 0;
                
                sprintf(text,"rob%c",msys + '0');
                printRobName(text);

                watch = 1000; while (watch); // 100 ms wait time
            }

            storeStr2EEPROM(text, EEPROM_ROB_NAME);
            printStored();

            robName = readFromEEPROM(EEPROM_ROB_NAME);

            while (digitalRead(TEST_PIN_TX2) == HIGH); // warten bis Kabel wieder gesteckt ist.
            while (digitalRead(TEST_PIN_TX2) == HIGH); // warten bis Kabel wieder gesteckt ist.
            while (digitalRead(TEST_PIN_TX2) == HIGH); // warten bis Kabel wieder gesteckt ist. // PRELLSCHUTZ

            printf("----- calibrieren -------\n");

            leds[0] = CRGB{0, 0, 0}; // R B G
            leds[1] = CRGB{0, 0, 0};
            leds[2] = CRGB{0, 0, 0};
            leds[3] = CRGB{0, 0, 0};
            FastLED.show();

            while (digitalRead(TEST_PIN_TX2) == HIGH); // warten bis Kabel wieder gesteckt ist.

            drive(0,0);

            watch = 500; while (watch); 

            onBoardLedOn();
 
            count = 3;

            while (count > -1)
            {
                printCount(count); 
                watch = 1000; while (watch); 
                count--;
            }

            watch = 500; while (watch);

            count = 200; // wird jetzt als speed verwendet
            prepareMFSCalibrationSystem();  // setzt nur die Variablen für höchsten und minimalsten Wert

            while (count < 256) //255 letzter Wert!
            {
                drive(count, -count);
                watch = 300; while (watch);
                count += 5;
                calibrateMFS();
            }

            count = 255;
            impulsCntL = 0;
            impulsCntR = 0; 
            last_impulsCntL = -2; 
            last_impulsCntR = -2;

            while ((count > 200))  // von 255 bis 200
            {
                drive(-count, count);
                watch = 300; while (watch);
                count -= 5;
                calibrateMFS();

                /* wir könnten hier feststellen, wie langsam die Motoren noch arbeiten ...*/
                /*
                // - zunächst geht es hier nur um die Calibrierung des Magnetfeldsensors. 
                if (((impulsCntL - last_impulsCntL) > 1) && 
                    ((impulsCntR - last_impulsCntR) > 1)     )
                {
                    //printf("# %d %d \n", (impulsCntL - last_impulsCntL), (impulsCntR - last_impulsCntR));
                    last_impulsCntL = impulsCntL;
                    last_impulsCntR = impulsCntR;
                    drive(-count, count);
                    calibrateMFS();
                    printSpeedMin(count);
                    impulsFlagL = impulsFlagR = FALSE;
                    watch = 50; while (watch);  // kürzer !
                    if (count > 200) count -= 4; 
                    else if (count > 150) count -= 3;
                    else if (count > 120) count -= 2;
                    else count--;
                    // for tests:
                    printf("count min : %d %d %d %d %d\n", count, impulsCntL, impulsCntR,
                        impulsCntL - last_impulsCntL, impulsCntR - last_impulsCntR);
                }
                */
            }

            // minSpeed = count; 
            // printSpeedMin(minSpeed);
            // storeInt2EEPROM(minSpeed,   EEPROM_MIN_SPEED);

            drive(0,0);
            
            storeInt2EEPROM(xMin, EEPROM_MFS_MINX);     // derzeit noch nicht verwendet!
            storeInt2EEPROM(xMax, EEPROM_MFS_MAXX);     // derzeit noch nicht verwendet!
            storeInt2EEPROM(yMin, EEPROM_MFS_MINY);     // derzeit noch nicht verwendet!
            storeInt2EEPROM(yMax, EEPROM_MFS_MAXY);     // derzeit noch nicht verwendet!

            watch = 500; while (watch); 
            watch = 5000; while (watch)
            {
                angle = getMFS_Angle() - 83.;  // bp str.4
                printComp(angle);
            }
            


            printReset();

            while (digitalRead(TEST_PIN_TX2) == HIGH); // warten bis Kabel wieder gesteckt ist.
            while (digitalRead(TEST_PIN_TX2) == HIGH); // warten bis Kabel wieder gesteckt ist.
            while (digitalRead(TEST_PIN_TX2) == HIGH); // warten bis Kabel wieder gesteckt ist. // PRELLSCHUTZ


            printf("Reset - done");
            esp_restart();



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
static int newS = 0; 
static int oldS = 0;
static int newC = 0; 
static int oldC = 0;

int c; 
static int fled = 1; 
float vx, vy, wr, wl;

static char rxBuf[32];
static int  rxPos = 0;
int joyX = 0, joyY = 0;   // -128 .. +128


/*
    int SensorWert1 = analogRead(LDR1);
    int SensorWert2 = analogRead(LDR2);
    int SensorWert3 = analogRead(LDR3);
    int SensorWert4 = analogRead(LDR4);
*/

    switch (mode)
    {
        case MODE_PS4:
       
            if (tenMSecFlag)
            {
                tenMSecFlag = FALSE;
            
                if (connected)
                {
                    switch(psMode)
                    {
                        case DUAL_MODE:
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
                        break;

                        case JOY_STICK:

                            vx = PS4.LStickX();
                            vy = PS4.LStickY();
                            wl = atan(vy/+vx) * 180 / M_PI;
                            wr = atan(vy/-vx) * 180 / M_PI;
                            if (vx < 0) wl += 180.;
                            if (vx > 0) wr += 180.;

                            if ((abs(vx) > 7) || (abs(vy) > 7))
                            {
                                // speedL = PS4.R2Value();
                                if (wl > 0)
                                {
                                    speedL = (PS4.R2Value()/128.)*((255 - minSpeed)* (wl/270.)) 
                                             + minSpeed;
                                    dirL = 1;
                                }
                                else
                                {
                                    speedL = (PS4.R2Value()/128.)*((255 - minSpeed)*((-wl)/270.)) + minSpeed;
                                    dirL = -1;

                                }

                                if (wr > 0)
                                {
                                    speedR = (PS4.R2Value()/128.)*((255 - minSpeed)*(wr/270.)) + minSpeed;
                                    dirR = 1;
                                }
                                else
                                {
                                    speedR = (PS4.R2Value()/128.)*((255 - minSpeed)*((-wr)/270.))
                                     + minSpeed;
                                    dirR = -1;
                                }
                            }
                            else 
                            {
                                speedL = speedR = 0;
                            }

                            //printf("x %.1f y %.1f wl %.1f wr %.1f speedL %d\n", vx, vy, wl, wr, speedL);

                        break;
                    }
            
                    newT = PS4.Triangle();

                    if((newT != 0) && (oldT == 0))
                    {
                        psMode = (psMode == DUAL_MODE) ? JOY_STICK : DUAL_MODE;
                        squareMode = FALSE;  
                        switchLedsOn(ps, squareMode);
                        if (psMode == JOY_STICK) onBoardLedOff();
                        if (psMode == DUAL_MODE) onBoardLedOn();

                    }
                    oldT = newT;

                    newS = PS4.Square();
                    if((newS != 0) && (oldS == 0))
                    {
                        psMode = (psMode == DUAL_MODE) ? JOY_STICK : DUAL_MODE;
                        squareMode = TRUE;  
                        switchLedsOn(ps, squareMode);

                        if (psMode == JOY_STICK) onBoardLedOff();
                        if (psMode == DUAL_MODE) onBoardLedOn();

                    }
                    oldS = newS;

                    newC = PS4.Circle();
                    if((newC != 0) && (oldC == 0))
                    {
                        circleMode = (circleMode) ? 0:1;
                    }
                    oldC = newC;
                }
                else
                {  
                    if(PS4.isConnected())
                    {
                        connected = TRUE; 
                        onBoardLedOn();
                        printf("connected!\n");

                        switchLedsOn(ps, squareMode);
                    }
                }
            }

            if (qSecFlag)
            {
                qSecFlag = FALSE;

                // distance: 
                printData();

            }

            if (connected == TRUE)
            {
                drive(dirL * speedL,  dirR * speedR);
            }

        break;

        case MODE_BT:

            if (tenMSecFlag)
            {
                tenMSecFlag = FALSE;
                while (BT.available()) 
                {
                    c = BT.read();

                    if (c == '\n' || c == '\r') 
                    {
                        rxBuf[rxPos] = '\0';
                        rxPos = 0;
        
                         if (strcmp(rxBuf, "#RESET") == 0)
                        {
                            printf("Reset!\n");
                            
                            vLSum = vRSum = 0;

                            // hier könnte man den Autopiloten ein ausschalten!

                    continue;   // fertig mit dieser Zeile

                        }

                        // Format: Jx;y   z.B. J-10;25
                        if (rxBuf[0] == 'J') 
                        {
                            int x, y;
                            if (sscanf(&rxBuf[1], "%d;%d", &x, &y) == 2) 
                            {
                                joyX = x;    
                                joyY = y;

                                vx = joyX;
                                vy = joyY;
                                /*
                                wl = atan(vy/+vx) * 180 / M_PI;
                                wr = atan(vy/-vx) * 180 / M_PI;
                           
                                
                                if ((vx >= 0) && (vy >= 0)) { dirL = dirR = 1; wr += 180; }
                                if ((vx <  0) && (vy >= 0)) { dirL = dirR = 1; wl += 180; }
                                if ((vx <  0) && (vy <  0)) { dirL = dirR =-1; wl = - wl + 180; }
                                if ((vx >= 0) && (vy <  0)) { dirL = dirR =-1; wr = - wr + 180; }
                                
                                speedR = speedL = sqrt(vx*vx + vy*vy)*sqrt(2);

                                */

                                vy = -vy;

                                vy *= 2.;

                                if (vx > 0) vy -= vx; else vy += vx;

                                if ((abs(vx) > 7) || (abs(vy) > 7))
                                {
                                    speedL = speedR = vy;

                                    if(speedL > 0)
                                    {
                                        dirL = +1;
                                    }
                                    else
                                    {
                                        speedL = -speedL;
                                        dirL = -1;
                                    }
                            
                                    if(speedR > 0)
                                    {
                                        dirR = +1;
                                    }
                                    else
                                    {
                                        speedR = -speedR;
                                        dirR = -1;
                                    }
                                }
                            
//                                printf("BT JOY:  wl %.1f wr %.1f speedL %d speedR %d dirL %d  dirR %d\n", 
//                                    wl, wr, speedL, speedR, dirL, dirR);

                                drive(dirL * speedL,  dirR * speedR);
                                printf("BT JOY:  vx %f vy %f speedL %d speedR %d dirL %d  dirR %d\n", 
                                    vx, vy, speedL, speedR, dirL, dirR);


                            }
                        }
                    } 
                    else 
                    {
                        if (rxPos < (int)sizeof(rxBuf) - 1) 
                        {
                            rxBuf[rxPos++] = c;
                        }

                        if (strcmp(rxBuf, "#RESET") == 0)
                        {
                            printf("Reset!\n");
                        }
                    }
                }
            }

            if (qSecFlag)
            {
                qSecFlag = FALSE;

//        printf("prell: %d ldr1-4: %d %D %d %d\n", prell, SensorWert1, SensorWert2, SensorWert3, SensorWert4);
//        BT.write((uint8_t)prell + '0');

            // distance: 

                digitalWrite(TRIG_PIN, LOW);        delay(5);
                digitalWrite(TRIG_PIN, HIGH);       delay(5);
                digitalWrite(TRIG_PIN, LOW);        
                distance = pulseIn(ECHO_PIN, HIGH) / 58.23;   // durch 58.23 

                printData();
                BT.printf("%6.2f;%6.2f;%d;%d;%6.2f\n", distance, angle, vLSum, vRSum, batteryLevel);
                //printf("%6.2f;%6.2f;%d;%d;%6.2f\n", distance, angle, vLSum, vRSum, batteryLevel);


                if (autonomous)
                {
                    if(stateBT == DRIVE)
                    {
                        dirL = dirR = 1; 
                        

                        leds[0] = CRGB{255, 255, 0}; // R B G
                        leds[1] = CRGB{255, 255, 0};
                        leds[2] = CRGB{0, 0, 255};
                        leds[3] = CRGB{0, 0, 255};

                        FastLED.show();

                        
                        drive(255,255);
                        if (distance <= 10) {stateBT = ROTATE; drive(0,0);}
                        else if (distance <= 15) drive(100,100);
                        else if (distance <= 20) drive(120,120);
                        else if (distance <= 25) drive(150,150);
                        else if (distance <= 30) drive(200,200);
                        else if (distance <= 40) drive(220,220);
                        else if (distance <= 60) drive(250,250);
                       


                    }
                    else
                    {
                        drive(120, -120);
                        if (distance >= 30) stateBT = DRIVE;

                        leds[0] = CRGB{255, 0, 0}; // R B G
                        leds[1] = CRGB{255, 0, 0};
                        leds[2] = CRGB{255, 0, 0};
                        leds[3] = CRGB{255, 0, 0};

                        FastLED.show();
                    }
                }
            }

        break;

        case MODE_MENU:
            // do nothing! 
        break;
    }
}

void switchLedsOn(int ps, int sMode)
{
    if (sMode)
    {
        switch (ps)
        {
            case PS4_GRAY: leds[2] = CRGB{0, 0, 255}; leds[3] = CRGB{0, 0, 255}; break;
            case PS4_RED:  leds[2] = CRGB{255, 0, 0}; leds[3] = CRGB{255, 0, 0}; break;
            case PS4_BLUE: leds[2] = CRGB{0, 255, 0}; leds[3] = CRGB{0, 255, 0}; break;
        }   
        leds[0] = CRGB{0, 0, 0};
        leds[1] = CRGB{0, 0, 0};
    }
    else
    {
        switch (ps)
        {
            case PS4_GRAY: leds[0] = CRGB{0, 0, 255}; leds[1] = CRGB{0, 0, 255}; break;
            case PS4_RED:  leds[0] = CRGB{255, 0, 0}; leds[1] = CRGB{255, 0, 0}; break;
            case PS4_BLUE: leds[0] = CRGB{0, 255, 0}; leds[1] = CRGB{0, 255, 0}; break;
        }   
        leds[2] = CRGB{0, 0, 0};
        leds[3] = CRGB{0, 0, 0};
    }

    FastLED.show();

}


// Magnetic Field Sensor:

int initMFS(void)
{
int ret = 0;   
 
// RETURN: 
// 0 success; 
// 2 kein device gfunden; 
// 3 Fehler bei der Übertragung der Daten an den Baustein  

    Wire.beginTransmission(mfs);
    Wire.write(0x00); Wire.write(0x70); // Konfiguration A
    Wire.endTransmission();

    Wire.beginTransmission(mfs);
    Wire.write(0x01); Wire.write(0xA0); // Gain höher setzen
    Wire.endTransmission();

    Wire.beginTransmission(mfs);
    Wire.write(0x02); Wire.write(0x00); // Continuous mode
    ret = Wire.endTransmission();

    if (ret != 0)
    {
        mfs = 0x0d;

        // dann gibt es vielleicht noch den Baustein, der auf =X0d antwortet. 
  
        Wire.beginTransmission(mfs);
        Wire.write(0x0A); Wire.write(0x80); // Reset Register und Soft Reset
        Wire.endTransmission();

        delay(10);
        
        // Control Register 1
        Wire.beginTransmission(mfs);
        Wire.write(0x09);
        Wire.write(0b00011101);
        // 0b00 01 11 01
        // |  |  |  |
        // |  |  |  +--> Mode = Continuous (01)
        // |  |  +-----> ODR = 200 Hz (11)
        // |  +--------> Range = ±8 Gauss (01)
        // +-----------> OSR = 512 (00)
        Wire.endTransmission();

        // Control Register 2
        Wire.beginTransmission(mfs);
        Wire.write(0x0A);
        Wire.write(0x01);   // Roll Pointer
        ret = Wire.endTransmission();
    }

    return ret;
}

void prepareMFSCalibrationSystem(void)
{
    xMin = yMin = 32767;
    xMax = yMax = -32768;
}


void readRawMFS(int16_t* x, int16_t* y, int16_t* z)
{
    if (isMFSavailable)
    {
        if (mfs == MFS)  // 0x1E: 
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
        else
        {
            Wire.beginTransmission(mfs);
            Wire.write(0x00);
            Wire.endTransmission(false);
    
            Wire.requestFrom(mfs, 6);
            if (Wire.available() == 6)
            {
                uint8_t xl = Wire.read();
                uint8_t xh = Wire.read();
                uint8_t yl = Wire.read();
                uint8_t yh = Wire.read();
                uint8_t zl = Wire.read();
                uint8_t zh = Wire.read();

                *x = (int16_t)((xh << 8) | xl);
                *y = (int16_t)((yh << 8) | yl);
                *z = (int16_t)((zh << 8) | zl);
            }
            else
            {
                *x = *y = *z = 0;
            }
        }
    }
    else
    {
        *x = *y = *z = 0;
    }
}

int calibrateMFS(void)
{
    int ret = FALSE; 
    unsigned long lastSampleTime = 0;

    readRawMFS(&x, &y, &z);

    // Ausreißer sollten eliminiert werden... (-4600...) 

    if (x > xMax) { xMax = x; }
    if (x < xMin) { xMin = x; }
    if (y > yMax) { yMax = y; }
    if (y < yMin) { yMin = y; }    

    if ( ((xMax -xMin) > 180) && ((yMax -yMin) > 200)) ret = isCalibrated = TRUE; 


    //printf("|| x: %04d  %04d  xmin: %04d xmax: %04d ||  y: %04d  %04d ymin %04d ymax %04d || L: %04d R: %04d || ret: %d angle: %3.2f\n",
    //        x, (xMax - xMin), xMin, xMax, y, (yMax - yMin), yMin, yMax, impulsCntL, impulsCntR, ret, getMFS_Angle());


    return ret;
}

float getMFS_Angle()
{
    float xw, yw, angle;

    if (isMFSavailable)
    {
        readRawMFS(&x, &y, &z);

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
    }
    else
    angle = 0;

    // South = 0°
    // East = 90°
    // West = -90°
    // North = +-180°

    return angle;  
}

// OLED DISPLAY:
void printComp(float w)
{
    int x, y;

    x = 30 * cos(w * M_PI / 180.);
    y = 30 * sin(w * M_PI / 180.);

    oled.fillRect(0, 0, 128, 64, 0); // clear all!
    
    while ( w < 180.) w += 360.;
    while ( w > 180.) w -= 360.;

    sprintf(text,"w: %3.1f", w);
    oled.setCursor(0, 48);
    oled.print(text);
    oled.drawCircle(64, 32, 31, WHITE);
    oled.drawLine(64, 32, x + 64, y + 32, WHITE);
    oled.display();
}


void printSpeedMin(int i)
{
    oled.fillRect(0, 20, 128, 64, 0); // clear all!
    sprintf(text,"s-Min:%d",i);
    oled.setCursor(20, 32);
    oled.print(text);
    oled.display();
}

void printCount(int i)
{
    oled.fillRect(0, 20, 128, 64, 0); // clear all!
    sprintf(text,"in: %c sec", i  + '0');
    oled.setCursor(20, 32);
    oled.print(text);
    oled.display();
}

void printPs4(int ps)
{
    oled.fillRect(0, 20, 128, 64, 0); // clear all!
    sprintf(text,"ps: %s", (ps == PS4_GRAY) ? "gray" : 
                           (ps == PS4_RED) ? "red" : 
                           "blue");
    oled.setCursor(20, 32);
    oled.print(text);
    oled.display();
}

void printRobName(const String& name)
{
    oled.fillRect(0, 20, 128, 64, 0); // clear all!
    oled.setCursor(5, 32);
    oled.print(name);
    oled.display();
}

void printMotorSystem(int msys)
{
    oled.fillRect(0, 20, 128, 64, 0); // clear all!
    sprintf(text,"m-sys: %c", msys  + '0');
    oled.setCursor(10, 32);
    oled.print(text);
    oled.display();
}

void printWifiSystem(int msys)
{
    oled.fillRect(0, 20, 128, 64, 0); // clear all!
    sprintf(text,"wifi:%s", (wsys == 0) ? "home":"work");
    oled.setCursor(10, 32);
    oled.print(text);
    oled.display();
}

void printStored(void)
{
    sprintf(text,"stored!");
    oled.setCursor(20, 48);
    oled.print(text);
    oled.display();
}

void printReset(void)
{
    oled.fillRect(0, 20, 128, 64, 0); // clear all!
    sprintf(text,"reset ? ");
    oled.setCursor(10, 48);
    oled.print(text);
    oled.display();
}

void printData(void)
{
    int angleInt = 0;

    angle = getMFS_Angle() - 83.;

    if (circleMode)
    {
        printComp(angle);
    }
    else
    {
        batteryLevel = analogRead(BATTERY_LEVEL) / REFV;

        oled.fillRect(0, 0, 128, 64, 0); // clear all!

        sprintf(text,"%c: 0.00 V", (ps == PS4_GRAY)? 'G': (ps == PS4_RED)? 'R':'B');
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

/*
        digitalWrite(TRIG_PIN, LOW);        delay(5);
        digitalWrite(TRIG_PIN, HIGH);       delay(5);
        digitalWrite(TRIG_PIN, LOW);        
        distance = pulseIn(ECHO_PIN, HIGH) / 58.23;   // durch 58.23 
*/
        if (connected == 0)
        {
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
        }

    // oled.drawRect(10, 25, 40, 15, WHITE); // links, unten, breit, hoch
    // oled.drawLine(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT, WHITE);
    // oled.drawCircle(64, 32, 31, WHITE);
        oled.display();
    }
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


void storeInt2EEPROM(int16_t x, int address)
{
    uint8_t low  = x & 0xFF;
    uint8_t high = (x >> 8) & 0xFF;

    EEPROM.write(address,     low);    
    EEPROM.write(address + 1, high);    
    EEPROM.commit();  // Änderungen speichern
}

int16_t getIntFromEEPROM(int address)
{
    uint8_t low  = EEPROM.read(address);
    uint8_t high = EEPROM.read(address + 1);

    return (int16_t)(low | (high << 8));
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
    //storeStr2EEPROM("rob1", EEPROM_ROB_NAME);
    //storeInt2EEPROM(128,  EEPROM_BRIGHTNESS_LEVEL);     
    //storeStr2EEPROM("0",              EEPROM_WIFI_SYS_ADDR);     
    storeStr2EEPROM("A1-7DC69BC1",    EEPROM_WIFI_0_SSID_ADDR); 
    storeStr2EEPROM("HvCtieELY4tVFs", EEPROM_WIFI_0_PASS_ADDR);
    storeStr2EEPROM("10.0.0.229",     EEPROM_WIFI_0_IP_ADDR);
    storeStr2EEPROM("HTL-WLAN-IoT",   EEPROM_WIFI_1_SSID_ADDR); // das sind noch alte Daten... 
    storeStr2EEPROM("HTL2IoT!",       EEPROM_WIFI_1_PASS_ADDR);
    storeStr2EEPROM("10.115.61.237",  EEPROM_WIFI_1_IP_ADDR);

}

void getSet(void)
{
    byte high, low;
    robName = readFromEEPROM(EEPROM_ROB_NAME); 
    robId = (char)robName[3] - '0';
    minSpeed = getIntFromEEPROM(EEPROM_MIN_SPEED);
    motorSysFromEEPROM = readFromEEPROM(EEPROM_MOTOR_SYS_ADDR);
    motorSys = (char)motorSysFromEEPROM[0] - '0';

    motorSysFromEEPROM = readFromEEPROM(EEPROM_WIFI_SYS_ADDR);
    wifiSys = (char)motorSysFromEEPROM[0] - '0';
   
    ssidWord = (wifiSys == 0)? readFromEEPROM(EEPROM_WIFI_0_SSID_ADDR):readFromEEPROM(EEPROM_WIFI_1_SSID_ADDR);
    password = (wifiSys == 0)? readFromEEPROM(EEPROM_WIFI_0_PASS_ADDR):readFromEEPROM(EEPROM_WIFI_1_PASS_ADDR);
    ipNode   = (wifiSys == 0)? readFromEEPROM(EEPROM_WIFI_0_IP_ADDR)  :readFromEEPROM(EEPROM_WIFI_1_IP_ADDR);
   
    ps = getIntFromEEPROM(EEPROM_PS4_ADDR);
    xMin = getIntFromEEPROM(EEPROM_MFS_MINX);
    xMax = getIntFromEEPROM(EEPROM_MFS_MAXX);
    yMin = getIntFromEEPROM(EEPROM_MFS_MINY);
    yMax = getIntFromEEPROM(EEPROM_MFS_MAXY);
   
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
    impulsCntL++;
    vLSum += (dirL>0)? 1:-1;
}


void impuls_R_isr(void)
{
    static int x = 0; 
    impulsFlagR = TRUE; 
    impulsCntR++;
    vRSum += (dirR>0)? 1:-1;
}


//******************************************************************
//              T i m e r   I n t e r r u  p t : 
//      periodic timer interrupt, expires each 0.1 msec
//******************************************************************

void IRAM_ATTR myTimer(void)   
{
    static int32_t otick  = 0;
    static int32_t qtick = 0;
    static int32_t mtick = 0;
    static int x = 0; 
    
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
        if (ramp >= vL) digitalWrite(WHEEL_L, L);  else digitalWrite(WHEEL_L, H);
        if (ramp >= vR) digitalWrite(WHEEL_R, L);  else digitalWrite(WHEEL_R, H);
    }
    if (motorSys == 1) // System mit Zusatzprint:
    { 
        if (LDir) if (ramp <= vL) digitalWrite(WHEEL_L, L);  else digitalWrite(WHEEL_L, H);
        else      if (ramp >= vL) digitalWrite(WHEEL_L, L);  else digitalWrite(WHEEL_L, H);

        if (RDir) if (ramp <= vR) digitalWrite(WHEEL_R, L);  else digitalWrite(WHEEL_R, H);
        else      if (ramp >= vR) digitalWrite(WHEEL_R, L);  else digitalWrite(WHEEL_R, H);
    }
}