/* 

--------------------------------------  
 Author .    : Ed Nieuwenhuys
 Changes V001: Derived from ESP32C3S3_WordClockV026-V008
 Changes V002: Mislukt
 Changes V003: Swiping works 
 Changes V004:  
 Changes V005: Turn off display if watch not held horizontal for 250 msexs working
 Changes V008: Clean up coding
 Changes V009: Stable version without errors when compiler warnings set to All
 Changes V010: Mislukt
 Changes V011: Stable version. 
 
 ToDo:
Optimize Power management
Help scherm uitprinten
Store serial-print per dag op SD per maand for debugging purposes

How to compile: Install ESP32 boards
Set to:
Board: TTGO T-watch 
Board revision: T-watch-2020-V3
Patition Scheme: Default 2*6.5Mb 3.6MB SPIFFS
PSRAM disable (otherwise `iram0_0_seg' overflowed by x bytes 


// ESP32-C3-12F layout seen from top
// ADC   ----- I0 19
// IO 01 ----- I0 18
// NC    ----- I0 10
// IO 02 ----- I0 09
// IO 03 ----- I0 08
// IO 04 ----- 3V3  
// IO 05 ----- GND  
// NC    ----- NC   
// NC    ----- I0 06
// GND   ----- I0 07
// 3V3   ----- NC   
// EN    ----- RX   
// IO 00 ----- TX   
// GND   ----- GND  
// 5V    ----- 3V3  

ILI9341 connections
MISO  to GPIO 10
LED   to 3V3
SCK   to GPIO 09
MOSI  to GPIO 08  
DC/RS to GPIO 06
RESET to GPIO 07
CS    to GND
GND   to GND
3V3   to 3V3
 
Usage of:
1 Screen, colours and fonts
2 BLE nRF UART connection with phone
3 Buttons
4 RGB LED
5 RTC module for time
6 ??SD-card or SPIFFS storage of a struct with settings
7 Get timezone corrected time with daylight savings from a NTP server via WIFI
8 Menu driven with serial monitor, BLE and WIFI
9 Change connection settings of WIFI via BLE
10 Get time with NTP server via WIFI
11 OTA updates of the software by WIFI

Starting the monitor will result in the board to be unresponsive. This is due to the CTS and RTS levels of the serial interface. Disabling the control lines prevents the board to become unresponsive. Edit the file “boards.txt” from the definition of the board. The file is located in the following directory, where xxxxx is the user name: “C:\Users\xxxxx\AppData\Local\Arduino15\packages\esp32\hardware\esp32\2.0.2”
To get to this location, find c:\Users\ednie\AppData\Local\Arduino15\packages\esp32\hardware\esp32\(2.0.9 or higher version )\boards.txt

esp32c3.serial.disableDTR=false
esp32c3.serial.disableRTS=false
to
esp32c3.serial.disableDTR=true
esp32c3.serial.disableRTS=true    

or of you have an s3 board esp32s3.serial....
               
*/
// =============================================================================================================================

#define LILYGO_WATCH_2020_V3              // To use T-Watch2020 , please uncomment this line

//twatch.serial.disableDTR=false   // in boards.txt at the moment unchanged
//twatch.serial.disableRTS=false
//--------------------------------------------
// ESP32-C3 Includes defines and initialisations
//--------------------------------------------

#include <NimBLEDevice.h>      // For BLE communication  https://github.com/h2zero/NimBLE-Arduino
#include <ESPNtpClient.h>      // https://github.com/gmag11/ESPNtpClient
#include <WiFi.h>              // Used for web page 
#include <AsyncTCP.h>          // Used for webpage   https://github.com/me-no-dev/ESPAsyncWebServer
#include <ESPAsyncWebServer.h> // Used for webpage   https://github.com/me-no-dev/ESPAsyncWebServer
#include <AsyncElegantOTA.h>   // Used for OTA
#include <Preferences.h>
#include "Colors.h"

#include <LilyGoWatch.h>       // https://github.com/Xinyuan-LilyGO/TTGO_TWatch_Library
/*
// **********************   #include <LilyGoWatch.h>
// *************************  #include "board/twatch2020_v3.h"
#define TFT_WIDTH                   (240)
#define TFT_HEIGHT                  (240)

#define TWATCH_TFT_MISO             (GPIO_NUM_MAX)
#define TWATCH_TFT_MOSI             (GPIO_NUM_19)
#define TWATCH_TFT_SCLK             (GPIO_NUM_18)
#define TWATCH_TFT_CS               (GPIO_NUM_5)
#define TWATCH_TFT_DC               (GPIO_NUM_27)
#define TWATCH_TFT_RST              (GPIO_NUM_MAX)
#define TWATCH_TFT_BL               (GPIO_NUM_15)

#define TOUCH_SDA                   (GPIO_NUM_23)
#define TOUCH_SCL                   (GPIO_NUM_32)
#define TOUCH_INT                   (GPIO_NUM_38)
#define TOUCH_RST                   (GPIO_NUM_14)

#define SEN_SDA                     (GPIO_NUM_21)
#define SEN_SCL                     (GPIO_NUM_22)

#define RTC_INT_PIN                 (GPIO_NUM_37)
#define AXP202_INT                  (GPIO_NUM_35)
#define BMA423_INT1                 (GPIO_NUM_39)

#define TWATCH_2020_IR_PIN          (GPIO_NUM_13)


#define TWATCH_DAC_IIS_BCK          (GPIO_NUM_26)
#define TWATCH_DAC_IIS_WS           (GPIO_NUM_25)
#define TWATCH_DAC_IIS_DOUT         (GPIO_NUM_33)
// ********************************  in twatch2020_v3.h
#
// Has
#define LILYGO_WATCH_HAS_TOUCH
#define LILYGO_WATCH_HAS_DISPLAY
#define LILYGO_WATCH_HAS_MOTOR
#define LILYGO_WATCH_HAS_PCF8563
#define LILYGO_WATCH_HAS_BMA423
#define LILYGO_WATCH_HAS_AXP202
#define LILYGO_WATCH_HAS_IRREMOTE
#define LILYGO_WATCH_HAS_BACKLIGHT

// Hardware not support
#undef  LILYGO_WATCH_HAS_NFC
#undef  LILYGO_WATCH_HAS_GPRS
#undef  LILYGO_WATCH_HAS_LORA
#undef  LILYGO_WATCH_HAS_MPU6050
#undef  LILYGO_WATCH_HAS_MAX301XX
#undef  LILYGO_WATCH_HAS_BUTTON
#undef  LILYGO_WATCH_HAS_MPR121
#undef  LILYGO_WATCH_HAS_BBQ_KEYBOARD
#undef  LILYGO_WATCH_HAS_S76_S78G
#undef  LILYGO_WATCH_HAS_SIM800L
#undef  LILYGO_WATCH_HAS_SIM868
#undef  LILYGO_WATCH_AIR530_GPS
#undef  LILYGO_WATCH_HAS_SDCARD
#undef  LILYGO_WATCH_DRV2605
#include "TTGO.h"
// ************************************ in LilyGoWatch.h
*/
TTGOClass    *ttgo;                      
TFT_eSPI     *tft;
AXP20X_Class *power;
BMA          *sensor;

#define HET     ColorLeds("Het",     1,   3, MINColor);   
#define IS      ColorLeds("is",      5,   6, SECColor);    Is = true; 
#define WAS     ColorLeds("was",     8,  10, SECColor);    Is = false;
#define MVIJF   ColorLeds("vijf",    12, 15, LetterColor); 
#define PRECIES ColorLeds("precies", 17, 23, LetterColor);
#define MTIEN   ColorLeds("tien",    24, 27, LetterColor); 
#define KWART   ColorLeds("kwart",   31, 35, LetterColor);
#define VOOR    ColorLeds("voor",    38, 41, LetterColor);
#define OVER    ColorLeds("over",    43, 46, LetterColor);
#define HALF    ColorLeds("half",    48, 51, LetterColor);
#define MIDDER  ColorLeds("midder",  53, 58, LetterColor);
#define VIJF    ColorLeds("vijf",    60, 63, LetterColor);
#define TWEE    ColorLeds("twee",    65, 68, LetterColor);
#define EEN     ColorLeds("een",     72, 74, LetterColor);
#define VIER    ColorLeds("vier",    77, 80, LetterColor);
#define ELF     ColorLeds("elf",     81, 83, LetterColor);
#define TIEN    ColorLeds("tien",    84, 87, LetterColor);
#define TWAALF  ColorLeds("twaalf",  89, 94, LetterColor);
#define DRIE    ColorLeds("drie",    97,100, LetterColor);
#define NEGEN   ColorLeds("negen",  102,106, LetterColor);
#define ACHT    ColorLeds("acht",   109,112, LetterColor);
#define NACHT   ColorLeds("nacht",  118,112, LetterColor);
#define ZES     ColorLeds("zes",    114,116, LetterColor);
#define ZEVEN   ColorLeds("zeven",  120,124, LetterColor);
#define NOEN    ColorLeds("noen",   126,129, LetterColor);
#define UUR     ColorLeds("uur",    137,139, LetterColor);
//#define EDSOFT  ColorLeds("EdSoft", 132,132, LetterColor);
#define X_OFF   ColorLeds("",         0,  2, 0);
#define X_ON    ColorLeds("",         0,  2, LetterColor);

#define IT      ColorLeds("It",       1,   2, MINColor);   
#define ISUK    ColorLeds("is",       5,   6, SECColor);    Is = true;
#define WASUK   ColorLeds("was",      8,  10, SECColor);    Is = false;
#define EXACTUK ColorLeds("exact",   13,  17, LetterColor);
#define HALFUK  ColorLeds("half",    18,  21, LetterColor); 
#define TWENTY  ColorLeds("twenty",  24,  29, LetterColor); 
#define MFIVE   ColorLeds("five",    31,  34, LetterColor);
#define QUARTER ColorLeds("quarter", 37,  43, LetterColor);
#define MTEN    ColorLeds("ten",     44,  46, LetterColor);
#define PAST    ColorLeds("past",    48,  51, LetterColor);
#define TO      ColorLeds("to",      53,  54, LetterColor);
#define MID     ColorLeds("mid",     56,  58, LetterColor);
#define SIXUK   ColorLeds("six",     61,  63, LetterColor);
#define TWO     ColorLeds("two",     65,  67, LetterColor);
#define FIVE    ColorLeds("five",    68,  71, LetterColor);
#define TWELVE  ColorLeds("twelve",  74,  79, LetterColor);
#define TEN     ColorLeds("ten",     81,  83, LetterColor);
#define ELEVEN  ColorLeds("eleven",  85,  90, LetterColor);
#define FOUR    ColorLeds("four",    92,  95, LetterColor);
#define ONE     ColorLeds("one",     97,  99, LetterColor);
#define EIGHT   ColorLeds("eight",  101, 105, LetterColor);
#define THREE   ColorLeds("three",  109, 113, LetterColor);
#define NIGHT   ColorLeds("night",  114, 118, LetterColor);
#define NINE    ColorLeds("nine",   121, 124, LetterColor);
#define SEVEN   ColorLeds("seven",  125, 129, LetterColor);
#define OCLOCK  ColorLeds("O'clock",137, 143, LetterColor);

#define ES      ColorLeds("Es",       1,   2, MINColor);   
#define IST     ColorLeds("ist",      4,   6, SECColor);    Is = true;
#define WAR     ColorLeds("war",      7,   9, SECColor);    Is = false;
#define GENAU   ColorLeds("genau",   12,  16, LetterColor);
#define MZEHN   ColorLeds("zehn",    18,  21, LetterColor);
#define MFUNF   ColorLeds("funf",    25,  28, LetterColor);
#define VIERTEL ColorLeds("viertel", 29,  35, LetterColor);
#define ZWANZIG ColorLeds("zwanzig", 36,  42, LetterColor);
#define KURZ    ColorLeds("kurz",    44,  47, LetterColor);
#define VOR     ColorLeds("vor",     49,  51, LetterColor);
#define NACH    ColorLeds("nach",    52,  55, LetterColor);
#define HALB    ColorLeds("halb",    60,  63, LetterColor);
#define FUNF    ColorLeds("funf",    65,  68, LetterColor);
#define VIERDE  ColorLeds("vier",    75,  78, LetterColor);
#define ZWOLF   ColorLeds("zwolf",   79,  83, LetterColor);
#define EINS    ColorLeds("eins",    84,  87, LetterColor);
#define MITTER  ColorLeds("mitter",  89,  94, LetterColor);
#define ACHTDE  ColorLeds("acht",    97, 101, LetterColor);
#define NACHTDE ColorLeds("nacht",   96, 101, LetterColor);
#define DREI    ColorLeds("drei",   103, 106, LetterColor);
#define SECHS   ColorLeds("sechs",  108, 112, LetterColor);
#define SIEBEN  ColorLeds("sieben", 112, 117, LetterColor);
#define NEUN    ColorLeds("neun",   120, 123, LetterColor);
#define ZWEI    ColorLeds("zwei",   126, 129, LetterColor);
#define ZEHN    ColorLeds("zehn",   124, 127, LetterColor);
#define ELFDE   ColorLeds("elf",    134, 136, LetterColor);
#define UHR     ColorLeds("uhr",    140, 142, LetterColor);

#define IL      ColorLeds("Il",       1,   2, MINColor);   
#define EST     ColorLeds("est",      4,   6, SECColor);    Is = true;
#define ETAIT   ColorLeds("etait",    7,  11, SECColor);    Is = false;
#define EXACT   ColorLeds("exact",   13,  17, LetterColor);
#define SIX     ColorLeds("six",     18,  20, LetterColor); 
#define DEUX    ColorLeds("deux",    25,  28, LetterColor); 
#define TROIS   ColorLeds("trois",   30,  34, LetterColor);
#define ONZE    ColorLeds("onze",    36,  39, LetterColor);
#define QUATRE  ColorLeds("quatre",  41,  46, LetterColor);
#define MINUIT  ColorLeds("minuit",  48,  53, LetterColor);
#define DIX     ColorLeds("dix",     56,  58, LetterColor);
#define CINQ    ColorLeds("cinq",    60,  63, LetterColor);
#define NEUF    ColorLeds("neuf",    64,  67, LetterColor);
#define MIDI    ColorLeds("midi",    68,  71, LetterColor);
#define HUIT    ColorLeds("huit",    72,  75, LetterColor);
#define SEPT    ColorLeds("sept",    76,  79, LetterColor);
#define UNE     ColorLeds("une",     81,  83, LetterColor);
#define HEURE   ColorLeds("heure",   85,  89, LetterColor);
#define HEURES  ColorLeds("heures",  85,  90, LetterColor);
#define ET      ColorLeds("et",      93,  94, LetterColor);
#define MOINS   ColorLeds("moins",   99, 103, LetterColor);
#define LE      ColorLeds("le",     105, 106, LetterColor);
#define DEMI    ColorLeds("demie",  110, 114, LetterColor);
#define QUART   ColorLeds("quart",  120, 124, LetterColor);
#define MDIX    ColorLeds("dix",    128, 131, LetterColor);
#define VINGT   ColorLeds("vingt",  133, 137, LetterColor);
#define MCINQ   ColorLeds("cinq",   139, 142, LetterColor);
#define DITLEHEURE DitLeHeure();

//--------------------------------------------
// SPIFFS storage
//--------------------------------------------
Preferences FLASHSTOR;

 //--------------------------------------------
// COLOURS stored in Mem.DisplayChoice
//--------------------------------------------   

const byte DEFAULTCOLOUR = 0;
const byte HOURLYCOLOUR  = 1;          
const byte WHITECOLOR    = 2;
const byte OWNCOLOUR     = 3;
const byte OWNHETISCLR   = 4;
const byte WHEELCOLOR    = 5;
const byte DIGITAL       = 6;
const byte ANALOOG       = 7;

// Pin Assignments
#define  PhotoCellPin 2
#define  EMPTY01      1
#define  EMPTY00      0

#define  RGB_LED     38
#define  REDPIN       3
#define  GREENPIN     4
#define  BLUEPIN      5
#define  COLD_WHITE  18
#define  WARM_WHITE  19

//--------------------------------------------                                                //
// ESP32-C3 initialysations
//--------------------------------------------
bool     PrintDigital = false;                                                                // Show digital display 
bool     IP_Printed   = false;                                                                // The IP address will be printed on the display
bool     Date_Printed = false;                                                                // The date will be printed on the display  
char     Template[150];                                                                       // Contains the string with characters to be printed in the display
uint32_t LastButtonTime = 0;                                                                  // Used to avoid debouncing the button
uint32_t SecondsLeap = 0;                                                                     // Seconds counter
//------------------------------------------------------------------------------
// LDR PHOTOCELL
//------------------------------------------------------------------------------
//                                                                                            //
const byte SLOPEBRIGHTNESS  = 80;                                                             // Steepness of with luminosity of the LED increases
const int  MAXBRIGHTNESS    = 255;                                                            // Maximun value in bits  for luminosity of the LEDs (1 - 255)
const byte LOWBRIGHTNESS    = 5;                                                              // Lower limit in bits of Brightness ( 0 - 255)   
byte     TestLDR            = 0;                                                              // If true LDR inf0 is printed every second in serial monitor
int      OutPhotocell;                                                                        // stores reading of photocell;
int      MinPhotocell       = 999;                                                            // stores minimum reading of photocell;
int      MaxPhotocell       = 1;                                                              // stores maximum reading of photocell;
uint32_t SumLDRreadshour    = 0;
uint32_t NoofLDRreadshour   = 0;

//--------------------------------------------
// CLOCK initialysations
//--------------------------------------------                                 

static  uint32_t msTick;                                                                      // Number of millisecond ticks since we last incremented the second counter
byte    lastminute = 0, lasthour = 0, lastday = 0, sayhour = 0;
bool    Zelftest             = false;
bool    Is                   = true;                                                          // toggle of displaying Is or Was
bool    ZegUur               = true;                                                          // Say or not say Uur in NL clock
struct tm timeinfo;                                                                           // storage of time 

//--------------------------------------------                                                //
// SCREEN Pixel initialysations
//--------------------------------------------
const byte MATRIX_WIDTH     = 12;
const byte MATRIX_HEIGHT    = 12;
const byte NUM_LEDS         = MATRIX_WIDTH * MATRIX_HEIGHT ;        
struct     LEDPrintbuffer
            { 
             char Character;  
             uint32_t RGBColor;
            } Strippos[NUM_LEDS+1];
bool     LEDsAreOff            = false;         // If true LEDs are off except time display
bool     NoTextInColorLeds     = false;         // Flag to control printing of the text in function ColorLeds()
int      Previous_LDR_read     = 512;           // The actual reading from the LDR + 4x this value /5  
uint32_t MINColor              = C_YELLOW;      //C_RED;
uint32_t SECColor              = C_YELLOW;     //C_GREEN;
uint32_t LetterColor           = C_YELLOW;      
uint32_t HourColor[24] ={C_WHITE, C_RED, C_ORANGE, C_YELLOW,  C_YELLOW_GREEN, C_GREEN,
                         C_CYAN,  C_DODGER_BLUE, C_PURPLE, C_MAGENTA, C_GOLD, C_SPRING_GREEN,
                         C_WHITE, C_CHOCOLATE, C_ORANGE, C_KHAKI, C_YELLOW_GREEN, C_BEIGE,
                         C_AQUA_MARINE,  C_SKY_BLUE, C_HOT_PINK, C_DEEP_PINK, C_CORAL, C_LAWN_GREEN};  

//--------------------------------------------
// SCREEN COMMON
//--------------------------------------------
 #define TFT_LED    1                                                                      // the LED backlight display
 bool DisplayIsOn = true;

//--------------------------------------------
// SCREEN ST7789
//--------------------------------------------

uint8_t ScreenMiddleX = 120; //ttgo->tft->width()  / 2;            //   120;
uint8_t ScreenMiddleY = 120; //ttgo->tft->height() / 2;           //   120;

//--------------------------------------------                                                //
// BLE   //#include <NimBLEDevice.h>
//--------------------------------------------
BLEServer *pServer = NULL;
BLECharacteristic * pTxCharacteristic;
bool deviceConnected    = false;
bool oldDeviceConnected = false;
std::string ReceivedMessageBLE;

#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"                         // UART service UUID
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

//----------------------------------------
// WEBSERVER
//----------------------------------------
int WIFIConnected = 0;              // Is wIFI connected?
AsyncWebServer server(80);          // For OTA Over the air uploading
#include "Webpage.h"
//----------------------------------------
// NTP
//----------------------------------------
boolean syncEventTriggered = false;                                                           // True if a time even has been triggered
NTPEvent_t ntpEvent;                                                                          // Last triggered event
//----------------------------------------
// Common
//--------------------------------------------                                                //
 uint64_t Loopcounter = 0;
#define MAXTEXT 140
char sptext[MAXTEXT];                                                                         // For common print use 
bool SerialConnected = true;   
struct EEPROMstorage {                                                                        // Data storage in EEPROM to maintain them after power loss
  byte DisplayChoice    = 0;
  byte TurnOffLEDsAtHH  = 0;
  byte TurnOnLEDsAtHH   = 0;
  byte LanguageChoice   = 0;
  byte LightReducer     = 0;
  int  LowerBrightness  = 0;
  int  UpperBrightness  = 0;
  int  NVRAMmem[24];                                                                          // LDR readings
  byte BLEOnOff         = 1;
  byte NTPOnOff         = 1;
  byte WIFIOnOff        = 1;  
  byte StatusLEDOnOff   = 1;
  int  ReconnectWIFI    = 0;                                                                  // No of times WIFI reconnected 
  byte UseSDcard        = 0;
  byte WIFINoOfRestarts = 0;                                                                  // If 1 than resart MCU once
  byte DisplayOnForSecs = 0;                                                                  // Keeps display on for .. seconds. If 0 keeps display state unchanged
  byte ByteFuture2      = 0;                                                                  // For future use
  byte ByteFuture3      = 0;                                                                  // For future use
  byte ByteFuture4      = 0;                                                                  // For future use
  int  IntFuture1       = 0;                                                                  // For future use
  int  IntFuture2       = 0;                                                                  // For future use
  int  IntFuture3       = 0;                                                                  // For future use
  int  IntFuture4       = 0;                                                                  // For future use   
  byte UseBLELongString = 0;                                                                  // Send strings longer than 20 bytes per message. Possible in IOS app BLEserial Pro 
  uint32_t OwnColour    = 0;                                                                  // Self defined colour for clock display
  uint32_t DimmedLetter = 0;
  uint32_t BackGround   = 0;
  char Ssid[30];                                                                             // 
  char Password[40];                                                                         // 
  char BLEbroadcastName[30];                                                                 // Name of the BLE beacon
  char Timezone[50];
  int  Checksum        = 0;
}  Mem; 

 
//  -------------------------------------   End Definitions  ---------------------------------------

//--------------------------------------------                                                //
// ARDUINO Setup
//--------------------------------------------
void setup() 
{
 Serial.begin(115200);                                                                        // Setup the serial port to 115200 baud // 
 //function takes the following frequencies as valid values:
//  240, 160, 80    <<< For all XTAL types
//  40, 20, 10      <<< For 40MHz XTAL
//  26, 13          <<< For 26MHz XTAL
//  24, 12          <<< For 24MHz XTAL
// setCpuFrequencyMhz(80);
 SetupWatch();                                                                               // Setup watch
 ttgo->motor_begin(); 
//    int i = 1;
//    do {  ttgo->motor->onec();        delay(500);
//    } while (i--);
 Tekstprintln("Serial started");
 ledcSetup(0, 5000, 8);        //ledChannel, freq, resolution);                               // configure LED PWM functionalitites 5000 Hz, 8 bits
 ledcAttachPin(TFT_LED, 0);  //ledPin, ledChannel);                                           // ILI9341 backlight on ledchannel 0 and GPIO pin TFT_LED

 Previous_LDR_read = ReadLDR();
 InitStorage();                           Tekstprintln("Setting loaded");                     // Load settings from storage and check validity 
 InitDisplay();                           Tekstprintln("Display started");
 if (Mem.BLEOnOff) StartBLEService();     Tekstprintln("BLE started");                        // Start BLE service
 if(Mem.WIFIOnOff){ Tekstprintln("Starting WIFI");  StartWIFI_NTP();   }                      // Start WIFI and optional NTP if Mem.WIFIOnOff = 1                                                                                          // Mem.DisplayChoice == DIGITAL?PrintDigital=true:PrintDigital=false; 
 SWversion();                                                                                 // Print the menu + version 
 GetTijd(0);                                                                                  // 
 Print_RTC_tijd();
 ClearScreen(); 
 if (Mem.DisplayChoice == ANALOOG)   AnalogClockSetup(); 
 else Displaytime();                                                                          // Print the tekst time in the display 
 msTick = LastButtonTime = millis(); 
}
//--------------------------------------------                                                //
// Common Loop
//--------------------------------------------
void loop() 
{
 Loopcounter++;                                                                               // Check is loop is run >500 times per second
 EverySecondCheck();                                                                          // Start of program with check at specific times intervals
 CheckDevices();                                                                              // Check several input devices as fast as possible otherwise once per second, minute, hour,  et cetera
}
//--------------------------------------------                                                //     sprintf(sptext,"aac.x: %d aac.y %d aac.z" %d",acc.x,acc.y,acc.z);     Tekstprintln(sptext); 
// Common Check connected input devices
//--------------------------------------------
void CheckDevices(void)
{
 CheckBLE();                                                                                  // Something with BLE to do?
 SerialCheck();                                                                               // Check serial port every second 
// ButtonsCheck();                                                                            // Check if buttons pressed
 ReworkSwipeOrTouch(DisplayTouchOrSwipe());                                                   // Check for display touches             
}

//--------------------------------------------                                                //
// CLOCK Update routine done every second
//--------------------------------------------
void EverySecondCheck(void)
{
 uint32_t msLeap;
 msLeap = millis() - msTick;                                                                  // uint32_t msLeapButton = millis()- LastButtonTime;
 if (msLeap >999)                                                                             // Every second enter the loop
  {    
   msTick = millis();
   SecondsLeap++;                                                                             // Second time for watch operation
   CheckDisplayOnOrOff();
   GetTijd(0);                                                                                // Update timeinfo.tm_sec, timeinfo.tm_min, timeinfo.tm_hour, timeinfo.tm_mday, timeinfo.tm_mon, timeinfo.tm_year
   if(!LEDsAreOff)    ScreenSecondProcess();                                                  // Draw seconds in digital and analogue display mode
   DimLeds(TestLDR);                                                                          // Every second an intensitiy check and update from LDR reading 
   if (timeinfo.tm_min != lastminute) EveryMinuteUpdate();                                    // Enter the every minute routine after one minute; 
   Loopcounter = 0;                                                                           // Counter that counts how many time the loop is looped per second. Shown with option K in the menu
  }  
 }
//--------------------------------------------
// CLOCK Update routine done every minute
//-------------------------------------------- 
void EveryMinuteUpdate(void)
{   
 lastminute = timeinfo.tm_min;  
 if(!LEDsAreOff)  Displaytime();                                                              // Turn the display on                 
// Print_tijd();                                                                              // sprintf(sptext,"NTP:%s", NTP.getTimeDateString());   Tekstprintln(sptext);  
 //  if(WiFi.localIP()[0]>0) WIFIConnected = true; else WIFIConnected = false;                  // To be sure connection is still there
 if(timeinfo.tm_hour != lasthour) EveryHourUpdate();
}
//--------------------------------------------
// CLOCK Update routine done every hour
//--------------------------------------------
void EveryHourUpdate(void)
{
 if(WiFi.localIP()[0]==0)
   {
     sprintf(sptext, "Disconnected from station, attempting reconnection");
     Tekstprintln(sptext);
     WiFi.reconnect();
   }
 lasthour = timeinfo.tm_hour;
 if(timeinfo.tm_hour == Mem.TurnOffLEDsAtHH){ LEDsAreOff = true;  ClearScreen();    }         // Is it time to turn off the LEDs?
 if(timeinfo.tm_hour == Mem.TurnOnLEDsAtHH) { LEDsAreOff = false; RestartDisplay(); }         // Redraw the outer rings of the analog clock if necessary 
 Mem.NVRAMmem[lasthour] =(byte)((SumLDRreadshour / NoofLDRreadshour?NoofLDRreadshour:1));     // Update the average LDR readings per hour
 SumLDRreadshour  = 0;
 NoofLDRreadshour = 0;
 if (timeinfo.tm_mday != lastday) EveryDayUpdate();  
}
//--------------------------------------------                                                //
// CLOCK Update routine done every day
//------------------------------------------------------------------------------
void EveryDayUpdate(void)
{
 if(timeinfo.tm_mday != lastday) 
   {
    lastday = timeinfo.tm_mday; 
    Previous_LDR_read = ReadLDR();                                                            // to have a start value
    MinPhotocell      = Previous_LDR_read;                                                    // Stores minimum reading of photocell;
    MaxPhotocell      = Previous_LDR_read;                                                    // Stores maximum reading of photocell;
//    StoreStructInFlashMemory();                                                             // 
    }
}
//--------------------------------------------
// Common check for serial input
//--------------------------------------------
void SerialCheck(void)
{
 String SerialString; 
 while (Serial.available())
    { 
     char c = Serial.read();                                                                  // Serial.write(c);
     if (c>31 && c<128) SerialString += c;                                                    // Allow input from Space - Del
     else c = 0;                                                                              // Delete a CR
    }
 if (SerialString.length()>0) 
    {
     ReworkInputString(SerialString+"\n");                                                    // Rework ReworkInputString();
     SerialString = "";
    }
}
//--------------------------------------------                                                //
// Common Reset to default settings
//------------------------------------------------------------------------------
void Reset(void)
{
 Mem.Checksum         = 25065;                                                                //
 Mem.DisplayChoice    = DEFAULTCOLOUR;                                                        // Default colour scheme 
 Mem.OwnColour        = C_ORANGE;                                                             // Own designed colour.
 Mem.DimmedLetter     = C_DIM_GRAY;
 Mem.BackGround       = C_BLACK; 
 Mem.LanguageChoice   = 4;                                                                    // 0 = NL, 1 = UK, 2 = DE, 3 = FR, 4 = Wheel
 Mem.LightReducer     = SLOPEBRIGHTNESS;                                                      // Factor to dim ledintensity with. Between 0.1 and 1 in steps of 0.05
 Mem.UpperBrightness  = MAXBRIGHTNESS;                                                        // Upper limit of Brightness in bits ( 1 - 1023)
 Mem.LowerBrightness  = LOWBRIGHTNESS;                                                        // Lower limit of Brightness in bits ( 0 - 255)
 Mem.TurnOffLEDsAtHH  = 0;                                                                    // Display Off at nn hour
 Mem.TurnOnLEDsAtHH   = 0;                                                                    // Display On at nn hour Not Used
 Mem.BLEOnOff         = 1;                                                                    // BLE On
 Mem.UseBLELongString = 0;
 Mem.NTPOnOff         = 0;                                                                    // NTP On
 Mem.WIFIOnOff        = 0;                                                                    // WIFI On  
 Mem.ReconnectWIFI    = 0;                                                                    // Correct time if necesaary in seconds
 Mem.WIFINoOfRestarts = 0;                                                                    //  
 Mem.DisplayOnForSecs = 6;                                                                    // Keeps display on for .. seconds. If 0 keeps display state unchanged
 Mem.UseSDcard        = 0;
 Previous_LDR_read    = ReadLDR();                                                            // Read LDR to have a start value. max = 4096/8 = 255
 MinPhotocell         = Previous_LDR_read;                                                    // Stores minimum reading of photocell;
 MaxPhotocell         = Previous_LDR_read;                                                    // Stores maximum reading of photocell;                                            
 TestLDR              = 0;                                                                    // If true LDR display is printed every second
 
 strcpy(Mem.Ssid,"");                                                                         // Default SSID
 strcpy(Mem.Password,"");                                                                     // Default password
 strcpy(Mem.BLEbroadcastName,"TTGOWatch");
 strcpy(Mem.Timezone,"CET-1CEST,M3.5.0,M10.5.0/3");                                           // Central Europe, Amsterdam, Berlin etc.
 // For debugging
// strcpy(Mem.Ssid,"Guest");
// strcpy(Mem.Password,"guest_001");
// Mem.NTPOnOff        = 1;                                                                     // NTP On
// Mem.WIFIOnOff       = 1;                                                                     // WIFI On  

 Tekstprintln("**** Reset of preferences ****"); 
 StoreStructInFlashMemory();                                                                  // Update Mem struct       
 GetTijd(0);                                                                                  // Get the time and store it in the proper variables
 SWversion();                                                                                 // Display the version number of the software
 //Displaytime();
}
//--------------------------------------------                                                //
// Common common print routines
//--------------------------------------------
void Tekstprint(char const *tekst)    { if(Serial) Serial.print(tekst);  SendMessageBLE(tekst);sptext[0]=0;   } 
void Tekstprintln(char const *tekst)  { sprintf(sptext,"%s\n",tekst); Tekstprint(sptext);  }
void TekstSprint(char const *tekst)   { printf(tekst); sptext[0]=0;}                          // printing for Debugging purposes in serial monitor 
void TekstSprintln(char const *tekst) { sprintf(sptext,"%s\n",tekst); TekstSprint(sptext); }

//------------------------------------------------------------------------------
//  Common Constrain a string with integers
// The value between the first and last character in a string is returned between the low and up bounderies
//------------------------------------------------------------------------------
int SConstrainInt(String s,byte first,byte last,int low,int up){return constrain(s.substring(first, last).toInt(), low, up);}
int SConstrainInt(String s,byte first,          int low,int up){return constrain(s.substring(first).toInt(), low, up);}
//--------------------------------------------                                                //
// Common Init and check contents of EEPROM
//--------------------------------------------
void InitStorage(void)
{
 // if(!SPIFFS.begin(FORMAT_SPIFFS_IF_FAILED)){ Tekstprintln("Card Mount Failed");   return;}
 // else Tekstprintln("SPIFFS mounted"); 

 GetStructFromFlashMemory();
 if( Mem.Checksum != 25065)
   {
    sprintf(sptext,"Checksum (25065) invalid: %d\n Resetting to default values",Mem.Checksum); 
    Tekstprintln(sptext); 
    Reset();                                                                                  // If the checksum is NOK the Settings were not set
   }
 Mem.LanguageChoice  = _min(Mem.LanguageChoice, 4);                                           // Constrain the value to valid ranges 
 Mem.DisplayChoice   = _min(Mem.DisplayChoice, ANALOOG);                                      // Constrain the value to valid ranges 
 if(Mem.OwnColour == 0) Mem.OwnColour = C_GREEN;                                              // If memory is empty cq black colour then store default value, blue  
 Mem.LightReducer    = constrain(Mem.LightReducer,1,250);                                     // 
 Mem.LowerBrightness = constrain(Mem.LowerBrightness, 1, 250);                                // 
 Mem.UpperBrightness = _min(Mem.UpperBrightness, 255); 
 if(strlen(Mem.Password)<5 || strlen(Mem.Ssid)<3)     Mem.WIFIOnOff = Mem.NTPOnOff = 0;       // If ssid or password invalid turn WIFI/NTP off
 
 StoreStructInFlashMemory();
}
//--------------------------------------------                                                //
// COMMON Store mem.struct in FlashStorage or SD
// Preferences.h  
//--------------------------------------------
void StoreStructInFlashMemory(void)
{
  FLASHSTOR.begin("Mem",false);       //  delay(100);
  FLASHSTOR.putBytes("Mem", &Mem , sizeof(Mem) );
  FLASHSTOR.end();          
  
// Can be used as alternative
//  SPIFFS
//  File myFile = SPIFFS.open("/MemStore.txt", FILE_WRITE);
//  myFile.write((byte *)&Mem, sizeof(Mem));
//  myFile.close();
 }
//--------------------------------------------
// COMMON Get data from FlashStorage
// Preferences.h
//--------------------------------------------
void GetStructFromFlashMemory(void)
{
 FLASHSTOR.begin("Mem", false);
 FLASHSTOR.getBytes("Mem", &Mem, sizeof(Mem) );
 FLASHSTOR.end(); 

// Can be used as alternative if no SD card
//  File myFile = SPIFFS.open("/MemStore.txt");  FILE_WRITE); myFile.read((byte *)&Mem, sizeof(Mem));  myFile.close();

 sprintf(sptext,"Mem.Checksum = %d",Mem.Checksum);Tekstprintln(sptext); 
}
//--------------------------------------------                                                //
//  CLOCK Input from swipe or touch display
//--------------------------------------------
void ReworkSwipeOrTouch(uint32_t Action)
{  if (Action == 0) return;
   sptext[0]=0;                                                                              // Make the string empty. Prevents also unnecesary save in StoreStructinFlash
   switch (Action)
          {
    case 1:  DisplayWakeUp();                                                                // Touch  Wake up the display
             break;
    case 2:  Mem.DisplayChoice++;                                                            // To Left
             if(Mem.DisplayChoice>ANALOOG) Mem.DisplayChoice = 0;                            // Effectively 0 but in case: 2 one is subtracted
             sprintf(sptext,"Display choice: Q%d", Mem.DisplayChoice);
             lastminute = 99;                                                                 // Down Force a minute update
             ClearScreen();
             if (Mem.DisplayChoice == ANALOOG) { AnalogClockSetup(); }                       //PrintAnalog display
             break;               
    case 3:  if(Mem.DisplayChoice == 0) Mem.DisplayChoice = ANALOOG+1;                       // To Right
             Mem.DisplayChoice--;
             sprintf(sptext,"Display choice: Q%d", Mem.DisplayChoice);
             lastminute = 99;                                                                 // Down Force a minute update
             ClearScreen();
             if (Mem.DisplayChoice == ANALOOG) { AnalogClockSetup(); }                       // PrintAnalog display
             break;  
    case 4: ReworkInputString("L0\n");                                                       // Down   
            sprintf(sptext,"L0");   
                        break;                                         
    case 5: ReworkInputString("L4\n");                                                       // Up  
            sprintf(sptext,"L4");
                        break;   
   default: break;
          }
if(sptext[0]>0) { Tekstprintln(sptext); StoreStructInFlashMemory();   }
}
//--------------------------------------------                                                //
//  CLOCK Input from Bluetooth or Serial
//--------------------------------------------
void ReworkInputString(String InputString)
{
 char ff[50];  InputString.toCharArray(ff,InputString.length());                              // Convert a String to char array
 sprintf(sptext,"Inputstring: %s  Lengte : %d\n", ff,InputString.length()-1); 
 // Tekstprint(sptext);
 if(InputString.length()> 40){Serial.printf("Input string too long (max40)\n"); return;}                                                         // If garbage return
 sptext[0] = 0;                                                                               // Empty the sptext string
 if(InputString[0] > 31 && InputString[0] <127)                                               // Does the string start with a letter?
  { 
  switch (InputString[0])
   {
    case 'A':
    case 'a': 
            if (InputString.length() >5 )
            {
             InputString.substring(1).toCharArray(Mem.Ssid,InputString.length()-1);
             sprintf(sptext,"SSID set: %s", Mem.Ssid);  
            }
            else sprintf(sptext,"**** Length fault. Use between 4 and 30 characters ****");
            break;
    case 'B':
    case 'b': 
           if (InputString.length() >5 )
            {  
             InputString.substring(1).toCharArray(Mem.Password,InputString.length()-1);
             sprintf(sptext,"Password set: %s\n Enter @ to reset ESP32 and connect to WIFI and NTP", Mem.Password); 
             Mem.NTPOnOff        = 1;                                                         // NTP On
             Mem.WIFIOnOff       = 1;                                                         // WIFI On  
            }
            else sprintf(sptext,"%s,**** Length fault. Use between 4 and 40 characters ****",Mem.Password);
            break;   
    case 'C':
    case 'c': 
           if (InputString.length() >5 )
            {  
             InputString.substring(1).toCharArray(Mem.BLEbroadcastName,InputString.length()-1);
             sprintf(sptext,"BLE broadcast name set: %s", Mem.BLEbroadcastName); 
             Mem.BLEOnOff        = 1;                                                         // BLE On
            }
            else sprintf(sptext,"**** Length fault. Use between 4 and 30 characters ****");
            break;      
    case 'D':
    case 'd':  
            if (InputString.length() == 10 )
              {
               timeinfo.tm_mday = (int) SConstrainInt(InputString,1,3,0,31);
               timeinfo.tm_mon  = (int) SConstrainInt(InputString,3,5,0,12) - 1; 
               timeinfo.tm_year = (int) SConstrainInt(InputString,5,9,2000,9999) - 1900;
               Date_Printed = false;                                                            // Date will be printed in Digital display mode
               SetRTCTime();
               Print_tijd();  //Tekstprintln(sptext);                                           // Print_Tijd() fills sptext with time string
               break;
              }

            if (InputString[1] == 'O' || InputString[1] == 'o' )                                // If entered DOxx where xxx is seconds display stays on
               {
                 Mem.DisplayOnForSecs = InputString.substring(2,5).toInt();                     
                 sprintf(sptext,"Display stays on for %d seconds", Mem.DisplayOnForSecs);
                 if(Mem.DisplayOnForSecs == 0) sprintf(sptext,"Display stays on forever");
                 break; 
               }
            sprintf(sptext,"****\nLength fault. \n****"); 
            break;  
    case 'F':
    case 'f':  
             if (InputString.length() == 8 )                                                      // 
               {
                LetterColor = Mem.OwnColour = HexToDec(InputString.substring(1,7));               // Display letter color 
                sprintf(sptext,"Font colour stored: 0X%06X", Mem.OwnColour);
                Tekstprintln("**** Own colour changed ****");    
                Displaytime();
               }
             else sprintf(sptext,"****Length fault. Enter Frrggbb hexadecimal (0 - F)****\nStored: 0X%06X", Mem.OwnColour);              
             break;
    case 'G':
    case 'g':  
             if (InputString.length() == 8 )
               {
                Mem.DimmedLetter = HexToDec(InputString.substring(1,7));               // Display letter color 
                sprintf(sptext,"Dimmed colour stored; 0X%06X", Mem.DimmedLetter);
                Tekstprintln("**** Dimmed font colour changed ****");    
                Displaytime();
               }
             else sprintf(sptext,"****Length fault. Enter Grrggbb hexadecimal (0 - F)****\nStored: 0X%06X", Mem.DimmedLetter);            
             break;
    case 'H':
    case 'h':  
             if (InputString.length() == 8 )
               {
                Mem.BackGround = HexToDec(InputString.substring(1,7));               // Display letter color 
                sprintf(sptext,"Own colour stored: 0X%06X", Mem.BackGround);
                Tekstprintln("**** BackGround colour changed ****");    
                Displaytime();
               }
             else sprintf(sptext,"****Length fault. Enter Hrrggbb hexadecimal (0 - F)****\nStored: 0X%06X", Mem.BackGround);           
             break;            
    case 'I':
    case 'i': 
            SWversion();
            break;
    case 'K':
    case 'k':
             TestLDR = 1 - TestLDR;                                                           // If TestLDR = 1 LDR reading is printed every second instead every 30s
             sprintf(sptext,"TestLDR: %s \nLDR reading, min and max of one day, %%Out, loops per second and time",TestLDR? "On" : "Off");
             break;      
    case 'L':                                                                                   // Language to choose
    case 'l':
             if (InputString.length() == 3 )
               {
                byte res = (byte) InputString.substring(1,2).toInt();   
                Mem.LanguageChoice = res%5;                       // Result between 0 and 4
                byte ch = Mem.LanguageChoice;                
                sprintf(sptext,"Language choice:%s",ch==0?"NL":ch==1?"UK":ch==2?"DE":ch==3?"FR":ch==4?"Rotate language":"NOP"); 
//                Tekstprintln(sptext);
                lastminute = 99;                                  // Force a minute update
               }
             else sprintf(sptext,"****\nDisplay choice length fault. Enter L0 - L4\n****"); 
            break;     

    case 'N':
    case 'n':
             if (InputString.length() == 2 )         Mem.TurnOffLEDsAtHH = Mem.TurnOnLEDsAtHH = 0;
             if (InputString.length() == 6 )
              {
               Mem.TurnOffLEDsAtHH =(byte) InputString.substring(1,3).toInt(); 
               Mem.TurnOnLEDsAtHH = (byte) InputString.substring(3,5).toInt(); 
              }
             Mem.TurnOffLEDsAtHH = _min(Mem.TurnOffLEDsAtHH, 23);
             Mem.TurnOnLEDsAtHH  = _min(Mem.TurnOnLEDsAtHH, 23); 
             sprintf(sptext,"Display is OFF between %2d:00 and %2d:00", Mem.TurnOffLEDsAtHH,Mem.TurnOnLEDsAtHH );
 //            Tekstprintln(sptext); 
             break;
    case 'O':
    case 'o':
             if(InputString.length() == 2)
               {
                LEDsAreOff = !LEDsAreOff;
                sprintf(sptext,"Display is %s", LEDsAreOff?"OFF":"ON" );
                if(LEDsAreOff) { ClearScreen();}                                                  // Turn the display off
                else 
                 {
                  RestartDisplay();
                  Displaytime();                                                               // Turn the display on                
                 }
               }
             break;                                                                   
    case 'p':
    case 'P':  
             if(InputString.length() == 2)
               {
                Mem.StatusLEDOnOff = !Mem.StatusLEDOnOff;
//                SetStatusLED(0,0,0,0,0); 
                sprintf(sptext,"StatusLEDs are %s", Mem.StatusLEDOnOff?"ON":"OFF" );               
               }
             break;        

    case 'q':
    case 'Q':  
             if (InputString.length() == 3 )
               {
                IP_Printed   = false;                                                           // The IP address will be printed on the display
                Date_Printed = false;                                                           // The date will be printed on the display  
                Mem.DisplayChoice = (byte) InputString.substring(1,2).toInt(); 
                if(Mem.DisplayChoice>ANALOOG) Mem.DisplayChoice = 0;
                sprintf(sptext,"Display choice: Q%d", Mem.DisplayChoice);
                lastminute = 99;                                                                 // Force a minute update
                ClearScreen();
                if (Mem.DisplayChoice == ANALOOG) { AnalogClockSetup(); }                       //PrintAnalog )  
               }
             else sprintf(sptext,"**** Display choice length fault. Enter Q0 - Q7"); 
         //    Displaytime();                                             // Turn on the display with proper time 
            break;
    case 'R':
    case 'r':
             if (InputString.length() == 2)
               {   
                Reset();
                sprintf(sptext,"\nReset to default values: Done");
                Displaytime();                                          // Turn on the display with proper time
               }                                
             else sprintf(sptext,"**** Length fault. Enter R ****");
             break;      
    case 'S':                                                                                 // factor ( 0 - 1) to multiply brighness (0 - 255) with 
    case 's':
            if (InputString.length() < 6)
               {    
                Mem.LightReducer = (byte) SConstrainInt(InputString,1,1,255);
                sprintf(sptext,"Slope brightness changed to: %d%%",Mem.LightReducer);
               }
              break;                    
    case 'T':
    case 't':
//                                                                                            //
             if(InputString.length() == 8)  // T125500
               {
                timeinfo.tm_hour = (int) SConstrainInt(InputString,1,3,0,23);
                timeinfo.tm_min  = (int) SConstrainInt(InputString,3,5,0,59); 
                timeinfo.tm_sec  = (int) SConstrainInt(InputString,5,7,0,59);
                SetRTCTime();
                Print_tijd(); 
               }
             else sprintf(sptext,"**** Length fault. Enter Thhmmss ****");
             break;            
    case 'U':                                                                                 // factor to multiply brighness (0 - 255) with 
    case 'u':
            if (InputString.length() < 6)
               {    
                Mem.UpperBrightness = SConstrainInt(InputString,1,1,255);
                sprintf(sptext,"Upper brightness changed to: %d bits",Mem.UpperBrightness);
               }
              break;  
    case 'V':
    case 'v':  
             if (InputString.length() < 6)
               {      
                Mem.LowerBrightness = (byte) SConstrainInt(InputString,1,0,255);
                sprintf(sptext,"Lower brightness: %d bits",Mem.LowerBrightness);
               }
             break;      
    case 'W':
    case 'w':
             if (InputString.length() == 2)
               {   
                Mem.WIFIOnOff = 1 - Mem.WIFIOnOff; 
                Mem.ReconnectWIFI = 0;                                                       // Reset WIFI reconnection counter 
                sprintf(sptext,"WIFI is %s after restart", Mem.WIFIOnOff?"ON":"OFF" );
               }                                
             else sprintf(sptext,"**** Length fault. Enter W ****");
             break; 
    case 'X':
    case 'x':
             if (InputString.length() == 2)
               {   
                Mem.NTPOnOff = 1 - Mem.NTPOnOff; 
                sprintf(sptext,"NTP is %s after restart", Mem.NTPOnOff?"ON":"OFF" );
               }                                
             else sprintf(sptext,"**** Length fault. Enter X ****");
             break; 
    case 'Y':
    case 'y':
             if (InputString.length() == 2)
               {   
                Mem.BLEOnOff = 1 - Mem.BLEOnOff; 
                sprintf(sptext,"BLE is %s after restart", Mem.BLEOnOff?"ON":"OFF" );
               }                                
             else sprintf(sptext,"**** Length fault. Enter Y ****");
             break; 
    case 'Z':
    case 'z':
             if (InputString.length() == 2)
               {   
                Mem.UseBLELongString = 1 - Mem.UseBLELongString; 
                sprintf(sptext,"Fast BLE is %s", Mem.UseBLELongString?"ON":"OFF" );
               }                                
             else sprintf(sptext,"**** Length fault. Enter U ****");
             break; 
        
    case '@':
             if (InputString.length() == 2)
               {   
               Tekstprintln("\n*********\n ESP restarting\n*********\n");
                ESP.restart();   
               }                                
             else sprintf(sptext,"**** Length fault. Enter @ ****");
             break;     
    case '#':
             if (InputString.length() == 2)
               {   
                Zelftest = 1 - Zelftest; 
                sprintf(sptext,"Zelftest: %d",Zelftest);
                Displaytime();                                          // Turn on the display with proper time
               }                                
             else sprintf(sptext,"**** Length fault. Enter S ****");
             break; 
    case '$':
             if (InputString.length() == 2)
               {   
                Mem.UseSDcard = 1 - Mem.UseSDcard; 
                sprintf(sptext,"SD is %s after restart", Mem.UseSDcard?"used":"NOT used" );
                sprintf(sptext,"This function is not working, .... yet" );
               }                                
             else sprintf(sptext,"**** Length fault. Enter Z ****");
             break; 
    case '&':                                                            // Get NTP time
             if (InputString.length() == 2)
              {
               NTP.getTime();                                           // Force a NTP time update      
               Tekstprint("NTP query started. \nClock time: ");
               delay(500);          
               GetTijd(1);
               Tekstprint("\n  NTP time: ");
               PrintNTP_tijd();
               Tekstprintln("");
               } 
             break;
    case '0':
    case '1':
    case '2':        
             if (InputString.length() == 7 )                                                  // For compatibility input with only the time digits
              {
               timeinfo.tm_hour = (int) SConstrainInt(InputString,0,2,0,23);
               timeinfo.tm_min  = (int) SConstrainInt(InputString,2,4,0,59); 
               timeinfo.tm_sec  = (int) SConstrainInt(InputString,4,6,0,59);
               sprintf(sptext,"Time set");  
               SetRTCTime();
               Print_RTC_tijd(); 
               } 
    default: break;
    }
  }  
 Tekstprintln(sptext); 
 StoreStructInFlashMemory();                                                                   // Update EEPROM                                     
 InputString = "";
}
//--------------------------------------------
// LDR reading are between 0 and 255. 
// ESP32 analogue read is between 0 - 4096 --   is: 4096 / 8
//--------------------------------------------
int ReadLDR(void)
{
  return analogRead(PhotoCellPin)/16;
}

//--------------------------------------------                                                //
//  LED Dim the leds measured by the LDR and print values
// LDR reading are between 0 and 255. The Brightness send to the LEDs is between 0 and 255
//--------------------------------------------
void DimLeds(bool print) 
{ 
  int LDRread = ReadLDR();                                                                    // ESP32 analoge read is between 0 - 4096, reduce it to 0-1024                                                                                                   
  int LDRavgread = (4 * Previous_LDR_read + LDRread ) / 5;                                    // Read lightsensor and avoid rapid light intensity changes
  Previous_LDR_read = LDRavgread;                                                             // by using the previous reads
  OutPhotocell = (uint32_t)((Mem.LightReducer * sqrt(255*LDRavgread))/100);                   // Linear --> hyperbolic with sqrt. Result is between 0-255
  MinPhotocell = min(MinPhotocell, LDRavgread);                                               // Lowest LDR measurement
  MaxPhotocell = max(MaxPhotocell, LDRavgread);                                               // Highest LDR measurement
  OutPhotocell = constrain(OutPhotocell, Mem.LowerBrightness, Mem.UpperBrightness);           // Keep result between lower and upper boundery en calc percentage
  SumLDRreadshour += LDRavgread;    NoofLDRreadshour++;                                       // For statistics LDR readings per hour
  if(print)
  {
//   sprintf(sptext,"LDR:%3d Avg:%3d (%3d-%3d) Out:%3d=%2d%% Loop(%ld) ",
//        LDRread,LDRavgread,MinPhotocell,MaxPhotocell,OutPhotocell,(int)(OutPhotocell/2.55),Loopcounter);    
   sprintf(sptext,"LDR:%3d (%3d-%3d) %2d%% %5lld l/s ",
        LDRread,MinPhotocell,MaxPhotocell,(int)(OutPhotocell/2.55),Loopcounter);   
   Tekstprint(sptext);
   Print_tijd();  
  }
 if(LEDsAreOff) OutPhotocell = 0;
 SetBrightnessLeds(OutPhotocell);     // values between 0 and 255
}

//--------------------------------------------
//  LED Set color for LED. 
// Fill the struct Strippos with the proper character and its colour
//--------------------------------------------
void ColorLeds(char const *Texkst, int FirstLed, int LastLed, uint32_t RGBColor)
{ 
 int n, i=0;
 char Tekst[strlen(Texkst)+10];
  if (!NoTextInColorLeds && (strlen(Texkst) > 0 && strlen(Texkst) <10) )
     {sprintf(sptext,"%s ",Texkst); Tekstprint(sptext); }                                     // Print the text  
                                                                                              // sprintf(sptext," %s, F:%d, L:%d F-L:%d ",Texkst, FirstLed,LastLed,1+LastLed-FirstLed );  Tekstprint(sptext);
 strcpy(Tekst,Texkst);
 to_upper(Tekst);   
 for (n=FirstLed; n<=FirstLed + (LastLed-FirstLed); n++)
 {
  Strippos[n].Character = Tekst[i++];
  Strippos[n].RGBColor = RGBColor;                                                            // Every character has its color stored here
//  sprintf(sptext,"-Strippos[%d].Character=:%c",n, Strippos[n].Character); Tekstprint(sptext);   
 }
}
//                                                                                            //
//--------------------------------------------
//  COMMON String upper
//--------------------------------------------
void to_upper(char* string)
{
 const char OFFSET = 'a' - 'A';
 while (*string)
  {
   (*string >= 'a' && *string <= 'z') ? *string -= OFFSET : *string;
   string++;
  }
}
//--------------------------------------------                                                //
//  LED Set brightness of backlight
//------------------------------------------------------------------------------  
void SetBrightnessLeds(byte Bright)
{
 SetBackLight(Bright);                                                                        // Set brightness of LEDs   
}
//--------------------------------------------
//  LED Clear the character string
//--------------------------------------------
void LedsOff(void) 
{ 
 for (int n=0; n<NUM_LEDS; n++) 
     Strippos[n].Character = Strippos[n].RGBColor = 0;                                        // Erase the struct Strippos
}
//------------------------------------------------------------------------------
//  LED Wheel
//  Input a value 0 to 255 to get a color value.
//  The colours are a transition r - g - b - back to r.
//------------------------------------------------------------------------------
uint32_t Wheel(byte WheelPos) 
{
 WheelPos = 255 - WheelPos;
 if(WheelPos < 85)   { return FuncCRGBW( 255 - WheelPos * 3, 0, WheelPos * 3, 0);  }
 if(WheelPos < 170)  { WheelPos -= 85;  return FuncCRGBW( 0,  WheelPos * 3, 255 - WheelPos * 3, 0); }
 WheelPos -= 170;      
 return FuncCRGBW(WheelPos * 3, 255 - WheelPos * 3, 0, 0);
}
//------------------------------------------------------------------------------
//  LED function to make RGBW color
//------------------------------------------------------------------------------ 
uint32_t FuncCRGBW( uint32_t Red, uint32_t Green, uint32_t Blue, uint32_t White)
{ 
 return ( (White<<24) + (Red << 16) + (Green << 8) + Blue );
}
//------------------------------------------------------------------------------
//  LED functions to extract RGBW colors
//------------------------------------------------------------------------------ 
 uint8_t Cwhite(uint32_t c) { return (c >> 24);}
 uint8_t Cred(  uint32_t c) { return (c >> 16);}
 uint8_t Cgreen(uint32_t c) { return (c >> 8); }
 uint8_t Cblue( uint32_t c) { return (c);      }
//--------------------------------------------
//  LED Set second color
//  Set the colour per second of 'IS' and 'WAS'
//--------------------------------------------
void SetSecondColour(void)
{
 switch (Mem.DisplayChoice)
  {
   case DEFAULTCOLOUR: LetterColor = C_YELLOW;
                       MINColor =    C_YELLOW;   // C_GREEN + ((timeinfo.tm_min/2)<<19);
                       SECColor=     C_YELLOW;   // C_GREEN + ((timeinfo.tm_hour/2)<<19);      // (30 << 19 = 0XF00000         
                                                                                              break;
   case HOURLYCOLOUR : LetterColor = MINColor = SECColor = HourColor[timeinfo.tm_hour];       break;    // A colour every hour
   case WHITECOLOR   : LetterColor = MINColor = SECColor = C_WHITE;                           break;    // All white
   case OWNCOLOUR    : LetterColor = MINColor = SECColor = Mem.OwnColour;                     break;    // Own colour
   case OWNHETISCLR  : LetterColor = Mem.OwnColour; 
                       MINColor = C_YELLOW;
                       SECColor = C_YELLOW;                                                   break;    // Own colour except HET IS WAS  
   case WHEELCOLOR   : LetterColor = MINColor = SECColor = Wheel((timeinfo.tm_min*4));        break;    // Colour of all letters changes per minute //(17*(timeinfo.tm_min*60)); 
   case DIGITAL      : LetterColor = C_WHITE; MINColor = SECColor = C_BLACK;                  break;    // Digital display of time. No IS WAS turn color off in display
   case ANALOOG      : LetterColor = C_YELLOW; MINColor = SECColor = C_BLACK;                 break;    //
  }
}
//--------------------------------------------                                                //
// CLOCK Version info
//--------------------------------------------
void SWversion(void) 
{ 
 #define FILENAAM (strrchr(__FILE__, '\\') ? strrchr(__FILE__, '\\') + 1 : __FILE__)
//--------------------------------------------                                                //
// Menu
//0        1         2         3         4
//1234567890123456789012345678901234567890----  
 char Menu[][40] = {
 "A SSID B Password C BLE beacon name",
 "D Date (D15012023) T Time (T132145)",
 "E Timezone  (E<-02>2 or E<+01>-1)",
 "  Make own colour of:  (Hex RRGGBB)",
 "F Font  G Dimmed font H Bkgnd",
 "I To print this Info menu",
 "K LDR reads/sec toggle On/Off", 
 "L L0 = NL, L1 = UK, L2 = DE",
 "  L3 = FR, L4 = Wheel",
 "N Display off between Nhhhh (N2208)",
 "O Display toggle On/Off",
 "P Status LED toggle On/Off", 
 "Q Display colour choice      (Q0-7)",
 "  Q0 Yellow  Q1 hourly",
 "  Q2 White   Q3 All Own",
 "  Q4 Own     Q5 Wheel",
 "  Q6 Digital Q7 Analog",
 "R Reset settings @ = Reset MCU",
 "--Light intensity settings (1-250)--",
 "S=Slope V=Min  U=Max   (S80 V5 U200)",
 "W=WIFI  X=NTP& Y=BLE  Z=Fast BLE", 
 "Ed Nieuwenhuys Aug 2023" };
 
 
 PrintLine(35);
 for (uint8_t i = 0; i < sizeof(Menu) / sizeof(Menu[0]); Tekstprintln(Menu[i++]));
 PrintLine(35);
 byte ch = Mem.LanguageChoice;
 byte dp = Mem.DisplayChoice;
 sprintf(sptext,"Display off between: %02dh - %02dh",Mem.TurnOffLEDsAtHH, Mem.TurnOnLEDsAtHH);  Tekstprintln(sptext);
 sprintf(sptext,"Display choice: %s",dp==0?"Yellow":dp==1?"Hourly":dp==2?"White":
                      dp==3?"All Own":dp==4?"Own":dp==5?"Wheel":dp==6?"Digital":dp==7?"Analog":"NOP");      Tekstprintln(sptext);
 sprintf(sptext,"Slope: %d     Min: %d     Max: %d ",Mem.LightReducer, Mem.LowerBrightness,Mem.UpperBrightness);  Tekstprintln(sptext);
 sprintf(sptext,"SSID: %s", Mem.Ssid);                                           Tekstprintln(sptext);
 
// sprintf(sptext,"Password: %s", Mem.Password);                                       Tekstprintln(sptext);
 sprintf(sptext,"BLE name: %s", Mem.BLEbroadcastName);                               Tekstprintln(sptext);
 sprintf(sptext,"IP-address: %d.%d.%d.%d (/update)", WiFi.localIP()[0], WiFi.localIP()[1], WiFi.localIP()[2], WiFi.localIP()[3] );  Tekstprintln(sptext);
 sprintf(sptext,"Timezone:%s", Mem.Timezone);                                                Tekstprintln(sptext); 
 sprintf(sptext,"%s %s %s %s", Mem.WIFIOnOff?"WIFI=On":"WIFI=Off", 
                               Mem.NTPOnOff? "NTP=On":"NTP=Off",
                               Mem.BLEOnOff? "BLE=On":"BLE=Off",
                               Mem.UseBLELongString? "FastBLE=On":"FastBLE=Off" );           Tekstprintln(sptext);
 sprintf(sptext,"Language choice: %s",
         ch==0?"NL":ch==1?"UK":ch==2?"DE":ch==3?"FR":ch==4?"Rotate language":"NOP");         Tekstprintln(sptext);
 sprintf(sptext,"Freqency (MHz): CPU %d, XTAL %d, APB %d", getCpuFrequencyMhz(), getXtalFrequencyMhz(), getApbFrequency()/1000000);  Tekstprintln(sptext); 
 sprintf(sptext,"Software: %s",FILENAAM);                                                    Tekstprintln(sptext);  //VERSION); 
 Print_RTC_tijd(); Tekstprintln(""); 
 PrintLine(35);
}
void PrintLine(byte Lengte)
{
 for(int n=0; n<Lengte; n++) sptext[n]='_';
 sptext[Lengte] = 0;
 Tekstprintln(sptext);
}
//--------------------------------------------                                                //
// CLOCK Say the time and load the LEDs 
// with the proper colour and intensity
//--------------------------------------------
void Displaytime(void)
{ 
 SetSecondColour();
 if (Mem.DisplayChoice == DIGITAL) return;                                                    // PrintDigital) 
 if (Mem.DisplayChoice == ANALOOG) return;                                                    // PrintAnalog)  

 byte Language;                                                                               // Start by clearing the display to a known state 

 if(Mem.LanguageChoice == 4)  Language = random(4);
   else                       Language = Mem.LanguageChoice;
 switch(Language)                                                                             //
     {
      case 0: 
      strncpy(Template,"SHETVISOWASOVIJFQPRECIESTIENKPFKWARTTSVOORSOVERAHALFSMIDDERTVIJFATWEESOEEENOXVIERELFTIENKTWAALFBHDRIECNEGENANACHTFZESSENZEVENSNOENVUQUOTEUURFRTT",145);  
              ColorLeds(Template,0,143, Mem.DimmedLetter);
              Dutch(); Print_tijd();   break;
      case 1: 
      strncpy(Template,"HITVISOWASOPEXACTHALFITSTWENTYESFIVEPQUARTERTENTPASTATOLMIDTKSIXVTWOFIVEEETWELVEXTENNELEVENEFOURTONETEIGHTEEHTHREENIGHTKFNINESEVENRHFUNNYO'CLOCK",145);  
              ColorLeds(Template,0,143, Mem.DimmedLetter);
              English(); Print_tijd(); break;
      case 2: 
      strncpy(Template,"GESTISTWARSEGENAUTZEHNFUGFUNFVIERTELZWANZIGTKURZAVORNACHYHACHALBKFUNFBOENSEVIERZWOLFEINSRMITTERDNACHTNDREISESECHSIEBENIJNEUNZEHNTELFVAELFWXKUHRO",145);
              ColorLeds(Template,0,143, Mem.DimmedLetter);
              German(); Print_tijd();  break;
      case 3:
      strncpy(Template,"WILWESTETAITGEXACTSIXLAUDEUXTTTROISOONZEHQUATRERMINUITXTDIXCCINQNEUFMIDIHUITSEPTVUNEBHEURESYEETFTOSMOINSELETKIDEMIENEZQMQUARTHTNDIXTOVINGTNCINQS",145); 
               ColorLeds(Template,0,143, Mem.DimmedLetter);
              French(); Print_tijd();  break;
     }
      ShowChars(); 
}

/*
      case 0: 
      strncpy(Template,"SHETVISOWASO
                        VIJFQPRECIES
                        TIENKPFKWART
                        TSVOORSOVERA
                        HALFSMIDDERT
                        VIJFATWEESOE
                        EENOXVIERELF
                        TIENKTWAALFB
                        HDRIECNEGENA
                        NACHTFZESSEN
                        ZEVENSNOENVU
                        QUOTEUURFRTT",145);  
              ColorLeds(Template,0,143, Mem.DimmedLetter);
              Dutch(); Print_tijd();   break;
      case 1: 
      strncpy(Template,"HITVISOWASOP
                        EXACTHALFITS
                        TWENTYESFIVE
                        PQUARTERTENT
                        PASTATOLMIDT
                        KSIXVTWOFIVE
                        EETWELVEXTEN
                        NELEVENEFOUR
                        TONETEIGHTEE
                        HTHREENIGHTK
                        FNINESEVENRH
                        FUNNYO'CLOCK",145);  
              ColorLeds(Template,0,143, Mem.DimmedLetter);
              English(); Print_tijd(); break;
      case 2: 
      strncpy(Template,"GESTISTWARSE
                        GENAUTZEHNFU
                        GFUNFVIERTEL
                        ZWANZIGTKURZ
                        AVORNACHYHAC
                        HALBKFUNFBOE
                        NSEVIERZWOLF
                        EINSRMITTERD
                        NACHTNDREISE
                        SECHSIEBENIJ
                        NEUNZEHNTELF
                        VAELFWXKUHRO",145);
              ColorLeds(Template,0,127, Mem.DimmedLetter); 
              German(); Print_tijd();  break;
      case 3:
      strncpy(Template,"WILWESTETAIT
                        GEXACTSIXLAU
                        DEUXTTTROISO
                        ONZEHQUATRER
                        MINUITXTDIXC
                        CINQNEUFMIDI
                        HUITSEPTVUNE
                        BHEURESYEETF
                        TOSMOINSELET
                        KIDEMIENEZQM
                        QUARTHTNDIXT
                        OVINGTNCINQS",145); 
              ColorLeds(Template,0,127, Mem.DimmedLetter);
              French(); Print_tijd();  break;
     }

 */
//--------------------------- Time functions --------------------------
//--------------------------------------------                                                //
// RTC Get time from NTP cq RTC 
// and store it in timeinfo struct
//--------------------------------------------
void GetTijd(byte printit)
{
 if(Mem.NTPOnOff)  getLocalTime(&timeinfo);                                                    // if NTP is running get the loacal time
else 
{ 
      time_t now;
      time(&now);
      localtime_r(&now, &timeinfo);
    } 
 if (printit)  Print_RTC_tijd();                                                              // otherwise time in OK struct timeonfo
}
//--------------------------------------------                                                //
// RTC prints time to serial
//--------------------------------------------
void Print_RTC_tijd(void)
{
 sprintf(sptext,"%02d/%02d/%04d %02d:%02d:%02d ", 
     timeinfo.tm_mday,timeinfo.tm_mon+1,timeinfo.tm_year+1900,
     timeinfo.tm_hour,timeinfo.tm_min,timeinfo.tm_sec);
 Tekstprint(sptext);
}
//--------------------------------------------
// NTP print the NTP time for the timezone set 
//--------------------------------------------
void PrintNTP_tijd(void)
{
 sprintf(sptext,"%s  ", NTP.getTimeDateString());  
 Tekstprint(sptext);              // 17/10/2022 16:08:15

// int dd,mo,yy,hh,mm,ss=0;      // nice method to extract the values from a string into vaiabeles
// if (sscanf(sptext, "%2d/%2d/%4d %2d:%2d:%2d", &dd, &mo, &yy, &hh, &mm, &ss) == 6) 
//     {      sprintf(sptext,"%02d:%02d:%02d %02d-%02d-%04d", hh,mm,ss,dd,mo,yy);      Tekstprintln(sptext); }
}

//--------------------------------------------
// NTP print the NTP UTC time 
//--------------------------------------------
void PrintUTCtijd(void)
{
 time_t tmi;
 struct tm* UTCtime;
 time(&tmi);
 UTCtime = gmtime(&tmi);
 sprintf(sptext,"UTC: %02d:%02d:%02d %02d-%02d-%04d  ", 
     UTCtime->tm_hour,UTCtime->tm_min,UTCtime->tm_sec,
     UTCtime->tm_mday,UTCtime->tm_mon+1,UTCtime->tm_year+1900);
 Tekstprint(sptext);   
}

//--------------------------------------------
// NTP processSyncEvent 
//--------------------------------------------
void processSyncEvent (NTPEvent_t ntpEvent) 
{
 switch (ntpEvent.event) 
    {
        case timeSyncd:
        case partlySync:
        case syncNotNeeded:
        case accuracyError:
            sprintf(sptext,"[NTP-event] %s\n", NTP.ntpEvent2str (ntpEvent));
            Tekstprint(sptext);
            break;
        default:
            break;
    }
}
//--------------------------------------------                                                //
// RTC Fill sptext with time
//--------------------------------------------
void Print_tijd(){ Print_tijd(2);}                                                            // print with linefeed
void Print_tijd(byte format)
{
 sprintf(sptext,"%02d:%02d:%02d",timeinfo.tm_hour,timeinfo.tm_min,timeinfo.tm_sec);
 switch (format)
 {
  case 0: break;
  case 1: Tekstprint(sptext); break;
  case 2: Tekstprintln(sptext); break;  
 }
}

//--------------------------------------------
// RTC Set time from global timeinfo struct
// Check if values are within range
// setDateTime(info.tm_year + 1900, info.tm_mon + 1, info.tm_mday, info.tm_hour, info.tm_min, info.tm_sec);
// setdatetime and settimeofday are system calls in Unix-like operating systems that allow 
// the user to set the system’s date and timesettimeofday is a more powerful version
// of setdatetime that allows the user to set the system’s time with microsecond precision
//--------------------------------------------
void SetRTCTime(void)
{ 
 time_t t = mktime(&timeinfo);
 sprintf(sptext, "Setting time: %s", asctime(&timeinfo)); Tekstprintln(sptext);
 struct timeval now = { .tv_sec = t , .tv_usec = t};
 settimeofday(&now, NULL);
// GetTijd(0);                                                                                  // Synchronize time with RTC clock
 Displaytime();
// Print_tijd();
}

//                                                                                            //
// ------------------- End  Time functions 

//--------------------------------------------
//  CLOCK Convert Hex to uint32
//--------------------------------------------
uint32_t HexToDec(String hexString) 
{
 uint32_t decValue = 0;
 int nextInt;
 for (uint8_t i = 0; i < hexString.length(); i++) 
  {
   nextInt = int(hexString.charAt(i));
   if (nextInt >= 48 && nextInt <= 57)  nextInt = map(nextInt, 48, 57, 0, 9);
   if (nextInt >= 65 && nextInt <= 70)  nextInt = map(nextInt, 65, 70, 10, 15);
   if (nextInt >= 97 && nextInt <= 102) nextInt = map(nextInt, 97, 102, 10, 15);
   nextInt = constrain(nextInt, 0, 15);
   decValue = (decValue * 16) + nextInt;
  }
 return decValue;
}
//--------------------------------------------                                                //
//  CLOCK Dutch clock display
//--------------------------------------------
void Dutch(void)
{
HET;                                                                                          // HET  is always on
 if (timeinfo.tm_hour == 12 && timeinfo.tm_min == 0 && random(2)==0) { IS; NOEN; return; }
 if (timeinfo.tm_hour == 00 && timeinfo.tm_min == 0 && random(2)==0) { IS; MIDDER; NACHT; return; } 
switch (timeinfo.tm_min)
 {
  case  0: IS;  PRECIES; break;
  case  1: IS;  break;
  case  2: 
  case  3: WAS; break;
  case  4: 
  case  5: 
  case  6: IS;  MVIJF; OVER; break;
  case  7: 
  case  8: WAS; MVIJF; OVER; break;
  case  9: 
  case 10: 
  case 11: IS;  MTIEN; OVER; break;
  case 12: 
  case 13: WAS; MTIEN; OVER; break;
  case 14: 
  case 15: 
  case 16: IS;  KWART; OVER; break;
  case 17: 
  case 18: WAS; KWART; OVER; break;
  case 19: 
  case 20: 
  case 21: IS;  MTIEN; VOOR; HALF; break;
  case 22: 
  case 23: WAS; MTIEN; VOOR; HALF; break;
  case 24: 
  case 25: 
  case 26: IS;  MVIJF; VOOR; HALF; break;
  case 27: 
  case 28: WAS; MVIJF; VOOR; HALF; break;
  case 29: IS;  HALF; break;
  case 30: IS;  PRECIES; HALF; break;
  case 31: IS;  HALF; break;
  case 32: 
  case 33: WAS; HALF; break;
  case 34: 
  case 35: 
  case 36: IS;  MVIJF; OVER; HALF; break;
  case 37: 
  case 38: WAS; MVIJF; OVER; HALF; break;
  case 39: 
  case 40: 
  case 41: IS;  MTIEN; OVER; HALF; break;
  case 42: 
  case 43: WAS; MTIEN; OVER; HALF; break;
  case 44: 
  case 45: 
  case 46: IS;  KWART; VOOR; break;
  case 47: 
  case 48: WAS; KWART; VOOR; break;
  case 49: 
  case 50: 
  case 51: IS;  MTIEN; VOOR;  break;
  case 52: 
  case 53: WAS; MTIEN; VOOR;  break;
  case 54: 
  case 55: 
  case 56: IS;  MVIJF; VOOR; break;
  case 57: 
  case 58: WAS; MVIJF; VOOR; break;
  case 59: IS;  break;
}
//if (timeinfo.tm_hour >=0 && hour <12) digitalWrite(AMPMpin,0); else digitalWrite(AMPMpin,1);

 sayhour = timeinfo.tm_hour;
 if (timeinfo.tm_min > 18 )  sayhour = timeinfo.tm_hour+1;
 if (sayhour == 24) sayhour = 0;

switch (sayhour)
 {
  case 13:  
  case 1: EEN; break;
  case 14:
  case 2: TWEE; break;
  case 15:
  case 3: DRIE; break;
  case 16:
  case 4: VIER; break;
  case 17:
  case 5: VIJF; break;
  case 18:
  case 6: ZES; break;
  case 19:
  case 7: ZEVEN; break;
  case 20:
  case 8: ACHT; break;
  case 21:
  case 9: NEGEN; break;
  case 22:
  case 10: TIEN; break;
  case 23:
  case 11: ELF; break;
  case 0:
  case 12: TWAALF; break;
 } 
 switch (timeinfo.tm_min)
 {
  case 59: 
  case  0: 
  case  1: 
  case  2: 
  case  3: UUR;  break; 
 }
}
//--------------------------------------------                                                //
//  CLOCK English clock display
//--------------------------------------------
void English(void)
{
 IT;                                                                                          // IT is always on
 if (timeinfo.tm_hour == 00 && timeinfo.tm_min == 0 && random(2)==0) { ISUK; MID; NIGHT; return; } 
 switch (timeinfo.tm_min)
 {
  case  0: ISUK;  EXACTUK; break;
  case  1: ISUK;  break;
  case  2: 
  case  3: WASUK; break;
  case  4: 
  case  5: 
  case  6: ISUK;  MFIVE; PAST; break;
  case  7: 
  case  8: WASUK; MFIVE; PAST; break;
  case  9: 
  case 10: 
  case 11: ISUK;  MTEN; PAST; break;
  case 12: 
  case 13: WASUK; MTEN; PAST; break;
  case 14: 
  case 15: 
  case 16: ISUK;  QUARTER; PAST; break;
  case 17: 
  case 18: WASUK; QUARTER; PAST; break;
  case 19: 
  case 20: 
  case 21: ISUK;  TWENTY; PAST; break;
  case 22: 
  case 23: WASUK; TWENTY; PAST; break;
  case 24: 
  case 25: 
  case 26: ISUK;  TWENTY; MFIVE; PAST; break;
  case 27: 
  case 28: WASUK; TWENTY; MFIVE; PAST; break;
  case 29: ISUK;  HALFUK; PAST; break;
  case 30: ISUK;  EXACTUK; HALFUK; PAST; break;
  case 31: ISUK;  HALFUK; PAST; break;
  case 32: 
  case 33: WASUK; HALFUK; PAST; break;
  case 34: 
  case 35: 
  case 36: ISUK;  TWENTY; MFIVE; TO; break;
  case 37: 
  case 38: WASUK; TWENTY; MFIVE; TO; break;
  case 39: 
  case 40: 
  case 41: ISUK;  TWENTY; TO; break;
  case 42: 
  case 43: WASUK; TWENTY; TO break;
  case 44: 
  case 45: 
  case 46: ISUK;  QUARTER; TO; break;
  case 47: 
  case 48: WASUK; QUARTER; TO; break;
  case 49: 
  case 50: 
  case 51: ISUK;  MTEN; TO;  break;
  case 52: 
  case 53: WASUK; MTEN; TO;  break;
  case 54: 
  case 55: 
  case 56: ISUK;  MFIVE; TO; break;
  case 57: 
  case 58: WASUK; MFIVE; TO; break;
  case 59: ISUK;  break;
}
//if (timeinfo.tm_hour >=0 && hour <12) digitalWrite(AMPMpin,0); else digitalWrite(AMPMpin,1);

 sayhour = timeinfo.tm_hour;
 if (timeinfo.tm_min > 33 ) sayhour = timeinfo.tm_hour+1;
 if (sayhour == 24) sayhour = 0;

switch (sayhour)
 {
  case 13:  
  case 1:  ONE; break;
  case 14:
  case 2:  TWO; break;
  case 15:
  case 3:  THREE; break;
  case 16:
  case 4:  FOUR; break;
  case 17:
  case 5:  FIVE; break;
  case 18:
  case 6:  SIXUK; break;
  case 19:
  case 7:  SEVEN; break;
  case 20:
  case 8:  EIGHT; break;
  case 21:
  case 9:  NINE; break;
  case 22:
  case 10: TEN; break;
  case 23:
  case 11: ELEVEN; break;
  case 0:
  case 12: TWELVE; break;
 } 
 switch (timeinfo.tm_min)
 {
  case 59: 
  case  0: 
  case  1: 
  case  2: 
  case  3: OCLOCK;  break; 
 }
}
//--------------------------------------------                                                //
//  CLOCK German clock display
//--------------------------------------------
void German(void)
{
  ES;                                                                                         // ES is always on
 if (timeinfo.tm_hour == 00 && timeinfo.tm_min == 0 && random(2)==0) {IST; MITTER; NACHTDE; return; } 
 switch (timeinfo.tm_min)
 {
  case  0: IST;  GENAU; break;
  case  1: IST; KURZ; NACH; break;
  case  2: 
  case  3: WAR; break;
  case  4: 
  case  5: 
  case  6: IST; MFUNF; NACH; break;
  case  7: 
  case  8: WAR; MFUNF; NACH; break;
  case  9: 
  case 10: 
  case 11: IST; MZEHN; NACH; break;
  case 12: 
  case 13: WAR; MZEHN; NACH; break;
  case 14: 
  case 15: 
  case 16: IST; VIERTEL; NACH; break;
  case 17: 
  case 18: WAR; VIERTEL; NACH; break;
  case 19: 
  case 20: 
  case 21: IST; MZEHN; VOR; HALB; break;
  case 22: 
  case 23: WAR; MZEHN; VOR; HALB; break;
  case 24: 
  case 25: 
  case 26: IST; MFUNF; VOR; HALB; break;
  case 27: 
  case 28: WAR; MFUNF; VOR; HALB; break;
  case 29: IST; KURZ;  VOR; HALB; break;
  case 30: IST; GENAU; HALB; break;
  case 31: IST; KURZ;  NACH; HALB; break;
  case 32: 
  case 33: WAR; HALB; break;
  case 34: 
  case 35: 
  case 36: IST; MFUNF; NACH; HALB; break;
  case 37: 
  case 38: WAR; MFUNF; NACH; HALB; break;
  case 39: 
  case 40: 
  case 41: IST; MZEHN; NACH; HALB; break;
  case 42: 
  case 43: WAR; MZEHN; NACH; HALB; break;
  case 44: 
  case 45: 
  case 46: IST; VIERTEL; VOR; break;
  case 47: 
  case 48: WAR; VIERTEL; VOR; break;
  case 49: 
  case 50: 
  case 51: IST; MZEHN; VOR;  break;
  case 52: 
  case 53: WAR; MZEHN; VOR;  break;
  case 54: 
  case 55: 
  case 56: IST; MFUNF; VOR; break;
  case 57: 
  case 58: WAR; MFUNF; VOR; break;
  case 59: IST;  break;
}
//if (timeinfo.tm_hour >=0 && hour <12) digitalWrite(AMPMpin,0); else digitalWrite(AMPMpin,1);

 sayhour = timeinfo.tm_hour;
 if (timeinfo.tm_min > 18 ) sayhour = timeinfo.tm_hour+1;
 if (sayhour == 24) sayhour = 0;

switch (sayhour)
 {
  case 13:  
  case 1: EINS; break;
  case 14:
  case 2: ZWEI; break;
  case 15:
  case 3: DREI; break;
  case 16:
  case 4: VIERDE; break;
  case 17:
  case 5: FUNF; break;
  case 18:
  case 6: SECHS; break;
  case 19:
  case 7: SIEBEN; break;
  case 20:
  case 8: ACHTDE; break;
  case 21:
  case 9: NEUN; break;
  case 22:
  case 10: ZEHN; break;
  case 23:
  case 11: ELFDE; break;
  case 0:
  case 12: ZWOLF; break;
 } 
 switch (timeinfo.tm_min)
 {
  case 59: 
  case  0: 
  case  1: 
  case  2: 
  case  3: UHR;  break; 
 }
}
//--------------------------------------------                                                //
//  CLOCK French clock display
//--------------------------------------------
void French(void)
{
 IL;                                                                                          // IL is always on
 switch (timeinfo.tm_min)
 {
  case  0: EST;   EXACT; DITLEHEURE; break;
  case  1: EST;   DITLEHEURE; break;
  case  2: 
  case  3: ETAIT; DITLEHEURE; break;
  case  4: 
  case  5: 
  case  6: EST;   DITLEHEURE; MCINQ; break;
  case  7: 
  case  8: ETAIT; DITLEHEURE; MCINQ; break;
  case  9: 
  case 10: 
  case 11: EST;   DITLEHEURE; MDIX;  break;
  case 12: 
  case 13: ETAIT; DITLEHEURE; MDIX;  break;
  case 14: 
  case 15: 
  case 16: EST;   DITLEHEURE; ET; QUART; break;
  case 17: 
  case 18: ETAIT; DITLEHEURE; ET; QUART; break;
  case 19: 
  case 20: 
  case 21: EST;   DITLEHEURE; VINGT; break;
  case 22: 
  case 23: ETAIT; DITLEHEURE; VINGT; break;
  case 24: 
  case 25: 
  case 26: EST;   DITLEHEURE; VINGT; MCINQ; break;
  case 27: 
  case 28: ETAIT; DITLEHEURE; VINGT; MCINQ; break;
  case 29: EST;   DITLEHEURE; ET; DEMI; break;
  case 30: EST;   EXACT; DITLEHEURE;  ET; DEMI; break;
  case 31: EST;   DITLEHEURE; ET; DEMI; break;
  case 32: 
  case 33: ETAIT; DITLEHEURE; ET; DEMI; break;
  case 34: 
  case 35: 
  case 36: EST;   DITLEHEURE; MOINS; VINGT; MCINQ; break;
  case 37: 
  case 38: ETAIT; DITLEHEURE; MOINS; VINGT; MCINQ; break;
  case 39: 
  case 40: 
  case 41: EST;   DITLEHEURE; MOINS; VINGT;  break;
  case 42: 
  case 43: ETAIT; DITLEHEURE; MOINS; VINGT;  break;
  case 44: 
  case 45: 
  case 46: EST;   DITLEHEURE; MOINS; LE; QUART; break;
  case 47: 
  case 48: ETAIT; DITLEHEURE; MOINS; LE; QUART; break;
  case 49: 
  case 50: 
  case 51: EST;   DITLEHEURE; MOINS; MDIX;   break;
  case 52: 
  case 53: ETAIT; DITLEHEURE; MOINS; MDIX;   break;
  case 54: 
  case 55: 
  case 56: EST;   DITLEHEURE; MOINS; MCINQ;  break;
  case 57: 
  case 58: ETAIT; DITLEHEURE; MOINS; MCINQ;  break;
  case 59: EST;   DITLEHEURE;  break;
 }
}

void DitLeHeure(void)
{
 byte sayhour = timeinfo.tm_hour;
 if (timeinfo.tm_min > 33 ) sayhour = timeinfo.tm_hour+1;
 if (sayhour == 24) sayhour = 0;

switch (sayhour)
 {
  case 13:  
  case 1:  UNE;    HEURE;  break;
  case 14:
  case 2:  DEUX;   HEURES;  break;
  case 15:
  case 3:  TROIS;  HEURES;  break;
  case 16:
  case 4:  QUATRE; HEURES; break;
  case 17:
  case 5:  CINQ;   HEURES;   break;
  case 18:
  case 6:  SIX;    HEURES;   break;
  case 19:
  case 7:  SEPT;   HEURES;  break;
  case 20:
  case 8:  HUIT;   HEURES; break;
  case 21:
  case 9:  NEUF;   HEURES; break;
  case 22:
  case 10: DIX;    HEURES; break;
  case 23:
  case 11: ONZE;   HEURES; break;
  case 0:  MINUIT; break;
  case 12: MIDI;   break;
 } 
}
//--------------------------------------------                                                //
// BLE 
// SendMessage by BLE Slow in packets of 20 chars
// or fast in one long string.
// Fast can be used in IOS app BLESerial Pro
//------------------------------
void SendMessageBLE(std::string Message)
{
 if(deviceConnected) 
   {
    if (Mem.UseBLELongString)                                                                 // If Fast transmission is possible
     {
      pTxCharacteristic->setValue(Message); 
      pTxCharacteristic->notify();
      delay(10);                                                                              // Bluetooth stack will go into congestion, if too many packets are sent
     } 
   else                                                                                       // Packets of max 20 bytes
     {   
      int parts = (Message.length()/20) + 1;
      for(int n=0;n<parts;n++)
        {   
         pTxCharacteristic->setValue(Message.substr(n*20, 20)); 
         pTxCharacteristic->notify();
         delay(40);                                                                           // Bluetooth stack will go into congestion, if too many packets are sent
        }
     }
   } 
}

//-----------------------------
// BLE Start BLE Classes
//------------------------------
class MyServerCallbacks: public BLEServerCallbacks 
{
 void onConnect(BLEServer* pServer) {deviceConnected = true; };
 void onDisconnect(BLEServer* pServer) {deviceConnected = false;}
};

class MyCallbacks: public BLECharacteristicCallbacks 
{
 void onWrite(BLECharacteristic *pCharacteristic) 
  {
   std::string rxValue = pCharacteristic->getValue();
   ReceivedMessageBLE = rxValue + "\n";
//   if (rxValue.length() > 0) {for (int i = 0; i < rxValue.length(); i++) printf("%c",rxValue[i]); }
//   printf("\n");
  }  
};
//--------------------------------------------                                                //
// BLE Start BLE Service
//------------------------------
void StartBLEService(void)
{
 BLEDevice::init(Mem.BLEbroadcastName);                                                       // Create the BLE Device
 pServer = BLEDevice::createServer();                                                         // Create the BLE Server
 pServer->setCallbacks(new MyServerCallbacks());
 BLEService *pService = pServer->createService(SERVICE_UUID);                                 // Create the BLE Service
 pTxCharacteristic                     =                                                      // Create a BLE Characteristic 
     pService->createCharacteristic(CHARACTERISTIC_UUID_TX, NIMBLE_PROPERTY::NOTIFY);                 
 BLECharacteristic * pRxCharacteristic = 
     pService->createCharacteristic(CHARACTERISTIC_UUID_RX, NIMBLE_PROPERTY::WRITE);
 pRxCharacteristic->setCallbacks(new MyCallbacks());
 pService->start(); 
 BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
 pAdvertising->addServiceUUID(SERVICE_UUID); 
 pServer->start();                                                                            // Start the server  Nodig??
 pServer->getAdvertising()->start();                                                          // Start advertising
 TekstSprint("BLE Waiting a client connection to notify ...\n"); 
}
//                                                                                            //
//-----------------------------
// BLE  CheckBLE
//------------------------------
void CheckBLE(void)
{
 if(!deviceConnected && oldDeviceConnected)                                                   // Disconnecting
   {
//    delay(30);                                                                               // Give the bluetooth stack the chance to get things ready
    TekstSprint("Start advertising\n");
    pServer->startAdvertising();                                                              // Restart advertising
    oldDeviceConnected = deviceConnected;
   }
 if(deviceConnected && !oldDeviceConnected)                                                   // Connecting
   { 
    oldDeviceConnected = deviceConnected;
    SWversion();
   }
 if(ReceivedMessageBLE.length()>0)
   {
    SendMessageBLE(ReceivedMessageBLE);
    String BLEtext = ReceivedMessageBLE.c_str();
    ReceivedMessageBLE = "";
    ReworkInputString(BLEtext); 
   }
}

//--------------------------------------------                                                //
//  DISPLAY Init display
//--------------------------------------------
void InitDisplay()
{

InitScreenST7789();
}

//--------------------------------------------                                                //
//  DISPLAY Restart display after been turned off
//--------------------------------------------
void RestartDisplay(void)
{
  if (Mem.DisplayChoice == ANALOOG) AnalogClockSetupST7789();
}
//--------------------------------------------                                                //
//  DISPLAY
//  Wake up display
//--------------------------------------------
 void DisplayWakeUp(void)
 {
  ttgo->displayWakeup();
 }

//--------------------------------------------                                                //
//  DISPLAY
//  Power Off  display
//--------------------------------------------
 void DisplayPowerOff(void)
 {
  ttgo->tft->fillScreen (TFT_BLACK);
  ttgo->displaySleep();
 } 
//--------------------------------------------                                                //
//  DISPLAY
//  Print characters on the display with black background
//--------------------------------------------
void ClearScreen()
{
 ClearScreenST7789();
}
//--------------------------------------------
//  DISPLAY Set intensity backlight (0 -255)
//--------------------------------------------
void SetBackLight(byte intensity)
{ 
// analogWrite(TFT_LED, intensity); 
 ledcWrite(0, intensity);   //ledChannel, dutyCycle);
}
//--------------------------------------------                                                //
//  DISPLAY
//  Print characters on the display with black background
//--------------------------------------------
void ShowChars(void)
{
 ShowCharsST7789();
}
//--------------------------------------------                                                //
//  DISPLAY
//  Do screen update called every second
//--------------------------------------------
void ScreenSecondProcess()
{
   SetSecondColour();
   if (Mem.DisplayChoice == DIGITAL)  PrintTimeInScreen();                                    // Show the digital time in the screen
   if (Mem.DisplayChoice == ANALOOG)  DrawAnalogeClockHands();                                // Show the analogue time in the screen  
}
//--------------------------------------------                                                //
//  DISPLAY
//  Setup analogue clock display
//--------------------------------------------
void AnalogClockSetup()
{
 AnalogClockSetupST7789();
}

//--------------------------------------------                                                //
/* Avaiable functions for the ILI9341 display
// -------------------------------------------
 void fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
 void fillScreen(uint16_t color);
 void setTextSize(uint8_t s);
 void setTextSize(uint8_t sx, uint8_t sy);
 void setFont(const GFXfont *f = NULL);
 void drawChar(int16_t x, int16_t y, unsigned char c, uint16_t color, uint16_t bg, uint8_t size);
 void drawChar(int16_t x, int16_t y, unsigned char c, uint16_t color, uint16_t bg, uint8_t size_x, uint8_t size_y);
 virtual void setRotation(uint8_t r);
 virtual void invertDisplay(bool i);

 void setCursor(int16_t x, int16_t y)
 void setTextColor(uint16_t c, uint16_t bg)
 void setTextWrap(bool w) { wrap = w; }
 int16_t width(void) const { return _width; };
 int16_t height(void) const { return _height; }
 uint8_t getRotation(void) const { return rotation; }
 int16_t getCursorX(void) const { return cursor_x; }
 int16_t getCursorY(void) const { return cursor_y; };
*/

//--------------------------------------------                                                //
// Common common print routines to display
//--------------------------------------------
void Displayprint(const char *tekst)  { ttgo->tft->print(tekst);   }                          // Print to an attached display
void Displayprintln(const char *tekst){ ttgo->tft->println(tekst); }
void DisplayPosPrintln(uint8_t x, uint8_t y, const char *tekst)
     {
     ttgo->tft->setCursor(x, y);
     ttgo->tft->println(tekst); 
     delay (100);
     }
//--------------------------------------------
//  SCREEN ST7789 Init screen ST7789
//--------------------------------------------
void InitScreenST7789(void)
{
 byte ch = Mem.LanguageChoice;
 byte dp = Mem.DisplayChoice;
 uint32_t row = 15; uint32_t col = 3;  uint32_t n = 20;
 ttgo->tft->setTextColor( ConvertRGB32toRGB16(C_ORANGE),C_BLACK);                             // Convert colors to RGB16, except C_BLACK (=0x000000)
 ttgo->tft->setTextSize(1);
 sprintf(sptext,"Display off : %02dh - %02dh",Mem.TurnOffLEDsAtHH, Mem.TurnOnLEDsAtHH);  DisplayPosPrintln(col, row, sptext); row += n;
 sprintf(sptext,"Choice: %s",dp==0?"Yellow":dp==1?"Hourly":dp==2?"White":
      dp==3?"All Own":dp==4?"Own":dp==5?"Wheel":dp==6?"Digital":dp==7?"Analog":"NOP");   DisplayPosPrintln(col, row, sptext); row += n;
 sprintf(sptext,"Slope: %d Min: %d Max: %d ",
      Mem.LightReducer, Mem.LowerBrightness,Mem.UpperBrightness);                        DisplayPosPrintln(col, row, sptext); row += n;
 sprintf(sptext,"BLE name: %s", Mem.BLEbroadcastName);                                   DisplayPosPrintln(col, row, sptext); row += n;
 sprintf(sptext,"%s %s %s %s", Mem.WIFIOnOff?"WIFI=On":"WIFI=Off", 
                               Mem.NTPOnOff? "NTP=On":"NTP=Off",
                               Mem.BLEOnOff? "BLE=On":"BLE=Off",
                               Mem.UseBLELongString? "FastBLE=On":"FastBLE=Off" );       DisplayPosPrintln(col, row, sptext); row += n;
 sprintf(sptext,"Language choice: %s",
         ch==0?"NL":ch==1?"UK":ch==2?"DE":ch==3?"FR":ch==4?"Rotate language":"NOP");     DisplayPosPrintln(col, row, sptext); row += n;
 sprintf(sptext,"%s",FILENAAM);                                                          DisplayPosPrintln(col, row, sptext);   //VERSION); 

}
//--------------------------------------------                                                //
//  SCREEN ST7789 Convert 32 bit RGBW to 16 bit RGB
//--------------------------------------------

unsigned short ConvertRGB32toRGB16(uint32_t sourceColor)
{
  uint32_t red   = (sourceColor & 0x00FF0000) >> 16;
  uint32_t green = (sourceColor & 0x0000FF00) >> 8;
  uint32_t blue  =  sourceColor & 0x000000FF;
  return (red >> 3 << 11) + (green >> 2 << 5) + (blue >> 3);
}

//--------------------------------------------
//  SCREEN ST7789 Make the the screen black
//--------------------------------------------
void ClearScreenST7789(void) 
{ 
  ttgo->tft->fillScreen(Mem.BackGround);
}

//--------------------------------------------                                                //
//  SCREEN ST7789
//  Print characters on the display with black background
//--------------------------------------------
void ShowCharsST7789(void)
{
 int LEDnr = 0;
 for(int y = 0; y < MATRIX_HEIGHT; y++)
    for(int x = 0; x < MATRIX_WIDTH; x++, LEDnr++)
       {
        ttgo->tft->drawChar(20 * x,  20 * y, Strippos[LEDnr].Character, 
                      ConvertRGB32toRGB16(Strippos[LEDnr].RGBColor), 
                            ConvertRGB32toRGB16(Mem.BackGround), 2); 
       }     
}

//--------------------------------------------                                                //
//  SCREEN ST7789 Print the digital time in screen
//--------------------------------------------
void PrintTimeInScreen(void) 
{
 ttgo->tft->setTextColor(ConvertRGB32toRGB16(LetterColor), C_BLACK);                                 // Convert colors to RGB16, except C_BLACK (=0x000000)
 GetTijd(0); 
// if(Mem.WIFIOnOff && !IP_Printed)                                                             // If WIFI is on print the IP-address
 {
  ttgo->tft->setTextSize(1);
  ttgo->tft->setCursor(20, 20); 
  PrintIPaddressInScreen(); 
  IP_Printed = true; 
  }
 sprintf(sptext,"%02d:%02d:%02d",timeinfo.tm_hour,timeinfo.tm_min,timeinfo.tm_sec);
 ttgo->tft->setTextSize(4); 
 ttgo->tft->setCursor(20, 90);
 Displayprint(sptext); 
 sprintf(sptext,"Battery %2d%%",ttgo->power->getBattPercentage());
 ttgo->tft->setCursor(20, 150);
 ttgo->tft->setTextSize(2);
 Displayprint(sptext);
 
 if (Date_Printed)  return;
 sprintf(sptext,"%02d-%02d-%04d",timeinfo.tm_mday,timeinfo.tm_mon+1,timeinfo.tm_year+1900);
 ttgo->tft->setCursor(20, 200);
 ttgo->tft->setTextSize(2);
 Displayprint(sptext);
 Date_Printed = true;
}
//--------------------------------------------                                                //
//  SCREEN ST7789
// Print Web server IP address in screen
//--------------------------------------------
void PrintIPaddressInScreen()
{
 sprintf(sptext, "%d.%d.%d.%d", WiFi.localIP()[0], WiFi.localIP()[1], WiFi.localIP()[2], WiFi.localIP()[3] );
 Displayprintln(sptext);
}

//--------------------------------------------
// ANALOG CLOCK setup
//--------------------------------------------

void  AnalogClockSetupST7789(void) 
{
 float sx, sy; 
 uint8_t smx = ScreenMiddleX;
 uint8_t smy = ScreenMiddleY;
 uint16_t x00=0, x11=0, y00=0, y11=0;
// ttgo->tft->fillScreen(ConvertRGB32toRGB16(C_GRAY)); 
 ttgo->tft->setTextColor(ConvertRGB32toRGB16(C_WHITE), ConvertRGB32toRGB16(C_GRAY));                // Adding a background colour erases previous text automatically
 ttgo->tft->fillCircle(smx, smy, 118, ConvertRGB32toRGB16(C_GREEN));                                // Draw clock face
 ttgo->tft->fillCircle(smx, smy, 110, ConvertRGB32toRGB16(C_BLACK));
 for(int i = 0; i<360; i+= 30)   // Draw 12 lines
   {
    sx = cos((i-90)*0.0174532925);                                                           // 0.0174532925 rad/s in 1 deg/s
    sy = sin((i-90)*0.0174532925);                                                           // 1 rad = 57.29577951 degrees
    x00 = sx*114+smx;
    y00 = sy*114+smy;
    x11 = sx*100+smx;
    y11 = sy*100+smy;
    ttgo->tft->drawLine(x00, y00, x11, y11, ConvertRGB32toRGB16(C_GREEN));
   }
 for(int i = 0; i<360; i+= 6)                                                                // Draw 60 dots
   {
    sx = cos((i-90)*0.0174532925);
    sy = sin((i-90)*0.0174532925);
    x00 = sx*102+smx;
    y00 = sy*102+smy;
    ttgo->tft->drawPixel(x00, y00, ConvertRGB32toRGB16(C_WHITE));                                   // Draw minute markers
    if(i==0  || i==180) ttgo->tft->fillCircle(x00, y00, 2, ConvertRGB32toRGB16(C_WHITE));           // Draw main quadrant dots
    if(i==90 || i==270) ttgo->tft->fillCircle(x00, y00, 2, ConvertRGB32toRGB16(C_WHITE));
   }
//  ttgo->tft->fillCircle(160, 121, 3, ConvertRGB32toRGB16(C_WHITE));

}
//                                                                                            //
//--------------------------------------------
// ANALOG CLOCK Draw hands
//--------------------------------------------
void DrawAnalogeClockHands(void)
{
 uint8_t smx = ScreenMiddleX;
 uint8_t smy = ScreenMiddleY;
 static int ss = 0,mm = 0,hh = 0;                                                                                             // Pre-compute hand degrees, x & y coords for a fast screen update
 if (mm !=timeinfo.tm_min || ss == 59) 
   {
    DrawMinuteHand(mm, 0, smx, smy, C_BLACK);                                                 // Erase old minute and hour hand
    DrawHourHand  (hh, mm, smx, smy, C_BLACK); 
    }
 DrawSecondHand(ss,   smx, smy, C_BLACK);                                                     // Redraw new hand positions, hour and minute hands not erased here to avoid flicker
 ss = (int) timeinfo.tm_sec;                                                                  // Retrieve the actual time
 mm = (int) timeinfo.tm_min;
 hh = (int) timeinfo.tm_hour;

 // DrawSecondHand(ss,     smx, smy, 0xF800);   // (C_RED));
 DrawMinuteHand(mm, 0,  smx, smy, 0xFFFF);    // (C_WHITE)
 DrawHourHand(  hh, mm, smx, smy, 0xFFFF); 
 DrawSecondHand(ss,     smx, smy, 0xF800);

 sprintf(sptext,"%02d:%02d:%02d",timeinfo.tm_hour,timeinfo.tm_min,timeinfo.tm_sec);          // Print the digital time in the analog clock
 ttgo->tft->setTextColor(ConvertRGB32toRGB16(C_YELLOW), C_BLACK);
 ttgo->tft->setTextSize(2); 
 ttgo->tft->setCursor(smx-50, smy/2);
 Displayprint(sptext); 
}
//--------------------------------------------
// ANALOG CLOCK Draw second hands
//--------------------------------------------
void DrawSecondHand(int ss , uint8_t smx, uint8_t smy, uint32_t Kleur)
{  
 if(ss <0 ) ss = 59;                                    
 uint16_t osx = cos((ss * 6 - 90) * 0.0174532925) * 90 + smx;                                 // 0-59 ss -> 0-354 degrees
 uint16_t osy = sin((ss * 6 - 90) * 0.0174532925) * 90 + smy; 
 ttgo->tft->drawLine(osx,   osy,   smx, smy, Kleur);    
 ttgo->tft->fillCircle(smx, smy, 3, 0xF800);
}
//--------------------------------------------
// ANALOG CLOCK Draw minute hands
//--------------------------------------------
void DrawMinuteHand(int mm, int ss, uint8_t smx, uint8_t smy, uint32_t Kleur)
{
 if(mm < 0) mm = 59;
 if(ss <0 ) ss = 59;  
 uint8_t ycorr = 0, xcorr = 0;
 float mdeg = mm * 6; //+ ss * 6 * 0.01666667;                                                // 0-59 -> 0-360 - includes not seconds
 uint16_t omx = cos((mdeg-90) * 0.0174532925) * 84 + smx;    
 uint16_t omy = sin((mdeg-90) * 0.0174532925) * 84 + smy;
                                                                                              // sprintf(sptext,"MINUUT: mm:%d ss:%d ohx:%d ohy:%d ",mm,ss,omx,omy ); Tekstprintln(sptext);
 if(mm>7  && mm <22) {xcorr = 0; ycorr = 1;} 
 if(mm>21 && mm <38) {xcorr = 1; ycorr = 0;}    
 if(mm>37 && mm <52) {xcorr = 0; ycorr = 1;} 
 if(mm>51 || mm <8)  {xcorr = 1; ycorr = 0;}  
 for (int n=-4; n<5; n++)
   ttgo->tft->drawLine(omx + 0 * xcorr, omy + 0 * ycorr, smx + n * xcorr, smy + n * ycorr, Kleur); 
}
//--------------------------------------------
// ANALOG CLOCK Draw hour hands
//--------------------------------------------
void DrawHourHand(uint8_t hh, int mm, uint8_t smx, uint8_t smy, uint32_t Kleur)
{
 uint8_t ycorr = 0, xcorr = 0;
 float hdeg = hh%12 * 30 + mm * 0.5;                                                          // 0-11 -> 0-360 - includes minutes
 uint16_t  ohx = cos((hdeg-90) * 0.0174532925) * 62 + smx;    
 uint16_t  ohy = sin((hdeg-90) * 0.0174532925) * 62 + smy;
// sprintf(sptext,"   UUR: hh:%d mm:%d ohx:%d ohy:%d ",hh,mm,ohx,ohy ); Tekstprintln(sptext);
 
 if(hh>1  && hh <5)  {xcorr = 0; ycorr = 1;} 
 if(hh>4  && hh <8)  {xcorr = 1; ycorr = 0;}    
 if(hh>7  && hh <11) {xcorr = 0; ycorr = 1;} 
 if(hh>10 || hh <2)  {xcorr = 1; ycorr = 0;}       
 for (int n=-5; n<6; n++)
  ttgo->tft->drawLine(ohx + 0 * xcorr, ohy + 0 * ycorr, smx + n * xcorr, smy + n * ycorr, Kleur);   
}

//--------------------------------------------                                                //
// WATCH Setup Watch
//--------------------------------------------
void SetupWatch(void)
{
 ttgo=TTGOClass::getWatch();                                                                  // Get watch instance, see also the documentation for the TTGO Watch library
 ttgo->begin();                                                                               // Initialize the hardware
 ttgo->openBL();                                                                              // Turn on the backlight                    
 ttgo->power->adc1Enable(AXP202_VBUS_VOL_ADC1 | AXP202_VBUS_CUR_ADC1 | AXP202_BATT_CUR_ADC1 | AXP202_BATT_VOL_ADC1, true);  // Settings for using the power managemnt of the watch
 Acfg Accelcfg;                                                                               // Accelerator parameter structure
 Accelcfg.odr = BMA4_OUTPUT_DATA_RATE_100HZ;  
 Accelcfg.range = BMA4_ACCEL_RANGE_2G;  
 Accelcfg.bandwidth = BMA4_ACCEL_NORMAL_AVG4;
 Accelcfg.perf_mode = BMA4_CONTINUOUS_MODE;  
 ttgo->bma->accelConfig(Accelcfg);                                                            // Configure the BMA423 accelerometer
 ttgo->bma->enableAccel();                                                                    // Enable BMA423 accelerometer   // Warning : Need to use feature, you must first enable the accelerometer  
 ttgo->bma->enableFeature(BMA423_STEP_CNTR, false);                                           // Disable BMA423 isStepCounter feature
 ttgo->bma->enableFeature(BMA423_TILT, true);                                                 // Enable BMA423 isTilt feature
 ttgo->bma->enableFeature(BMA423_WAKEUP, true);                                               // Enable BMA423 isDoubleClick feature
 ttgo->bma->resetStepCounter();                                                               // Reset steps
 // sensor->enableStepCountInterrupt();                                                       // Turn off feature interrupt
 ttgo->bma->enableTiltInterrupt();
 ttgo->bma->enableWakeupInterrupt();                                                          // It corresponds to isDoubleClick interrupt
}

//--------------------------------------------                                                //
// WATCH Power and backlight off
//--------------------------------------------
bool PowerBacklightOff(void)
{
 DisplayIsOn = false; 
 ttgo->tft->fillScreen (TFT_BLACK);
 ttgo->displaySleep();
 ttgo->closeBL();  
 SecondsLeap = 0;
 return DisplayIsOn;
}

//--------------------------------------------                                                //
// WATCH Power and backlight On
//--------------------------------------------
bool PowerBacklightOn(void)
{
 DisplayIsOn = true; 
 ttgo->displayWakeup();      // ...then wake up the display
 ttgo->openBL();             // ...and turn on the backlight
 if (Mem.DisplayChoice == ANALOOG)   AnalogClockSetup(); 
 else Displaytime();                                                             // Turn the display on 
 return DisplayIsOn;
}

//--------------------------------------------                                                //
// WATCH Check watch tilted for looking to it
//--------------------------------------------
bool CheckForWatchLookingTo(void)
{
 Accel acc;
 ttgo->bma->getAccel(acc);                                                                    // Get x y z coordinates
 if ( (acc.x < -600) || (acc.x >  150) ) return false;                                        // Check if they are between bounderies
 if ( (acc.y <  -50) || (acc.y >  300) ) return false;
 if ( (acc.z <-1100) || (acc.z > -900) ) return false;
//sprintf(sptext,"aac.x: %d aac.y %d aac.z %d",acc.x,acc.y,acc.z);     Tekstprintln(sptext);  
 return true; 
} 

//--------------------------------------------                                                //
// WATCH Check and turn on or off Display
// If the watch is kept horizontal for >250 msec
// Turns the display off after 6 seconds
//--------------------------------------------
void CheckDisplayOnOrOff(void)
{
 static uint32_t DisplayReactionTimer = 0; 
 if (Mem.DisplayOnForSecs == 0) return;                                                      // Keeps display state unchanged
 if (millis() - DisplayReactionTimer < 250) return;                                          // Keep the watch for 250 ms in the right horizintal position
 DisplayReactionTimer = millis();
 if(CheckForWatchLookingTo() && !DisplayIsOn)
    { PowerBacklightOn(); return;}
 if (!CheckForWatchLookingTo() && SecondsLeap > Mem.DisplayOnForSecs && DisplayIsOn)       
    { PowerBacklightOff();}
}

//--------------------------------------------                                                //
// WATCH Setup Watch
// Shut down the watch completely
//--------------------------------------------
void PowerOffWatch(void)
{
  ttgo->tft->fillScreen (TFT_BLACK);
  ttgo->displaySleep();
  ttgo->closeBL();
  ttgo->powerOff();
  esp_sleep_enable_ext1_wakeup(GPIO_SEL_39, ESP_EXT1_WAKEUP_ANY_HIGH);                         // Use ext1 for external wakeup
  esp_deep_sleep_start();  
}
//--------------------------------------------                                                //
// WATCH Print battery status
//--------------------------------------------
void PrintBattery(void)
{
 Serial.print(ttgo->power->getBattPercentage());
 Serial.print(ttgo->power->isVBUSPlug());
}
//--------------------------------------------                                                //
// WATCH Wait for touch of the display
// Waiting for touching the display and calculating if display was touched at one position or calculating the direction of wiping the display
//
//--------------------------------------------
uint32_t DisplayTouchOrSwipe(void)
{
  int16_t TouchX = 0, TouchY = 0;
  int16_t TouchX1 = 0, TouchY1 = 0, TouchX2 = 0, TouchY2 = 0;
  uint32_t Timer = millis();                                                                   // Used as a timer
  if(ttgo->getTouch(TouchX, TouchY))  {TouchX1=TouchX; TouchY1=TouchY;  } 
  else return 0;                                                                               //  sprintf(sptext,"TouchX1 %d TouchY1 %d",TouchX1,TouchY1);       Tekstprintln(sptext); 11
  
  while (ttgo->getTouch(TouchX, TouchY) )                                                      // Display is no more touched compare the saved coordinates to calculate the direction of wiping
    TouchX2 = TouchX;
    TouchY2 = TouchY;
//  sprintf(sptext,"TouchX2 %d TouchY2 %d",TouchX2,TouchY2);       Tekstprintln(sptext); 
  if (Timer-millis() < 50)                                         {return 0;}                 // To short touch or swipe
  if (abs(TouchX2 - TouchX1)< 20 && abs(TouchY2 - TouchY1) < 20 )  {return 1;}                 // Display is touched
  if     (TouchX2 < TouchX1 - 50)                                  {return 2;}                 // Swiping to the left side
  if     (TouchX2 > TouchX1 + 50)                                  {return 3;}                 // Swiping to the right side
  if     (TouchY2 < TouchY1 - 50)                                  {return 4;}                 // Swiping up to the upper side
  if     (TouchY2 > TouchY1 + 50)                                  {return 5;}                 // Swiping down to the bottom side
return 0;
}
//--------------------------------------------                                                //
// WATCH Set internal rtc time
// Syncs &timeinfo to internal rtc  NOT USED
//--------------------------------------------
void SetInternalRTCtime(){  ttgo->rtc->syncToRtc();}
//--------------------------------------------                                                //
// WATCH Get internal rtc time  NOT USED
//--------------------------------------------
void GetInternalRTCtime()
{
  getLocalTime(&timeinfo);
  Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
}

//--------------------------------------------                                                //
// WIFI Events Reconnect when Wifi is lost
// An annoyance with the standard WIFI connection examples is that the connections 
// often fails after uploading of starting the MCU
// After an ESP32.reset a connection succeeds
// By using WIFIevents the WIFI manager tries several times to connect and ofter succeeds within two attempts.

// NOT USED in this version
//--------------------------------------------

//--------------------------------------------                                                //
// WIFI Events
//--------------------------------------------
void WiFiEvent(WiFiEvent_t event)
{
  sprintf(sptext,"[WiFi-event] event: %d", event); 
  Tekstprintln(sptext);
  WiFiEventInfo_t info;
    switch (event) {
        case ARDUINO_EVENT_WIFI_READY: 
            Tekstprintln("WiFi interface ready");
            break;
        case ARDUINO_EVENT_WIFI_SCAN_DONE:
            Tekstprintln("Completed scan for access points");
            break;
        case ARDUINO_EVENT_WIFI_STA_START:
            Tekstprintln("WiFi client started");
            break;
        case ARDUINO_EVENT_WIFI_STA_STOP:
            Tekstprintln("WiFi clients stopped");
            break;
        case ARDUINO_EVENT_WIFI_STA_CONNECTED:
            Tekstprintln("Connected to access point");
            break;
       case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
            Mem.ReconnectWIFI++;
            sprintf(sptext, "Disconnected from station, attempting reconnection for the %d time", Mem.ReconnectWIFI);
            Tekstprintln(sptext);
  //          sprintf(sptext,"WiFi lost connection. Reason: %d",info.wifi_sta_disconnected.reason); 
 //           Tekstprintln(sptext);
            WiFi.reconnect();
            break;
        case ARDUINO_EVENT_WIFI_STA_AUTHMODE_CHANGE:
            Tekstprintln("Authentication mode of access point has changed");
            break;
        case ARDUINO_EVENT_WIFI_STA_GOT_IP:
             sprintf(sptext, "Obtained IP address: %d.%d.%d.%d", WiFi.localIP()[0], WiFi.localIP()[1], WiFi.localIP()[2], WiFi.localIP()[3] );
            Tekstprintln(sptext);
            WiFiGotIP(event,info);
            break;
        case ARDUINO_EVENT_WIFI_STA_LOST_IP:
            Tekstprintln("Lost IP address and IP address is reset to 0");
            break;
        case ARDUINO_EVENT_WPS_ER_SUCCESS:
            Tekstprintln("WiFi Protected Setup (WPS): succeeded in enrollee mode");
            break;
        case ARDUINO_EVENT_WPS_ER_FAILED:
            Tekstprintln("WiFi Protected Setup (WPS): failed in enrollee mode");
            break;
        case ARDUINO_EVENT_WPS_ER_TIMEOUT:
            Tekstprintln("WiFi Protected Setup (WPS): timeout in enrollee mode");
            break;
        case ARDUINO_EVENT_WPS_ER_PIN:
            Tekstprintln("WiFi Protected Setup (WPS): pin code in enrollee mode");
            break;
        case ARDUINO_EVENT_WIFI_AP_START:
            Tekstprintln("WiFi access point started");
            break;
        case ARDUINO_EVENT_WIFI_AP_STOP:
            Tekstprintln("WiFi access point  stopped");
            break;
        case ARDUINO_EVENT_WIFI_AP_STACONNECTED:
            Tekstprintln("Client connected");
            break;
        case ARDUINO_EVENT_WIFI_AP_STADISCONNECTED:
            sprintf(sptext,"Client disconnected.");                                            // Reason: %d",info.wifi_ap_stadisconnected.reason); 
            Tekstprintln(sptext);

//          Serial.print("WiFi lost connection. Reason: ");
//          Tekstprintln(info.wifi_sta_disconnected.reason);
            break;
        case ARDUINO_EVENT_WIFI_AP_STAIPASSIGNED:
            Tekstprintln("Assigned IP address to client");
            break;
        case ARDUINO_EVENT_WIFI_AP_PROBEREQRECVED:
            Tekstprintln("Received probe request");
            break;
        case ARDUINO_EVENT_WIFI_AP_GOT_IP6:
            Tekstprintln("AP IPv6 is preferred");
            break;
        case ARDUINO_EVENT_WIFI_STA_GOT_IP6:
            Tekstprintln("STA IPv6 is preferred");
            break;
        case ARDUINO_EVENT_ETH_GOT_IP6:
            Tekstprintln("Ethernet IPv6 is preferred");
            break;
        case ARDUINO_EVENT_ETH_START:
            Tekstprintln("Ethernet started");
            break;
        case ARDUINO_EVENT_ETH_STOP:
            Tekstprintln("Ethernet stopped");
            break;
        case ARDUINO_EVENT_ETH_CONNECTED:
            Tekstprintln("Ethernet connected");
            break;
        case ARDUINO_EVENT_ETH_DISCONNECTED:
            Tekstprintln("Ethernet disconnected");
            break;
        case ARDUINO_EVENT_ETH_GOT_IP:
            Tekstprintln("Obtained IP address");
            WiFiGotIP(event,info);
            break;
        default: break;
    }}
//--------------------------------------------                                                //
// WIFI GOT IP address 
//--------------------------------------------
void WiFiGotIP(WiFiEvent_t event, WiFiEventInfo_t info)
{
  WIFIConnected = 1;
//       Displayprintln("WIFI is On"); 
 if(Mem.WIFIOnOff) WebPage();                                                                 // Show the web page if WIFI is on
 if(Mem.NTPOnOff)
   {
    NTP.setTimeZone(Mem.Timezone);                                                            // TZ_Europe_Amsterdam); //\TZ_Etc_GMTp1); // TZ_Etc_UTC 
    NTP.begin();                                                                              // https://raw.githubusercontent.com/nayarsystems/posix_tz_db/master/zones.csv
    Tekstprintln("NTP is On"); 
    Displayprintln("NTP is On");                                                              // Print the text on the display
   } 
}

//--------------------------------------------                                                //
// WIFI WEBPAGE 
//--------------------------------------------
void StartWIFI_NTP(void)
{
// WiFi.disconnect(true);
// WiFi.onEvent(WiFiEvent);   // Using WiFi.onEvent interrupts and crashes screen display while writing the screen
// Examples of different ways to register wifi events
                         //  WiFi.onEvent(WiFiGotIP, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_GOT_IP);
                         //  WiFiEventId_t eventID = WiFi.onEvent([](WiFiEvent_t event, WiFiEventInfo_t info)
                         //    {Serial.print("WiFi lost connection. Reason: ");  Serial.println(info.wifi_sta_disconnected.reason); }, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_DISCONNECTED);

 
 WiFi.mode(WIFI_STA);
 WiFi.begin(Mem.Ssid, Mem.Password);
 if (WiFi.waitForConnectResult() != WL_CONNECTED) 
      { 
       if(Mem.WIFINoOfRestarts == 0)                                                           // Try once to restart the ESP and make a new WIFI connection
       {
          Mem.WIFINoOfRestarts = 1;
          StoreStructInFlashMemory();                                                          // Update Mem struct   
 //         ESP.restart(); 
       }
       else
       {
         Mem.WIFINoOfRestarts = 0;
         StoreStructInFlashMemory();                                                           // Update Mem struct   
         Displayprintln("WiFi Failed!");                                                       // Print the text on the display
         Tekstprintln("WiFi Failed! Enter @ to retry"); 
         WIFIConnected = 0;       
         return;
       }
      }
 else 
      {
       Tekstprint("Web page started\n");
       sprintf(sptext, "IP Address: %d.%d.%d.%d", WiFi.localIP()[0], WiFi.localIP()[1], WiFi.localIP()[2], WiFi.localIP()[3] );
       Displayprintln(sptext);
       Tekstprintln(sptext); 
       WIFIConnected = 1;
       Mem.WIFINoOfRestarts = 0;
       StoreStructInFlashMemory();                                                            // Update Mem struct   
       Displayprintln("WIFI is On"); 
       AsyncElegantOTA.begin(&server);                                                        // Start ElegantOTA       
       if(Mem.NTPOnOff)
          {
           NTP.setTimeZone(Mem.Timezone);                                                     // TZ_Europe_Amsterdam); //\TZ_Etc_GMTp1); // TZ_Etc_UTC 
           NTP.begin();                                                                       // https://raw.githubusercontent.com/nayarsystems/posix_tz_db/master/zones.csv
           Tekstprintln("NTP is On"); 
           Displayprintln("NTP is On");                                                       // Print the text on the display
          }
//       PrintIPaddressInScreen();
      }
 if(Mem.WIFIOnOff) WebPage();                                                                 // Show the web page if WIFI is on
}
//--------------------------------------------                                                //
// WIFI WEBPAGE 
//--------------------------------------------
void WebPage(void) 
{
 server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)                                  // Send web page with input fields to client
          { request->send_P(200, "text/html", index_html);  }    );
 server.on("/get", HTTP_GET, [] (AsyncWebServerRequest *request)                              // Send a GET request to <ESP_IP>/get?input1=<inputMessage>
       { 
        String inputMessage;    String inputParam;
        if (request->hasParam(PARAM_INPUT_1))                                                 // GET input1 value on <ESP_IP>/get?input1=<inputMessage>
           {
            inputMessage = request->getParam(PARAM_INPUT_1)->value();
            inputParam = PARAM_INPUT_1;
           }
        else 
           {
            inputMessage = "";    //inputMessage = "No message sent";
            inputParam = "none";
           }  
        inputMessage.toCharArray(sptext, inputMessage.length());  
   //     sprintf(sptext,"%s",inputMessage);    
        Tekstprintln(sptext);
 //  **** testen     ReworkInputString(sptext+"\n"); // error: invalid operands of types 'char [140]' and 'const char [2]' to binary 'operator+'
        ReworkInputString(sptext);        // Is the \n needed from line above??

        request->send_P(200, "text/html", index_html);
       }   );
 server.onNotFound(notFound);
 server.begin();
}

//--------------------------------------------
// WIFI WEBPAGE 
//--------------------------------------------
void notFound(AsyncWebServerRequest *request) 
{
  request->send(404, "text/plain", "Not found");
}
//                                                                                            //
