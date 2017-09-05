#include <Arduino.h>

/**
  SePoLab SMART Devices PROJECT
  Name: IRControlerNode
  Purpose: Give command to Air Conditioner as Remote Controller
  General Hardware:
  - BASE: 
  -- WeMosD1mini/WeMosD1miniPro(required if supported upgrade firmware via OTA) / WeMosD1Lite (no OTA support)
  - SHIELD:
  -- DYI IR remote shield (required) - used Port: D7
  -- 1-BUTTON shield (optional) - used Port: D3
  -- RGB LED Shield (optional) - used Port: D2
  - CONNECTION:
  -- SOFT RESET BUTTON: D3 => use to reset WiFi or MQTT server connection
  -- LED INDICATER: D2 => use to inform status of Connection

Pin Function                          ESP-8266 Pin
TX  TXD                                     TXD
RX  RXD                                     RXD
A0  Analog input                            A0
D0  IO                                      GPIO16
D1  IO, SCL                                 GPIO5
D2  IO, SDA                                 GPIO4
D3  IO, 10k Pull-up                         GPIO0
D4  IO, 10k Pull-up, BUILTIN_LED            GPIO2
D5  IO, SCK                                 GPIO14
D6  IO, MISO                                GPIO12
D7  IO, MOSI                                GPIO13
D8  IO, 10k Pull-down, SS                   GPIO15
G   Ground                                  GND
5V  5V                                      -
3V3 3.3V                                    3.3V
RST Reset                                   RST

  General Sofware:
  - Connect to MQTT Server: will input in WiFi Configuration.
  -- Subcrible topic: <ProjectName>/<MacAddress>/set
  -- Publish topic: <ProjectName>/<MacAddress>/state
  -- Command from MQTT Broker = {"DT":"AC/WF","B":"Toshiba","State":"on","M":"cool","T":"27","F":"max","S":"n"}
M
  @author Sebastian sepolab@gmail.com
  @version 1.0 6/15/17
########################################
  - OTA Update: basic update via ArduinoOTA library
  - Impacted on:
    -- setup_wifi(): configurate OTA
    -- loop(): looping OTA checking
  @version 1.1 7/15/17
*/

#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager version 0.11.0
#include <ESP8266WiFi.h>          //https://github.com/esp8266/Arduino version 1.0.0
#include <DNSServer.h>            // version 1.1.0
#include <ESP8266WebServer.h>     // version 1.0.0
#include <ArduinoJson.h>          //https://github.com/bblanchon/ArduinoJson version 5.1.0
#include <FS.h>
#include <PubSubClient.h> //April23 version 2.6.0
#include "math.h"
#include <Adafruit_NeoPixel.h>    //https://github.com/adafruit/Adafruit_NeoPixel version 1.1.1
#include <IRremoteESP8266.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <IRsend.h>
#include <ir_Daikin.h>


IRDaikinESP dakinir(D7);
IRsend irsend(D7); //an IR led is connected to GPIO pin 13 -D7
const unsigned int WFPaPo[] PROGMEM = {3496,3468,868,2612,868,2608,872,872,868,2612,868,2612,868,2616,864,876,916,824,868,876,860,876,868,868,868,2616,868,872,868,872,868,2612,916,876,816,872,868,876,864,2612,868,2612,868,2616,864,2612,924,2556,920,820,868,34764,3536,3424,868,2616,868,2608,868,876,868,2608,868,2616,868,2612,868,920,820,920,820,924,816,920,820,876,864,2612,868,872,868,876,864,2616,864,876,920,868,872,868,816,2612,868,2616,864,2612,868,2616,868,2612,920,820,916,34712,3536,3424,872,2612,864,2616,868,876,860,2616,868,2612,868,2616,864,876,864,876,864,876,864,900,840,900,840,2612,868,876,868,872,868,2612,916,828,864,872,864,876,864,2616,868,2616,864,2612,864,2616,868,2612,868,872,868};
const unsigned int WFPaSp[] PROGMEM = {3480,3480,872,2608,872,2612,868,872,868,2616,864,2616,864,2616,868,2608,916,2568,868,2612,864,2616,864,2616,868,876,864,872,920,820,920,2556,868,884,860,872,868,872,868,876,864,872,868,872,868,876,864,872,868,2616,864,34760,3484,3480,868,2612,872,2612,868,872,868,2612,868,2608,872,2612,868,2612,868,2616,864,2616,916,2560,868,2612,920,872,816,924,816,872,868,2612,868,876,864,924,820,920,816,876,864,880,860,924,820,920,820,872,868,2612,868,34760,3484,3480,868,2612,868,2612,868,876,916,2564,864,2616,864,2612,868,2612,868,2616,864,2616,868,2608,868,2616,868,872,868,872,868,872,868,2612,868,872,868,924,816,948,792,872,920,872,816,872,868,880,864,872,920,2560,864};
const unsigned int WFPaOs[] PROGMEM = {3480,3480,868,2612,872,2612,868,872,868,2608,872,2612,868,2612,868,2616,864,880,860,2612,868,2612,868,2612,868,880,864,876,864,876,860,2608,876,872,868,872,868,876,864,876,864,2616,864,872,864,880,864,872,868,2612,868,34760,3508,3456,868,2612,868,2612,868,876,868,2608,872,2608,872,2608,868,2612,872,872,868,2616,864,2612,868,2612,868,872,868,872,868,872,868,2612,868,872,872,876,860,880,860,872,872,2608,872,876,864,876,864,872,868,2608,872,34760,3484,3480,868,2608,872,2612,868,876,864,2612,868,2612,868,2612,896,2588,868,868,872,2612,868,2608,868,2616,864,880,864,876,860,876,916,2564,868,872,868,876,864,876,864,876,864,2608,872,872,872,876,860,876,864,2612,868};
const unsigned int WFPaTi[] PROGMEM = {3540,3424,868,2612,868,2612,868,876,864,2612,920,2564,864,2616,864,876,864,2616,868,2612,920,2560,868,2612,868,876,864,880,860,900,840,2612,868,880,860,876,864,876,864,2612,868,880,908,828,864,876,864,876,868,2612,868,34760,3536,3424,872,2612,868,2616,864,880,860,2612,868,2616,864,2612,868,880,864,2616,864,2612,868,2612,868,2612,864,876,920,824,864,876,864,2612,868,872,868,924,816,924,820,2608,924,820,916,824,864,876,916,824,868,2612,912,34716,3536,3428,864,2616,868,2616,864,924,816,2616,864,2612,868,2612,868,872,868,2616,864,2612,868,2616,864,2612,868,876,920,820,864,876,864,2616,868,872,864,948,796,920,816,2616,868,872,868,872,868,876,864,880,860,2612,868};
const unsigned int WFPaRh[] PROGMEM = {3508,3484,864,2616,888,2592,864,872,868,2620,860,2612,868,2620,888,916,796,876,868,2608,896,2588,868,2616,864,872,868,876,864,876,916,2604,824,920,820,876,864,872,896,2612,840,2612,868,876,864,876,864,872,868,2616,864,34764,3480,3480,872,2608,872,2612,868,872,868,2616,864,2612,868,2612,868,872,868,872,868,2612,872,2608,872,2612,892,848,868,872,888,852,868,2612,868,876,864,872,868,876,864,2612,868,2608,872,872,868,872,868,876,868,2608,900,34732,3504,3460,868,2612,864,2616,868,896,844,2608,872,2612,868,2608,868,900,844,896,844,2616,892,2588,864,2616,888,876,840,872,868,876,864,2612,868,900,868,872,840,876,864,2616,864,2616,892,876,836,900,868,872,840,2620,888};

const unsigned int WFMiPo[] PROGMEM = {3248,1616,352,1252,356,1252,324,424,392,448,368,400,392,1248,356,396,420,396,392,1248,360,1228,372,400,392,1248,352,472,348,396,392,1252,352,1232,376,400,388,1208,396,1232,372,468,324,1204,400,464,352,448,340,400,416,396,420,400,388,1252,352,400,420,396,392,420,396,1252,352,396,392,420,396,468,348,468,320,420,396,420,400,392,392,404,416,1212,392,1252,324,468,348,396,420,1252,324,468,352,420,392,400,392,1248,356,1232,372,396,396,420,396,396,420,392,396,396,416,400,420,396,392,396,420,396,420,396,392,420,400,1248,352,444,348,392,420,1232,376};
const unsigned int WFMiSp[] PROGMEM = {3196,1624,372,1248,356,1232,376,396,392,396,420,396,420,1252,324,468,348,400,416,1252,328,1248,356,400,416,1252,324,420,396,400,416,1256,324,1252,352,444,372,1248,332,1204,400,396,420,1252,324,400,420,444,368,400,392,396,420,444,372,1204,372,464,352,396,420,396,392,1208,396,396,420,428,360,400,420,396,420,400,388,420,396,396,420,396,392,1252,352,1252,352,400,388,468,348,1256,352,400,388,468,348,468,348,1248,328,448,372,1252,352,396,392,400,416,400,416,400,388,400,416,400,416,396,392,428,388,396,424,396,388,428,388,1232,376,464,324,1208,396};
const unsigned int WFMiOs[] PROGMEM = {3216,1580,396,1184,420,1184,420,400,388,396,420,396,420,1184,396,400,416,396,420,1184,392,1184,424,396,416,1188,392,396,420,396,420,1184,392,1184,424,392,420,1188,392,1188,416,400,416,1188,392,392,424,396,420,396,392,396,420,392,424,1184,396,392,420,400,416,396,396,1188,416,396,420,392,396,400,416,400,416,396,392,400,416,400,416,396,392,1184,424,1180,424,396,392,400,416,1188,416,404,388,396,416,400,420,1180,396,1184,420,1188,420,400,388,392,424,396,420,396,392,396,420,396,416,400,392,396,420,396,420,396,392,1184,420,1188,420,392,396,1188,416};
const unsigned int WFMiTi[] PROGMEM = {3236,1568,420,1188,420,1188,388,396,424,392,420,396,392,1188,420,400,416,396,392,1208,396,1184,420,396,396,1184,416,396,424,396,392,1208,396,1188,420,396,388,1212,396,1184,420,420,372,1184,420,396,420,400,388,396,420,396,420,396,392,1184,420,400,416,404,388,392,420,1188,420,400,388,396,420,396,420,400,388,396,420,396,420,400,388,420,396,1192,412,1184,396,396,420,392,424,1184,392,396,420,396,420,400,392,1208,396,392,424,392,396,1188,416,400,416,400,388,396,420,400,416,396,396,396,416,400,420,396,388,400,420,400,416,396,392,1184,420,1184,420};
const unsigned int WFMiRh[] PROGMEM = {3196,1600,396,1248,356,1232,372,400,392,396,420,396,420,1232,344,420,396,420,396,1252,324,1252,356,444,372,1228,348,468,348,428,388,1248,332,1204,400,396,420,1232,344,1248,360,444,368,1252,328,420,396,468,348,444,344,400,416,448,368,1248,332,464,352,396,420,396,392,1204,400,444,372,468,320,468,348,400,416,400,388,424,396,444,372,396,392,1248,352,1252,356,464,324,420,396,1252,352,404,388,400,416,396,420,1228,348,1252,356,464,348,1232,348,464,352,396,420,396,392,464,352,468,348,396,392,420,396,468,348,468,320,1252,352,468,352,1248,328,1248,356};


//---------SOFT RESET BUTTON PARAMETERS---------------------
const int SOFT_RST_PIN = D3;      // SOFT RESET button is map to port 0
int buttonState;             // the current reading from the input pin
int lastButtonState = LOW;   // the previous reading from the input pin
// the following variables are long's because the time, measured in miliseconds,
// will quickly become a bigger number than can be stored in an int.
long lastDebounceTime = 0;  // the last time the output pin was toggled
long debounceDelay = 1000;    // the debounce time; increase if the output flickers
unsigned long previousMillis2 = 0;
//--------LED INDICATION PARAMETERS--------------------
#define PINLED D2

// When we setup the NeoPixel library, we tell it how many pixels, and which pin to use to send signals.
// Note that for older NeoPixel strips you might need to change the third parameter--see the strandtest
// example for more information on possible values.
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(1, PINLED, NEO_GRB + NEO_KHZ800);
//----------------WIFI CHECK STATE PAMAMETERS----------------
int check = 0;
bool checkWifi = false; //wifi connect indication 0: disconnect; 1: connected
bool mqttReconnecting = false;
//------------WIFI CONFIGURATION PARAMETERS--------------------
bool factoryReset = false;
char APssid[20] = ""; //AP is ESP will connect
char APpassword[20] = "";
const char ssid[20] = ""; // ESP in AP mode
const char password[] = "password";
WiFiClient espClient; //April23
byte mac[6];
//--------------MQTT client PRAMETERS--------------
//define mqtt server default values here, if there are different values in config.json, they are overwritten.
char mqttServer[40] = ""; //April24
char mqttPort[6] = "1883"; //April24
char projectName[40] = "";
char blynk_token[34] = "YOUR_BLYNK_TOKEN";
//flag for saving data
bool shouldSaveConfig = false;
PubSubClient client(espClient);//April23
unsigned long previousMillis3 = 0; //set time for interval publish
unsigned long previousMillis4 = 0; //set time for interval publish
//char mqttClientID[] = ssid;
char pubTopicGen[50] = "";
char subTopicGen[50] = "";
int valueOfSensor = 0;// value that will be published
char pubMsg[150] = "Hello Server,it is client's 1st message!";//payload of publishing message
char subMsg[150]; //payload of subcribled message
long publishInveral = 15000;
bool mqttConnected = false;
const long reconnectInveral = 10000;
bool firstTime = true;
char temptMac[20] = "";
//------------------------
bool isDefinedCommand = true;
bool simulatedDropedWifi = false;
char ID[20];
char DeviceType[5];
//----------AC CONFIGURATION---------------
char ACID[20];
char ACstate[4] = "Off"; // on or off
char ACmode[5] = "Cool"; // cool; dry; fan; auto
char ACtemp[3] = "27"; // from 16 - 30
char ACfan[5] = "Max"; // min, med, max, auto
char ACswing[4] = "Off"; // On/Off
char finalResult[10];
//----------WF CONFIGURATION---------------
char WFID[20];
char WFCommand[7] = "none"; // on or off
char finResult[7];
// END OF VARIABLES--------------------------
// variable for LG AC
const unsigned int kAc_Type  = 1; // 0 : TOWER // 1 : WALL
unsigned int ac_heat = 1; // 0 : cooling // 1 : heating
unsigned int ac_power_on = 0; // 0 : off // 1 : on
unsigned int ac_air_swing = 0;// 0 : off // 1 : on
unsigned int ac_air_clean_state = 0; // 0 : off // 1 : on --> power on
unsigned int ac_temperature = 24;// temperature : 18 ~ 30
unsigned int ac_flow = 0; // 0 : low // 1 : mid // 2 : high // if kAc_Type = 1, 3 : change
const uint8_t kAc_Flow_Tower[3] = {0, 4, 6};
const uint8_t kAc_Flow_Wall[4] = {0, 2, 4, 5};
uint32_t ac_code_to_sent;

// variable for Toshiba AC
int halfPeriodicTime;
int IRpin;
int khz;


typedef enum HvacMode {
  HVAC_HOT,
  HVAC_COLD,
  HVAC_DRY,
  HVAC_FAN, // used for Panasonic only
  HVAC_AUTO
} HvacMode_t; // HVAC  MODE

typedef enum HvacFanMode {
  FAN_SPEED_1,
  FAN_SPEED_2,
  FAN_SPEED_3,
  FAN_SPEED_4,
  FAN_SPEED_5,
  FAN_SPEED_AUTO,
  FAN_SPEED_SILENT
} HvacFanMode_;  // HVAC  FAN MODE

typedef enum HvacVanneMode {
  VANNE_AUTO,
  VANNE_H1,
  VANNE_H2,
  VANNE_H3,
  VANNE_H4,
  VANNE_H5,
  VANNE_AUTO_MOVE
} HvacVanneMode_;  // HVAC  VANNE MODE

typedef enum HvacWideVanneMode {
  WIDE_LEFT_END,
  WIDE_LEFT,
  WIDE_MIDDLE,
  WIDE_RIGHT,
  WIDE_RIGHT_END
} HvacWideVanneMode_t;  // HVAC  WIDE VANNE MODE

typedef enum HvacAreaMode {
  AREA_SWING,
  AREA_LEFT,
  AREA_AUTO,
  AREA_RIGHT
} HvacAreaMode_t;  // HVAC  WIDE VANNE MODE

typedef enum HvacProfileMode {
  NORMAL,
  QUIET,
  BOOST
} HvacProfileMode_t;  // HVAC PANASONIC OPTION MODE


// HVAC TOSHIBA_
#define HVAC_TOSHIBA_HDR_MARK    4400
#define HVAC_TOSHIBA_HDR_SPACE   4300
#define HVAC_TOSHIBA_BIT_MARK    543
#define HVAC_TOSHIBA_ONE_SPACE   1623
#define HVAC_MISTUBISHI_ZERO_SPACE  472
#define HVAC_TOSHIBA_RPT_MARK    440
#define HVAC_TOSHIBA_RPT_SPACE   7048 // Above original iremote limit

// HVAC MITSUBISHI_
#define HVAC_MITSUBISHI_HDR_MARK    3400
#define HVAC_MITSUBISHI_HDR_SPACE   1750
#define HVAC_MITSUBISHI_BIT_MARK    450
#define HVAC_MITSUBISHI_ONE_SPACE   1300
#define HVAC_MISTUBISHI_ZERO_SPACE  420
#define HVAC_MITSUBISHI_RPT_MARK    440
#define HVAC_MITSUBISHI_RPT_SPACE   17100 // Above original iremote limit
/****************************************************************************
/* Send IR command to Mitsubishi HVAC - sendHvacMitsubishi
/***************************************************************************/
void sendHvacMitsubishi(
  HvacMode                HVAC_Mode,           // Example HVAC_HOT  HvacMitsubishiMode
  int                     HVAC_Temp,           // Example 21  (°c)
  HvacFanMode             HVAC_FanMode,        // Example FAN_SPEED_AUTO  HvacMitsubishiFanMode
  HvacVanneMode           HVAC_VanneMode,      // Example VANNE_AUTO_MOVE  HvacMitsubishiVanneMode
  int                     OFF                  // Example false
)
{

//#define  HVAC_MITSUBISHI_DEBUG;  // Un comment to access DEBUG information through Serial Interface

  byte mask = 1; //our bitmask
  byte data[18] = { 0x23, 0xCB, 0x26, 0x01, 0x00, 0x20, 0x08, 0x06, 0x30, 0x45, 0x67, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1F };
  // data array is a valid trame, only byte to be chnaged will be updated.

  byte i;

#ifdef HVAC_MITSUBISHI_DEBUG
  Serial.println("Packet to send: ");
  for (i = 0; i < 18; i++) {
    Serial.print("_");
    Serial.print(data[i], HEX);
  }
  Serial.println(".");
#endif

  // Byte 6 - On / Off
  if (OFF) {
    data[5] = (byte) 0x0; // Turn OFF HVAC
  } else {
    data[5] = (byte) 0x20; // Tuen ON HVAC
  }

  // Byte 7 - Mode
  switch (HVAC_Mode)
  {
    case HVAC_HOT:   data[6] = (byte) 0x08; break;
    case HVAC_COLD:  data[6] = (byte) 0x18; break;
    case HVAC_DRY:   data[6] = (byte) 0x10; break;
    case HVAC_AUTO:  data[6] = (byte) 0x20; break;
    default: break;
  }

  // Byte 8 - Temperature
  // Check Min Max For Hot Mode
  byte Temp;
  if (HVAC_Temp > 31) { Temp = 31;}
  else if (HVAC_Temp < 16) { Temp = 16; } 
  else { Temp = HVAC_Temp; };
  data[7] = (byte) Temp - 16;

  // Byte 10 - FAN / VANNE
  switch (HVAC_FanMode)
  {
    case FAN_SPEED_1:       data[9] = (byte) B00000001; break;
    case FAN_SPEED_2:       data[9] = (byte) B00000010; break;
    case FAN_SPEED_3:       data[9] = (byte) B00000011; break;
    case FAN_SPEED_4:       data[9] = (byte) B00000100; break;
    case FAN_SPEED_5:       data[9] = (byte) B00000100; break; //No FAN speed 5 for MITSUBISHI so it is consider as Speed 4
    case FAN_SPEED_AUTO:    data[9] = (byte) B10000000; break;
    case FAN_SPEED_SILENT:  data[9] = (byte) B00000101; break;
    default: break;
  }

  switch (HVAC_VanneMode)
  {
    case VANNE_AUTO:        data[9] = (byte) data[9] | B01000000; break;
    case VANNE_H1:          data[9] = (byte) data[9] | B01001000; break;
    case VANNE_H2:          data[9] = (byte) data[9] | B01010000; break;
    case VANNE_H3:          data[9] = (byte) data[9] | B01011000; break;
    case VANNE_H4:          data[9] = (byte) data[9] | B01100000; break;
    case VANNE_H5:          data[9] = (byte) data[9] | B01101000; break;
    case VANNE_AUTO_MOVE:   data[9] = (byte) data[9] | B01111000; break;
    default: break;
  }

  // Byte 18 - CRC
  data[17] = 0;
  for (i = 0; i < 17; i++) {
    data[17] = (byte) data[i] + data[17];  // CRC is a simple bits addition
  }

#ifdef HVAC_MITSUBISHI_DEBUG
  Serial.println("Packet to send: ");
  for (i = 0; i < 18; i++) {
    Serial.print("_"); Serial.print(data[i], HEX);
  }
  Serial.println(".");
  for (i = 0; i < 18; i++) {
    Serial.print(data[i], BIN); Serial.print(" ");
  }
  Serial.println(".");
#endif

  enableIROut(38);  // 38khz
  space(0);
  for (int j = 0; j < 2; j++) {  // For Mitsubishi IR protocol we have to send two time the packet data
    // Header for the Packet
    mark(HVAC_MITSUBISHI_HDR_MARK);
    space(HVAC_MITSUBISHI_HDR_SPACE);
    for (i = 0; i < 18; i++) {
      // Send all Bits from Byte Data in Reverse Order
      for (mask = 00000001; mask > 0; mask <<= 1) { //iterate through bit mask
        if (data[i] & mask) { // Bit ONE
          mark(HVAC_MITSUBISHI_BIT_MARK);
          space(HVAC_MITSUBISHI_ONE_SPACE);
        }
        else { // Bit ZERO
          mark(HVAC_MITSUBISHI_BIT_MARK);
          space(HVAC_MISTUBISHI_ZERO_SPACE);
        }
        //Next bits
      }
    }
    // End of Packet and retransmission of the Packet
    mark(HVAC_MITSUBISHI_RPT_MARK);
    space(HVAC_MITSUBISHI_RPT_SPACE);
    space(0); // Just to be sure
  }
}

/****************************************************************************
/* enableIROut : Set global Variable for Frequency IR Emission
/***************************************************************************/ 
void enableIROut(int khz) {
  // Enables IR output.  The khz value controls the modulation frequency in kilohertz.
  halfPeriodicTime = 500/khz; // T = 1/f but we need T/2 in microsecond and f is in kHz
}

/****************************************************************************
/* mark ( int time) 
/***************************************************************************/ 
void mark(int time) {
  // Sends an IR mark for the specified number of microseconds.
  // The mark output is modulated at the PWM frequency.
  long beginning = micros();
  while(micros() - beginning < time){
    digitalWrite(IRpin, HIGH);
    delayMicroseconds(halfPeriodicTime);
    digitalWrite(IRpin, LOW);
    delayMicroseconds(halfPeriodicTime); //38 kHz -> T = 26.31 microsec (periodic time), half of it is 13
  }
}

/****************************************************************************
/* space ( int time) 
/***************************************************************************/ 
/* Leave pin off for time (given in microseconds) */
void space(int time) {
  // Sends an IR space for the specified number of microseconds.
  // A space is no output, so the PWM output is disabled.
  digitalWrite(IRpin, LOW);
  if (time > 0) delayMicroseconds(time);
}

/****************************************************************************
/* sendRaw (unsigned int buf[], int len, int hz)
/***************************************************************************/ 
void sendRaw (unsigned int buf[], int len, int hz)
{
  enableIROut(hz);
  for (int i = 0; i < len; i++) {
    if (i & 1) {
      space(buf[i]);
    } 
    else {
      mark(buf[i]);
    }
  }
  space(0); // Just to be sure
}

/****************************************************************************
/* Send IR command to Toshiba HVAC - sendHvacToshiba
/***************************************************************************/
void sendHvacToshiba(
  HvacMode                HVAC_Mode,           // Example HVAC_HOT  
  int                     HVAC_Temp,           // Example 21  (°c)
  HvacFanMode             HVAC_FanMode,        // Example FAN_SPEED_AUTO  
  int                     OFF                  // Example false
)
{ 
#define HVAC_TOSHIBA_DATALEN 9
#define  HVAC_TOSHIBA_DEBUG;  // Un comment to access DEBUG information through Serial Interface

  byte mask = 1; //our bitmask
  //﻿F20D03FC0150000051
  byte data[HVAC_TOSHIBA_DATALEN] = { 0xF2, 0x0D, 0x03, 0xFC, 0x01, 0x00, 0x00, 0x00, 0x00 };
  // data array is a valid trame, only byte to be chnaged will be updated.

  byte i;

#ifdef HVAC_TOSHIBA_DEBUG
  Serial.println("Packet to send: ");
  for (i = 0; i < HVAC_TOSHIBA_DATALEN; i++) {
    Serial.print("_");
    Serial.print(data[i], HEX);
  }
  Serial.println(".");
#endif

  data[6] = 0x00;
  // Byte 7 - Mode
  switch (HVAC_Mode)
  {
    case HVAC_HOT:   data[6] = (byte) B00000011; break;
    case HVAC_COLD:  data[6] = (byte) B00000001; break;
    case HVAC_DRY:   data[6] = (byte) B00000010; break;
    case HVAC_AUTO:  data[6] = (byte) B00000000; break;
    default: break;
  }


  // Byte 7 - On / Off
  if (OFF) {
    data[6] = (byte) 0x07; // Turn OFF HVAC
  } else {
     // Turn ON HVAC (default)
  }

  // Byte 6 - Temperature
  // Check Min Max For Hot Mode
  byte Temp;
  if (HVAC_Temp > 30) { Temp = 30;}
  else if (HVAC_Temp < 17) { Temp = 17; } 
  else { Temp = HVAC_Temp; };
  data[5] = (byte) Temp - 17<<4;

  // Byte 10 - FAN / VANNE
  switch (HVAC_FanMode)
  {
    case FAN_SPEED_1:       data[6] = data[6] | (byte) B01000000; break;
    case FAN_SPEED_2:       data[6] = data[6] | (byte) B01100000; break;
    case FAN_SPEED_3:       data[6] = data[6] | (byte) B10000000; break;
    case FAN_SPEED_4:       data[6] = data[6] | (byte) B10100000; break;
    case FAN_SPEED_5:       data[6] = data[6] | (byte) B11000000; break; 
    case FAN_SPEED_AUTO:    data[6] = data[6] | (byte) B00000000; break;
    case FAN_SPEED_SILENT:  data[6] = data[6] | (byte) B00000000; break;//No FAN speed SILENT for TOSHIBA so it is consider as Speed AUTO
    default: break;
  }

  // Byte 9 - CRC
  data[8] = 0;
  for (i = 0; i < HVAC_TOSHIBA_DATALEN - 1; i++) {
    data[HVAC_TOSHIBA_DATALEN-1] = (byte) data[i] ^ data[HVAC_TOSHIBA_DATALEN -1];  // CRC is a simple bits addition
  }

#ifdef HVAC_TOSHIBA_DEBUG
  Serial.println("Packet to send: ");
  for (i = 0; i < HVAC_TOSHIBA_DATALEN; i++) {
    Serial.print("_"); Serial.print(data[i], HEX);
  }
  Serial.println(".");
  for (i = 0; i < HVAC_TOSHIBA_DATALEN ; i++) {
    Serial.print(data[i], BIN); Serial.print(" ");
  }
  Serial.println(".");
#endif

  enableIROut(38);  // 38khz
  space(0);
  for (int j = 0; j < 2; j++) {  // For Mitsubishi IR protocol we have to send two time the packet data
    // Header for the Packet
    mark(HVAC_TOSHIBA_HDR_MARK);
    space(HVAC_TOSHIBA_HDR_SPACE);
    for (i = 0; i < HVAC_TOSHIBA_DATALEN; i++) {
      // Send all Bits from Byte Data in Reverse Order
      for (mask = 10000000; mask > 0; mask >>= 1) { //iterate through bit mask
        if (data[i] & mask) { // Bit ONE
          mark(HVAC_TOSHIBA_BIT_MARK);
          space(HVAC_TOSHIBA_ONE_SPACE);
        }
        else { // Bit ZERO
          mark(HVAC_TOSHIBA_BIT_MARK);
          space(HVAC_MISTUBISHI_ZERO_SPACE);
        }
        //Next bits
      }
    }
    // End of Packet and retransmission of the Packet
    mark(HVAC_TOSHIBA_RPT_MARK);
    space(HVAC_TOSHIBA_RPT_SPACE);
    space(0); // Just to be sure
  }
}

//------------------------------------------------------------
//PROCEDURE: CONVERT AC REQUEST TO AC VARIABLE NAME
//------------------------------------------------------------
//PROCESS BY DATE:
// JUL22,2016: 
//    
//------------------------------------------------------------
bool convertToWFVariableName(char brand[],char comm[]) {
  bool isValid = true;
  char bufB[3] = "00";
  if ((brand[0] == 'M') and (brand[1] == 'i') and (brand[2] == 't') and (brand[3] == 's') and (brand[4] == 'h') and (brand[5] == 'u') and (brand[6] == 'b'))
    {snprintf(bufB, 3,"Mi");}
  else if ((brand[0] == 'P') and (brand[1] == 'a') and (brand[2] == 'n') and (brand[3] == 'a') and (brand[4] == 's') and (brand[5] == 'o') and (brand[6] == 'n') and (brand[7] == 'i') and (brand[8] == 'c'))
    {snprintf(bufB, 3,"Pa");}
  else if ((brand[0] == 'D') and (brand[1] == 'a') and (brand[2] == 'i') and (brand[3] == 'k') and (brand[4] == 'i') and (brand[5] == 'n'))
    {snprintf(bufB, 3,"Da");}
  else {isValid == false;}

  char bufC[3] = "00";
  if ((comm[0] == 'p') and (comm[1] == 'o') and (comm[2] == 'w') and (comm[3] == 'e') and (comm[4] == 'r')) {snprintf(bufC, 3,"Po");}
  else if ((comm[0] == 's') and (comm[1] == 'p') and (comm[2] == 'e') and (comm[3] == 'e') and (comm[4] == 'd')) {snprintf(bufC, 3,"Sp");}
  else if ((comm[0] == 'o') and (comm[1] == 's') and (comm[2] == 'c') and (comm[3] == 'i') and (comm[4] == 'l')) {snprintf(bufC, 3,"Os");}
  else if ((comm[0] == 't') and (comm[1] == 'i') and (comm[2] == 'm') and (comm[3] == 'e') and (comm[4] == 'r')) {snprintf(bufC, 3,"Ti");}
  else if ((comm[0] == 'r') and (comm[1] == 'h') and (comm[2] == 'y') and (comm[3] == 't') and (comm[4] == 'h')) {snprintf(bufC, 3,"Rh");}
  else {isValid == false;}

  if (isValid) {
    snprintf(finResult, 7,"WF%s%s",bufB,bufC);
    Serial.print("Covert to variable Name: ");Serial.println(finResult);
    return true;
    isDefinedCommand = true;
  } else {
    return false;
    snprintf(finResult, 7,"failed");
    isDefinedCommand = false;
  }
}
void adjustWF() {
//  if (readReceivedMsgInWFJson(breakedValue)) { // PARSE DATA SUCCESSFULLY
      if (convertToWFVariableName(WFID,WFCommand)) {
            Serial.print("IR sent code: ");
            if ((finResult[0] == 'W') and (finResult[1] == 'F')) {
              if ((finResult[2] == 'M') and (finResult[3] == 'i')) {
                if ((finResult[4] == 'P') and (finResult[5] == 'o')) {
                  Serial.println("WFMiPo");
                  sendRAW_Flash(WFMiPo, sizeof(WFMiPo)/sizeof(int),38);
                } else
                if ((finResult[4] == 'S') and (finResult[5] == 'p')) {
                  Serial.println("WFMiSp");
                  sendRAW_Flash(WFMiSp, sizeof(WFMiSp)/sizeof(int),38);
                } else
                if ((finResult[4] == 'O') and (finResult[5] == 's')) {
                  Serial.println("WFMiOs");
                  sendRAW_Flash(WFMiOs, sizeof(WFMiOs)/sizeof(int),38);
                } else
                if ((finResult[4] == 'T') and (finResult[5] == 'i')) {
                  Serial.println("WFMiTi");
                  sendRAW_Flash(WFMiTi, sizeof(WFMiTi)/sizeof(int),38);
                } else
                if ((finResult[4] == 'R') and (finResult[5] == 'h')) {
                  Serial.println("WFMiRh");
                  sendRAW_Flash(WFMiRh, sizeof(WFMiRh)/sizeof(int),38);
                } else { Serial.println("WFMi Not Found");}
              } else 
              if ((finResult[2] == 'P') and (finResult[3] == 'a')) {
                if ((finResult[4] == 'P') and (finResult[5] == 'o')) {
                  Serial.println("WFPaPo");
                  sendRAW_Flash(WFPaPo, sizeof(WFPaPo)/sizeof(int),38);
                } else
                if ((finResult[4] == 'S') and (finResult[5] == 'p')) {
                  Serial.println("WFPaSp");
                  sendRAW_Flash(WFPaSp, sizeof(WFPaSp)/sizeof(int),38);
                } else
                if ((finResult[4] == 'O') and (finResult[5] == 's')) {
                  Serial.println("WFPaOs");
                  sendRAW_Flash(WFPaOs, sizeof(WFPaOs)/sizeof(int),38);
                } else
                if ((finResult[4] == 'T') and (finResult[5] == 'i')) {
                  Serial.println("WFPaTi");
                  sendRAW_Flash(WFPaTi, sizeof(WFPaTi)/sizeof(int),38);
                } else
                if ((finResult[4] == 'R') and (finResult[5] == 'h')) {
                  Serial.println("WFPaRh");
                  sendRAW_Flash(WFPaRh, sizeof(WFPaRh)/sizeof(int),38);
                } else { Serial.println("WFPa Not Found");}
              }
            }
}}
// 
//------------------------------------------------------------
bool convertToACVariableName(char brand[],char Mode[],char temperature[], char fanLevel[], char swing[]) {
  bool isValid = true;
  char bufB[3] = "00";
  if ((brand[0] == 'T') and (brand[1] == 'o') and (brand[2] == 's') and (brand[3] == 'h') and (brand[4] == 'i') and (brand[5] == 'b') and (brand[6] == 'a'))
    {snprintf(bufB, 3,"To");}
  else if ((brand[0] == 'P') and (brand[1] == 'a') and (brand[2] == 'n') and (brand[3] == 'a') and (brand[4] == 's') and (brand[5] == 'o') and (brand[6] == 'n') and (brand[7] == 'i') and (brand[8] == 'c'))
    {snprintf(bufB, 3,"Pa");}
  else if ((brand[0] == 'D') and (brand[1] == 'a') and (brand[2] == 'i') and (brand[3] == 'k') and (brand[4] == 'i') and (brand[5] == 'n'))
    {snprintf(bufB, 3,"Da");}
  else {isValid == false;}
  
  char bufM[2] = "0";
  if ((Mode[0] == 'C') and (Mode[1] == 'o') and (Mode[2] == 'o') and (Mode[3] == 'l'))
    {snprintf(bufM, 2,"1");}
  else if ((Mode[0] == 'D') and (Mode[1] == 'r') and (Mode[2] == 'y'))
    {snprintf(bufM, 2,"2");}
  else if ((Mode[0] == 'F') and (Mode[1] == 'a') and (Mode[2] == 'n'))
    {snprintf(bufM, 2,"3");}
  else if ((Mode[0] == 'A') and (Mode[1] == 'u') and (Mode[2] == 't') and (Mode[3] == 'o'))
    {snprintf(bufM, 2,"4");}
  else {isValid == false;}

  char bufF[2] = "0";
  if ((fanLevel[0] == 'M') and (fanLevel[1] == 'i') and (fanLevel[2] == 'n')) {snprintf(bufF, 2,"1");}
  else if ((fanLevel[0] == 'M') and (fanLevel[1] == 'e') and (fanLevel[2] == 'd')) {snprintf(bufF, 2,"2");}
  else if ((fanLevel[0] == 'M') and (fanLevel[1] == 'a') and (fanLevel[2] == 'x')) {snprintf(bufF, 2,"3");}
  else if ((fanLevel[0] == 'A') and (fanLevel[1] == 'u') and (fanLevel[2] == 't') and (fanLevel[3] == 'o')) {snprintf(bufF, 2,"4");}
  else {isValid == false;}

  char bufS[2] = "0";
  if ((swing[0] == 'O') and (swing[1] == 'f')) {snprintf(bufS, 2,"0");}
  else if ((swing[0] == 'O') and (swing[1] == 'n')) {snprintf(bufS, 2,"1");}
  else {isValid == false;}

  if (isValid) {
    snprintf(finalResult, 10,"%s%s%s%s%s",bufB,bufM,temperature,bufF,bufS);
    Serial.print("Covert to variable Name: ");Serial.println(finalResult);
    return true;
    isDefinedCommand = true;
  } else {
    snprintf(finalResult, 10,"failed");
    return false;
    isDefinedCommand = false;
  }
}
//-------------------END PROCEDURE------------------------------
//PROCEDURE: CONVERT WF REQUEST TO WF VARIABLE NAME
//------------------------------------------------------------
//PROCESS BY DATE:
// JUL22,2016: convert brand Mitshubishi/Panasonic/Daikin to 2 first letters of brand; then with command in 2 letters.(
//    
// 
//------------------------------------------------------------
//-------------------END PROCEDURE------------------------------
//------------------------------------------------------------
//PROCEDURE: EMIT DATA FROM FLASH TO IR PIN
//------------------------------------------------------------
//PROCESS BY DATE:
// JUL22,2016: CREATED DATE LONG CODE SUPPORTED LENGTH = 600
//    
// 
//------------------------------------------------------------
void sendRAW_Flash(const unsigned int * signalArray, unsigned int signalLength, unsigned char carrierFreq) {

  irsend.enableIROut(carrierFreq); //initialise the carrier frequency for each signal to be sent
  
  for (unsigned int i=0;i<signalLength;i++){
    //tmp=pgm_read_word_near(&signalArray[i]);
   // tmp=cleanPanasonic(tmp); //not needed
    if (i & 1) irsend.space(pgm_read_word_near(&signalArray[i]));
    else irsend.mark(pgm_read_word_near(&signalArray[i]));
  }
  irsend.space(1);//make sure IR is turned off at end of signal

}
//-------------------END PROCEDURE------------------------------
//------------------------------------------------------------
//PROCEDURE: PARSING DATA OF RECEIVED MESSAGE IN WF JSON FORMAT WITH WF SYNTAX
//------------------------------------------------------------
//PROCESS BY DATE:
// JUL22,2016: CREATED DATE - JSON SYNTAX LOOKS LIKE = {"ID":"Panasonic","command":"power"} //command= power; speed; timer; oscil; rhyth
//  
// 
//------------------------------------------------------------
bool readReceivedMsgInWFJson (char inputString[]) {
  DynamicJsonBuffer  jsonBuffer;
  JsonObject& root = jsonBuffer.parseObject(inputString);

  // Test if parsing succeeds.
  if (!root.success()) {
    Serial.println("parseObject() failed");
    return false;
  }
  // Fetch values.
    snprintf(WFID, 20, root["Brand"]);
    snprintf(WFCommand, 7, root["command"]);

    Serial.println("Parsed Received Message successfully!");
    Serial.println("Variables:");
    Serial.print("WFID :"); Serial.println(WFID);
    Serial.print("WFCommand :"); Serial.println(WFCommand);
    return true;
}
//-------------------END PROCEDURE------------------------------
//------------------------------------------------------------
//PROCEDURE: PARSING DATA OF RECEIVED MESSAGE IN AC JSON FORMAT WITH AC SYNTAX
//------------------------------------------------------------
//PROCESS BY DATE:
// JUL22,2016: CREATED DATE - JSON SYNTAX LOOKS LIKE = {"DT":"AC/WF","B":"Toshiba","State":"on","M":"cool","T":"27","fanF":"max","S":"n"}
//    
// 
//------------------------------------------------------------
bool readReceivedMsgInACJson (char inputString[]) {
  DynamicJsonBuffer  jsonBuffer;
  JsonObject& root = jsonBuffer.parseObject(inputString);

  // Test if parsing succeeds.
  if (!root.success()) {
    Serial.println("parseObject() failed");
    return false;
  }
  // Fetch values.
    snprintf(ACID, 20, root["B"]);
    snprintf(ACstate, 4, root["State"]);
    snprintf(ACmode, 5, root["M"]);
    snprintf(ACtemp, 3, root["T"]);
    snprintf(ACfan, 5, root["F"]);
    snprintf(ACswing, 4, root["S"]);

    Serial.println("Parsed Received Message successfully!");
    Serial.println("Variables:");
    Serial.print("ACID :"); Serial.println(ACID);
    Serial.print("ACstate :"); Serial.println(ACstate);
    Serial.print("ACmode :"); Serial.println(ACmode);
    Serial.print("ACtemp :"); Serial.println(ACtemp);
    Serial.print("ACfan :"); Serial.println(ACfan);
    Serial.print("ACswing :"); Serial.println(ACswing);
    return true;
}

bool readReceivedMsgInJson (char inputString[]) {
  DynamicJsonBuffer  jsonBuffer;
  JsonObject& root = jsonBuffer.parseObject(inputString);
  Serial.println ("JsonObject root");
  // Test if parsing succeeds.
  if (!root.success()) {
    Serial.println("parseObject() failed");
    return false;
  } else {
  // Fetch values.
    snprintf(ID, 20, root["B"]);
    snprintf(DeviceType, 5, root["DT"]);

    Serial.println("Parsed Received Message successfully!");
    Serial.println("Variables:");
    Serial.print("Brand:"); Serial.println(ID);
    Serial.print("DeviceType:"); Serial.println(DeviceType);

    return true;
  }
}
//-------------------END PROCEDURE------------------------------
//------------------------------------------------------------
//PROCEDURE: VERIFY RECEIVED MESSAGE IS FOR CONTROLLING NODE
//------------------------------------------------------------
//PROCESS BY DATE:
// JUL22,2016: CREATED DATE
// the general received message will following format:
// {"DeviceType":"<kind>","Brand":"","state":"on","mode":"cool","temp":"27","fan":"max","swing":"n"}
// Required Parameter: "DeviceType", "Brand", "state"
// Optional Parameter for AC: mode, temp, fan, swing
// Optional Parameter for WF: command
//------------------------------------------------------------
//SendConfirm () will send feedback to MQTT to confirm the command 
//---------------
void sendConfirmtoRetained (char inputString[]) {
  firstTime = false;
  Serial.print("Message send: ");
  Serial.println(inputString);
  client.publish(pubTopicGen, inputString, true);
}

void Ac_Send_Code (uint32_t code) {
  Serial.print("code to send : ");
  Serial.print(code, BIN);
  Serial.print(" : ");
  Serial.println(code, HEX);
  irsend.sendLG(code, 28);
}

void Ac_Activate(unsigned int temperature, unsigned int air_flow,
                 unsigned int heat) {
  ac_heat = heat;
  unsigned int ac_msbits1 = 8;
  unsigned int ac_msbits2 = 8;
  unsigned int ac_msbits3 = 0;
  unsigned int ac_msbits4;
  if (ac_heat == 1)
    ac_msbits4 = 4;  // heating
  else
    ac_msbits4 = 0;  // cooling
  unsigned int ac_msbits5 =  (temperature < 15) ? 0 : temperature - 15;
  unsigned int ac_msbits6;

  if (0 <= air_flow && air_flow <= 2) {
    if (kAc_Type == 0)
      ac_msbits6 = kAc_Flow_Tower[air_flow];
    else
      ac_msbits6 = kAc_Flow_Wall[air_flow];
  }

  // calculating using other values
  unsigned int ac_msbits7 = (ac_msbits3 + ac_msbits4 + ac_msbits5 +
                             ac_msbits6) & B00001111;
  ac_code_to_sent = ac_msbits1 << 4;
  ac_code_to_sent = (ac_code_to_sent + ac_msbits2) << 4;
  ac_code_to_sent = (ac_code_to_sent + ac_msbits3) << 4;
  ac_code_to_sent = (ac_code_to_sent + ac_msbits4) << 4;
  ac_code_to_sent = (ac_code_to_sent + ac_msbits5) << 4;
  ac_code_to_sent = (ac_code_to_sent + ac_msbits6) << 4;
  ac_code_to_sent = (ac_code_to_sent + ac_msbits7);

  Ac_Send_Code(ac_code_to_sent);

  ac_power_on = 1;
  ac_temperature = temperature;
  ac_flow = air_flow;
}

void Ac_Change_Air_Swing(int air_swing) {
  if (kAc_Type == 0) {
    if (air_swing == 1) 
      ac_code_to_sent = 0x881316B;
    else
      ac_code_to_sent = 0x881317C;
  } else {
    if (air_swing == 1)
      ac_code_to_sent = 0x8813149;
    else
      ac_code_to_sent = 0x881315A;
  }
  Ac_Send_Code(ac_code_to_sent);
}

void Ac_Power_Down() {
  ac_code_to_sent = 0x88C0051;
  Ac_Send_Code(ac_code_to_sent);
  ac_power_on = 0;
}

void Ac_Air_Clean(int air_clean) {
  if (air_clean == '1')
    ac_code_to_sent = 0x88C000C;
  else
    ac_code_to_sent = 0x88C0084;
  Ac_Send_Code(ac_code_to_sent);
  ac_air_clean_state = air_clean;
}

void adjustLGAC() {

  //STATE ON or OFF
  if (ACstate[0] == 'O' and ACstate[1] == 'f' and ACstate[2] == 'f') {
    Ac_Power_Down(); 
  } else if (ACstate[0] == 'O' and ACstate[1] == 'n') {
 
    //Set SWING  VERTICAL
    if (ACswing[0] == 'y' and ACswing[1] == 'e' and ACswing[2] == 's') {
      if (ac_air_swing == 0) {
        Ac_Change_Air_Swing(1);
        ac_air_swing = 1;
      }
    } else
    if (ACswing[0] == 'n' and ACswing[1] == 'o') {
      if (ac_air_swing == 1) {
        Ac_Change_Air_Swing(0);
        ac_air_swing = 0;
      }
    }
    
    //set Mode
    if (ACmode[0] == 'C' and ACmode[1] == 'o' and ACmode[2] == 'o' and ACmode[3] == 'l') {
      ac_heat = 0;
    } else
    if (ACmode[0] == 'H' and ACmode[1] == 'e' and ACmode[2] == 'a' and ACmode[3] == 't') {
      ac_heat = 1;
    }
    
    //Set FAN
    if (ACfan[0] == 'M' and ACfan[1] == 'a' and ACfan[2] == 'x') {
      ac_flow = 2;
    } else
    if (ACfan[0] == 'M' and ACfan[1] == 'e' and ACfan[2] == 'd') {
      ac_flow = 1;
    } else
    if (ACfan[0] == 'M' and ACfan[1] == 'i' and ACfan[2] == 'n') {
      ac_flow = 0;
    }
    
    //set Temp
    int temptACTemp = (ACtemp[0] - '0') * 10 + (ACtemp[1] - '0');
    Serial.println(temptACTemp);
    ac_temperature = temptACTemp;
    Ac_Activate(ac_temperature, ac_flow, ac_heat);
    }
}

void adjustToshibaAC() {
HvacMode toshibaMode;
HvacFanMode toshibaFanMode;
int toshibaTemp = 24;
//STATE ON or OFF
  if (ACstate[0] == 'O' and ACstate[1] == 'f' and ACstate[2] == 'f') { sendHvacToshiba(HVAC_COLD, 24, FAN_SPEED_AUTO, true);} 
  else if (ACstate[0] == 'O' and ACstate[1] == 'n') { 
    
  //set Mode
  if (ACmode[0] == 'C' and ACmode[1] == 'o' and ACmode[2] == 'o' and ACmode[3] == 'l') {
    toshibaMode = HVAC_COLD;
  } else
  if (ACmode[0] == 'H' and ACmode[1] == 'e' and ACmode[2] == 'a' and ACmode[3] == 't') {
    toshibaMode = HVAC_HOT;
  } else
  if (ACmode[0] == 'A' and ACmode[1] == 'u' and ACmode[2] == 'y' and ACmode[3] == 'o') {
    toshibaMode = HVAC_AUTO;
  } else
  if (ACmode[0] == 'D' and ACmode[1] == 'r' and ACmode[2] == 'y') {
    toshibaMode = HVAC_DRY;
  }

  //Set FAN
  if (ACfan[0] == 'M' and ACfan[1] == 'a' and ACfan[2] == 'x') {
    toshibaFanMode = FAN_SPEED_5;
  } else
  if (ACfan[0] == 'M' and ACfan[1] == 'e' and ACfan[2] == 'd') {
    toshibaFanMode = FAN_SPEED_3;
  } else
  if (ACfan[0] == 'M' and ACfan[1] == 'i' and ACfan[2] == 'n') {
    toshibaFanMode = FAN_SPEED_1;
  } else
  if (ACfan[0] == 'A' and ACfan[1] == 'u' and ACfan[2] == 't') {
   toshibaFanMode = FAN_SPEED_AUTO;
  }
  
  //set Temp
  int temptACTemp = (ACtemp[0] - '0') * 10 + (ACtemp[1] - '0');
  toshibaTemp = temptACTemp;

  sendHvacToshiba(toshibaMode, toshibaTemp, toshibaFanMode, false);
  }
}

void adjustMitsubishiAC() {
HvacMode MitsubishiMode;
HvacFanMode MitsubishiFanMode;
int MitsubishiTemp = 24;
//STATE ON or OFF
  if (ACstate[0] == 'O' and ACstate[1] == 'f' and ACstate[2] == 'f') { sendHvacMitsubishi(HVAC_COLD, 24, FAN_SPEED_AUTO, VANNE_AUTO_MOVE, true);} 
  else if (ACstate[0] == 'O' and ACstate[1] == 'n') { 
    
  //set Mode
  if (ACmode[0] == 'C' and ACmode[1] == 'o' and ACmode[2] == 'o' and ACmode[3] == 'l') {
    MitsubishiMode = HVAC_COLD;
  } else
  if (ACmode[0] == 'H' and ACmode[1] == 'e' and ACmode[2] == 'a' and ACmode[3] == 't') {
    MitsubishiMode = HVAC_HOT;
  } else
  if (ACmode[0] == 'A' and ACmode[1] == 'u' and ACmode[2] == 'y' and ACmode[3] == 'o') {
    MitsubishiMode = HVAC_AUTO;
  } else
  if (ACmode[0] == 'D' and ACmode[1] == 'r' and ACmode[2] == 'y') {
    MitsubishiMode = HVAC_DRY;
  }

  //Set FAN
  if (ACfan[0] == 'M' and ACfan[1] == 'a' and ACfan[2] == 'x') {
    MitsubishiFanMode = FAN_SPEED_5;
  } else
  if (ACfan[0] == 'M' and ACfan[1] == 'e' and ACfan[2] == 'd') {
    MitsubishiFanMode = FAN_SPEED_3;
  } else
  if (ACfan[0] == 'M' and ACfan[1] == 'i' and ACfan[2] == 'n') {
    MitsubishiFanMode = FAN_SPEED_1;
  } else
  if (ACfan[0] == 'A' and ACfan[1] == 'u' and ACfan[2] == 't') {
   MitsubishiFanMode = FAN_SPEED_AUTO;
  }
  
  //set Temp
  int temptACTemp = (ACtemp[0] - '0') * 10 + (ACtemp[1] - '0');
  MitsubishiTemp = temptACTemp;

  sendHvacMitsubishi(MitsubishiMode, MitsubishiTemp, MitsubishiFanMode,VANNE_AUTO_MOVE, false);
  }
}

void adjustDaikinAC() {

  //STATE ON or OFF
  if (ACstate[0] == 'O' and ACstate[1] == 'n') { dakinir.on();} 
  else if (ACstate[0] == 'O' and ACstate[1] == 'f' and ACstate[2] == 'f') { dakinir.off();} 

  //set Mode
  if (ACmode[0] == 'C' and ACmode[1] == 'o' and ACmode[2] == 'o' and ACmode[3] == 'l') {
    dakinir.setMode(DAIKIN_COOL);
  } else
  if (ACmode[0] == 'H' and ACmode[1] == 'e' and ACmode[2] == 'a' and ACmode[3] == 't') {
    dakinir.setMode(DAIKIN_HEAT);
  } else
  if (ACmode[0] == 'F' and ACmode[1] == 'a' and ACmode[2] == 'n') {
    dakinir.setMode(DAIKIN_FAN);
  } else
  if (ACmode[0] == 'A' and ACmode[1] == 'u' and ACmode[2] == 'y' and ACmode[3] == 'o') {
    dakinir.setMode(DAIKIN_AUTO);
  } else
  if (ACmode[0] == 'D' and ACmode[1] == 'r' and ACmode[2] == 'y') {
    dakinir.setMode(DAIKIN_DRY);
  }

  //Set FAN
  if (ACfan[0] == 'M' and ACfan[1] == 'a' and ACfan[2] == 'x') {
    dakinir.setFan(5);
  } else
  if (ACfan[0] == 'M' and ACfan[1] == 'e' and ACfan[2] == 'd') {
    dakinir.setFan(3);
  } else
  if (ACfan[0] == 'M' and ACfan[1] == 'i' and ACfan[2] == 'n') {
    dakinir.setFan(1);
  } else
  if (ACfan[0] == 'A' and ACfan[1] == 'u' and ACfan[2] == 't') {
    dakinir.setFan(0);
  }
  
  //Set SWING  VERTICAL
  if (ACswing[0] == 'y' and ACswing[1] == 'e' and ACswing[2] == 's') {
    dakinir.setSwingVertical(1);
  } else
  if (ACswing[0] == 'n' and ACswing[1] == 'o') {
    dakinir.setSwingVertical(0);
  }

  //set Temp
  int temptACTemp = (ACtemp[0] - '0') * 10 + (ACtemp[1] - '0');
  Serial.println(temptACTemp);
  dakinir.setTemp(temptACTemp);

  dakinir.send();
}

void keepAlive (char inputString[]) {
//-------------------------------------
//keep-alive interval to update node status
//There is 2 states of interval
//1_active when ping frequently
//2_reconnecting when re-connect
//------------------------------------
  char publishMessage [100] = "";
  snprintf(publishMessage, 100, "{\"NodeMacAddress\":\"%02X:%02X:%02X:%02X:%02X:%02X\",\"State\":\"%s\"}",mac[0],mac[1],mac[2],mac[3],mac[4],mac[5], inputString);
  Serial.print("Message send: ");
  Serial.println(publishMessage);
  char keepAliveTopic [55] = "";
  snprintf(keepAliveTopic, 55,"%s/keepAlive",pubTopicGen);
  client.publish(keepAliveTopic, publishMessage, true);
}
//------------------------------------
void isControlNode(char inputString[]) {
  char breakedValue[150] = "";
  char breakedValue1[150] = "";
  char breakedValue2[150] = "";
  int check = 0;
  int countSpace = 0;
  int i = 0;
  int j = 0;
  int countOfChar = 0;

  //Remove All un-needed # command from received message
  while (inputString[i] != '#') {
                breakedValue[i] = inputString[i];
                breakedValue1[i] = inputString[i];
                breakedValue2[i] = inputString[i];
                i++;
  }
  //call parse data from payload under JSON
  Serial.println(breakedValue);
  readReceivedMsgInJson(breakedValue);
  //Checking the device is AC or WF or else to break other JSON parameters following kinds
  Serial.println(breakedValue1);
  
  if ((DeviceType[0] == 'A') and (DeviceType[1] == 'C')) {
    Serial.println(breakedValue1);
    readReceivedMsgInACJson(breakedValue1);
    if ((ACID[0] == 'D') and (ACID[1] == 'a') and (ACID[2] == 'i') and (ACID[3] == 'k')and (ACID[4] == 'i') and (ACID[5] == 'n')) {
      adjustDaikinAC();
    } 
    else if ((ACID[0] == 'T') and (ACID[1] == 'o') and (ACID[2] == 's') and (ACID[3] == 'h')and (ACID[4] == 'i') and (ACID[5] == 'b')) {
      adjustToshibaAC();
    }
    else if ((ACID[0] == 'L') and (ACID[1] == 'G')) {
      adjustLGAC();
    }
    else if ((ACID[0] == 'M') and (ACID[1] == 'i') and (ACID[2] == 't') and (ACID[3] == 's') and (ACID[4] == 'u')) {
      adjustMitsubishiAC();
    }
    else if ((ACID[0] == 'P') and (ACID[1] == 'a') and (ACID[2] == 'n') and (ACID[3] == 'a')) {
      //adjustMitsubishiAC();
    }
    sendConfirmtoRetained(breakedValue2);
  } else if ((DeviceType[0] == 'W') and (DeviceType[1] == 'F')) {
    Serial.println(breakedValue1);
    readReceivedMsgInWFJson(breakedValue1);
    adjustWF();
    sendConfirmtoRetained(breakedValue2);
  } else {
    isDefinedCommand = false;
  } 
}
//------------------------------------------------------------
//PROCEDURE: VERIFY RECEIVED MESSAGE IS FOR CONTROLLING AC
//------------------------------------------------------------
//PROCESS BY DATE:
// JUL22,2016: CREATED DATE
// 
//------------------------------------------------------------
//-------------------END PROCEDURE------------------------------


//-----void callback description----
//If the client is used to subscribe to topics, a callback function must be provided in the constructor.
//This function is called when new messages arrive at the client.
//topic - the topic the message arrived on (const char[])
//payload - the message payload (byte array)
//length - the length of the message payload (unsigned int)
//-----------------------------------
void callback(char* topic, byte* payload, unsigned int length) {
  for (int i = 0; i < 150; i++) {
    subMsg[i] = '#';
  }
  Serial.print("Message arrived from [");
  Serial.print(mqttServer);
  Serial.print("] ;topic: [");
  Serial.print(topic);
  Serial.print("] with payload: [");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
    subMsg[i] = (char)payload[i];
  }
  Serial.print("]");
  Serial.println();
////--------detect command-----------
  isDefinedCommand = false;
  if (!isDefinedCommand) { isControlNode(subMsg); }
  if (!isDefinedCommand) {
      char temptCommand[200] = "";
      char breakedValue[150] = "";
      int i = 0;
      int countOfChar = 0;
      //Remove All un-needed # command from received message
      while (subMsg[i] != '#') {
                breakedValue[i] = subMsg[i];
                i++;
      }     
      snprintf (temptCommand, 200, "[%s]: NO SYNTAX FOUND", breakedValue);
  }
}
//----------END void callback -----------------------
//callback notifying us of the need to save config
void saveConfigCallback () {
  Serial.println("Should save config");
  shouldSaveConfig = true;
}

/**
  setup_wifi to config wifi after user change setup
  @param: nope
  @return nope
  @version 1.0
  @author Tri Nguyen
  @date June12017
*/
void setup_wifi() {
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Trying connect to ");
  Serial.println(APssid);
  WiFi.mode(WIFI_STA);
  WiFi.begin(APssid, APpassword);
  delay(5000);
  //if you get here you have connected to the WiFi
  if (WiFi.status() == WL_CONNECTED) {
    checkWifi = true;   
    Serial.print("Connected at IP ");
    Serial.println(WiFi.localIP());
    WiFi.macAddress(mac);
    Serial.print("MAC: ");
    Serial.print(mac[0],HEX);
    Serial.print(":");
    Serial.print(mac[1],HEX);
    Serial.print(":");
    Serial.print(mac[2],HEX);
    Serial.print(":");
    Serial.print(mac[3],HEX);
    Serial.print(":");
    Serial.print(mac[4],HEX);
    Serial.print(":");
    Serial.println(mac[5],HEX);   
    snprintf(temptMac, 20, "TMAiOT_%02X%02X%02X%02X%02X%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    Serial.println("Connecting to MQTT");
    Serial.print("MQTT SERVER: "); Serial.print(mqttServer);
    Serial.print(" MQTT PORT: "); Serial.println(mqttPort);
    client.setServer(mqttServer, atoi(mqttPort)); //April23
    client.setCallback(callback); //April23
    snprintf (pubTopicGen, 50, "%s/%s/state", projectName, temptMac);
    snprintf (subTopicGen, 50, "%s/%s/set", projectName, temptMac);
    Serial.print("Default Publish Topic: "); Serial.println(pubTopicGen);
    Serial.print("Default Subscribe Topic: "); Serial.println(subTopicGen);
    reconnect(); //run for 1st register message
    ArduinoOTA.onStart([]() {
      Serial.println("Start");
    });
    ArduinoOTA.onEnd([]() {
      Serial.println("\nEnd");
    });
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    });
    ArduinoOTA.onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });
    ArduinoOTA.begin();
  }
  else {
    Serial.print("Failed connect to AP. Code: ");
    Serial.println(WiFi.status());
    checkWifi = false;
    pixels.setPixelColor(0, pixels.Color(255, 255, 0)); // RED+GREEN = YELLOW = Wifi DISCONNECTING...
    pixels.show();
  }
}

/**
  recoonect to check wifi status and mqtt connection to avoid send message when network is down
  @param: nope
  @return nope

  @version 1.0
  @author Tri Nguyen
  @date June12017

*/
void reconnect() {
  // Loop until we're reconnected
    client.connect(temptMac);
    Serial.print("Attempting MQTT connection...using clientID: ");
    Serial.print(temptMac);
    Serial.print(" Result: ");
    // Attempt to connect
    if (client.connected()) {
      Serial.println("connected");
      pixels.setPixelColor(0, pixels.Color(0, 0, 255)); // BLUE: done setup - GOOD STATE
      pixels.show();
      // Once connected, publish an announcement...
      char registerMessage[100] = "";
      snprintf (registerMessage, 100, "{\"ID\":\"%s\",\"type\":\"%s\",\"project\":\"%s\"}", temptMac,"IRControl",projectName);
      char defaultTopic[50] = "";
      snprintf (defaultTopic, 50, "%s/RegisterDevices", projectName);
      client.publish(defaultTopic, registerMessage, true);
      delay(200);
      // ... and resubscribe
      Serial.print("Sent register message: ");  Serial.print(registerMessage);  Serial.print(" to ");  Serial.println(defaultTopic);
      delay(200);
      client.subscribe(subTopicGen);
      mqttReconnecting = false;
    }
    else {
      pixels.setPixelColor(0, pixels.Color(255, 0, 255)); // PINK: error in connecting MQTT server
      pixels.show();
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 10 seconds");
      mqttReconnecting = true;
    }
  }
  
void setup() {
  //insert your own setup code here
    irsend.begin();
    dakinir.begin();
    Serial.begin(115200);
    pixels.begin(); // This initializes the NeoPixel library.
    pinMode(SOFT_RST_PIN, INPUT);
    IRpin=D7;
    khz=38;
    halfPeriodicTime = 500/khz;
      //clean FS, for testing
//    SPIFFS.format();

//--------Read Congiguration file-----------
  Serial.println("mounting FS...");
  pixels.setPixelColor(0, pixels.Color(255, 0, 0)); // RED: SETUP
  pixels.show();
  if (SPIFFS.begin()) {
    Serial.println("mounted file system");
    if (SPIFFS.exists("/config.json")) {
      //file exists, reading and loading
      Serial.println("reading config file");
      File configFile = SPIFFS.open("/config.json", "r");
      if (configFile) {
        factoryReset = false;
        Serial.println("opened config file");
        size_t size = configFile.size();
        // Allocate a buffer to store contents of the file.
        std::unique_ptr<char[]> buf(new char[size]);
        configFile.readBytes(buf.get(), size);
        DynamicJsonBuffer jsonBuffer;
        JsonObject& json = jsonBuffer.parseObject(buf.get());
        json.printTo(Serial);
        if (json.success()) {
          Serial.println("\nparsed config parameters");
          strcpy(APssid, json["APssid"]);
          strcpy(APpassword, json["APpassword"]);          
          strcpy(mqttServer, json["mqttServer"]);
          strcpy(mqttPort, json["mqttPort"]);
          strcpy(projectName, json["projectName"]);
//          strcpy(subTopicGen, json["subTopicGen"]);
          setup_wifi();
          factoryReset = false;
        } else {
          Serial.println("failed to load json config");
          factoryReset = true;
        }
    } else {
      Serial.println("File Config has not created");
      factoryReset = true;
    }
  }
  else {
    Serial.println("failed to mount FS");
    factoryReset = true;
  }
}
//--------END Read Congiguration file-----------
}

void loop() {
  WiFiManager wifiManager;
  // read the state of the switch into a local variable:
  int reading = digitalRead(SOFT_RST_PIN);
  if (WiFi.status() == WL_CONNECTED) {
    ArduinoOTA.handle();
  }
  // check to see if you just pressed the button
  // (i.e. the input went from LOW to HIGH),  and you've waited
  // long enough since the last press to ignore any noise:
  // If the switch changed, due to noise or pressing:

  if (reading != lastButtonState) {
    // reset the debouncing timer
    lastDebounceTime = millis();
  }

 //--------RESET WIFI CONFIGURATION-------------------------------
  if ((factoryReset == true) or ((millis() - lastDebounceTime) > debounceDelay)) {
    Serial.println("ACCESSING WIFI DIRECT - AP");
    //WiFiManager
    //reset saved settings
    wifiManager.resetSettings();
    delay(2000);
    //sets timeout until configuration portal gets turned off
    //useful to make it all retry or go to sleep
    //in seconds
    pixels.setPixelColor(0, pixels.Color(0, 255, 0)); //GREEN: WIFI CONFIGURE
    pixels.show();
    // The extra parameters to be configured (can be either global or just in the setup)
    // After connecting, parameter.getValue() will get you the configured value
    // id/name placeholder/prompt default length
    WiFiManagerParameter custom_mqttServer("server", "MQTT Server", mqttServer, 40);
    WiFiManagerParameter custom_mqttPort("port", "MQTT Port", mqttPort, 5);
    //    WiFiManagerParameter custom_blynk_token("blynk", "blynk token", blynk_token, 32);
    WiFiManagerParameter AP_password("p", "password of SSID", APpassword, 20);
    WiFiManagerParameter custom_project_name("projectName", "Project Name", projectName , 40);
//    WiFiManagerParameter custom_GeneralPublish_topic("pubTopicGen", "GeneralPublish_topic", pubTopicGen, 40);
    //set config save notify callback
    wifiManager.setSaveConfigCallback(saveConfigCallback);
    //exit after config instead of connecting
    wifiManager.setBreakAfterConfig(true);

    //add all your parameters here
    wifiManager.addParameter(&AP_password);
    wifiManager.addParameter(&custom_mqttServer);
    wifiManager.addParameter(&custom_mqttPort);
//    wifiManager.addParameter(&custom_blynk_token);
    wifiManager.addParameter(&custom_project_name);
//    wifiManager.addParameter(&custom_GeneralPublish_topic);
    
    WiFi.macAddress(mac);
    char temptSSID [20] = "";
    snprintf(temptSSID, 20, "TMAiOT_%02X%02X%02X%02X%02X%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    Serial.println(temptSSID);

    if (!wifiManager.autoConnect(temptSSID, password)) {
      Serial.println("failed to connect existing SSID ...");
    }
    else {
      //read updated parameters
      WiFi.status();
      strcpy(mqttServer, custom_mqttServer.getValue());
      strcpy(mqttPort, custom_mqttPort.getValue());
      //      strcpy(blynk_token, custom_blynk_token.getValue());
      strcpy(APpassword, AP_password.getValue());
      strcpy(projectName, custom_project_name.getValue());
//      strcpy(pubTopicGen, custom_GeneralPublish_topic.getValue());
      //save the custom parameters to FS
      if (shouldSaveConfig) {
        Serial.println("saving config");
        DynamicJsonBuffer jsonBuffer;
        JsonObject& json = jsonBuffer.createObject();
        json["APssid"] = WiFi.SSID();
        json["APpassword"] = APpassword;
        json["mqttServer"] = mqttServer;
        json["mqttPort"] = mqttPort;
        json["projectName"] = projectName;
//        json["pubTopicGen"] = pubTopicGen;
        File configFile = SPIFFS.open("/config.json", "w");
        if (!configFile) {
          Serial.println("failed to open config file for writing");
        }
        json.printTo(Serial);
        json.printTo(configFile);
        configFile.close();
        Serial.println();
        //end save
        pixels.setPixelColor(0, pixels.Color(0, 255, 0)); //BLUE GOOD
        pixels.show();
      }

      ESP.reset();
      delay(2000);
    }
  }
  //-----------END RESET--------------------
  //---START WI4341-----------------------------------------------------
  //Description: Mode Wifi Client : Loop Authen until successful
  if (mqttReconnecting) {
    if ((simulatedDropedWifi) or (WiFi.status() != WL_CONNECTED)) {
      checkWifi = false;
    }
    else {
      checkWifi = true;
    }
  }
  //----STOP WI4341----------------------------------------------------
//Send response after receiving command from broker
    if (checkWifi == true) {
      unsigned long currentMillis3 = millis();
      if (currentMillis3 - previousMillis3 > publishInveral) {
        previousMillis3 = currentMillis3;
        unsigned long currentMillis4 = millis();
        if (client.connected()) {
          char temptCommand[] = "Active";
//          keepAlive(temptCommand);   
      }
      else {
        if ((currentMillis4 - previousMillis4 > reconnectInveral)) {
        previousMillis4 = currentMillis4;
        reconnect();
        }
      }
    }
    client.loop();
  } else {
    setup_wifi();
  }
}

