/**
  TMAiOT SMART OFFICE PROJECT
  Name: TMAiOT_PowerMeasureNode
  Purpose: Tracking AC Current and Energy and send to MQTT Broker
  Status of LED:
  - Color RED: WiFi Node is starting up, please wait in 10 seconds.
  - Color BLUE: WiFi Node is working as expectation without error.
  - Color GREEN: WiFi Node is in Access Point Mode/Direct-WiFi for Configuration
  - Color PINK: WiFi is in Client Mode, but cannot connect to MQTT Broker Server due to Broker issue.
  - Color YELLOW: WiFi is in Client Mode, but cannot connect to WiFi due to WiFi Access Point issue.
  
  @author Tri Nguyen nductri@tma.com.vn
  @version 1.0 6/15/17
*/

#include "EmonLib.h" 
#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager
#include <ESP8266WiFi.h>          //https://github.com/esp8266/Arduino
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <ArduinoJson.h>          //https://github.com/bblanchon/ArduinoJson
#include <FS.h>
#include <PubSubClient.h> //April23
#include "math.h"
#include <Adafruit_NeoPixel.h>    //https://github.com/adafruit/Adafruit_NeoPixel
//-------------------------------

//---------SOFT RESET BUTTON PARAMETERS---------------------
const int SOFT_RST_PIN =  D3;      // SOFT RESET button is map to port 0
int buttonState;             // the current reading from the input pin
int lastButtonState = LOW;   // the previous reading from the input pin
// the following variables are long's because the time, measured in miliseconds,
// will quickly become a bigger number than can be stored in an int.
long lastDebounceTime = 0;  // the last time the output pin was toggled
long debounceDelay = 1000;    // the debounce time; increase if the output flickers

//--------LED INDICATION PARAMETERS--------------------
#define PINLED D2

// When we setup the NeoPixel library, we tell it how many pixels, and which pin to use to send signals.
// Note that for older NeoPixel strips you might need to change the third parameter--see the strandtest
// example for more information on possible values.
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(1, PINLED, NEO_GRB + NEO_KHZ800);

//----------------WIFI CHECK STATE PAMAMETERS----------------
int check = 0;
bool checkWifi = false; //wifi connect indication 0: disconnect; 1: connected
bool wifiReconnecting = false;

//------------WIFI CONFIGURATION PARAMETERS--------------------
bool factoryReset = false;
char APssid[20] = "UC_VP"; //AP is ESP will connect
char APpassword[20] = "RAPtor1234!";
char ssid[] = "PowerMeasureNode"; // ESP in AP mode
char password[] = "password";
WiFiClient espClient; //April23
byte mac[6];
char temptMac[20] = "";
//--------------MQTT client PRAMETERS--------------
//define mqtt server default values here, if there are different values in config.json, they are overwritten.
char mqtt_server[40] = "192.168.0.130"; //April24
char mqtt_port[6] = "1883"; //April24
char blynk_token[34] = "YOUR_BLYNK_TOKEN";
//flag for saving data
bool shouldSaveConfig = false;
PubSubClient client(espClient);//April23
unsigned long previousMillis3 = 0; //set time for interval publish
unsigned long previousMillis4 = 0; //set time for interval publish 
//char mqttClientID[] = ssid;
char pubTopicGen[40] = "SO/R21/powerNode1/state";
char subTopicName[40] = "SO/R21/powerNode1/set";
int valueOfSensor = 0;// value that will be published
char pubMsg[150] = "Hello Server,it is client's 1st message!";//payload of publishing message
char subMsg[150]; //payload of subcribled message
long publishInveral = 15000;
bool mqttConnected = false;
const long reconnectInveral = 10000;
bool firstTime = true;
//----------EnergyMonitor CONFIGURATION---------------
EnergyMonitor emon1;  
unsigned long previousMillis6 = 0;
unsigned long previousMillis7 = 0;
double Arms;
unsigned long runnningDuration = 0;
//----------ESP API in case using DTH21 CONFIGURATION---------------
bool isDefinedCommand = true;
bool simulatedDropedWifi = false;
////--------END VAR=-----------------------------------------------

//------------------------------------------------
//void API for cloud command


/**
  Send Confirmation to broker as same as payloaf for retained

  @param char[] payload
  @return nope
  @version 1.0
  @author Tri Nguyen
  @date June12017

*/
void sendConfirmtoRetained (char inputString[]) {
  firstTime = false;
  Serial.print("Message send: ");
  Serial.println(inputString);
  client.publish(pubTopicGen, inputString, true);
}

/**
sendPowerMeasureIndex to update node status
There is 2 states of interval
  @param nope
  @return
//1_Power Measure
//2_AC Current
//3_Voltage
  @version 1.0
  @author Tri Nguyen
  @date June12017
*/
void sendPowerMeasureIndex (int intervalSample) {
  double Irms = emon1.calcIrms(1480);  // Calculate Irms only
  double Prms = Irms* 0.23;
  double Ainstant = 0.0;
  double ArmsDisplay;
  char str_Irms[6];
  char str_Prms[9];
  char str_Arms[9];
    
  Serial.print("Prms = ");
  Serial.print(Prms);         // Apparent power
  Serial.print("kW"); 
  
  Serial.print("; Irms = ");
  Serial.print(Irms);
  Serial.print("A");

  Ainstant = Prms * intervalSample / 3600.0;
  Arms  = Arms + Ainstant;
  Serial.print("; Energy = ");
  
  ArmsDisplay = Arms;
  Serial.print(ArmsDisplay);
  Serial.print("kWh");          // Energy
  int d1 = ArmsDisplay;
  float f2 = ArmsDisplay - d1;
  int d2 = (f2 * 100);
  snprintf (str_Arms, 9,"%d.%02d", d1, d2);
  
  Serial.print(" after ");
  runnningDuration = round (millis() / 1000);
  Serial.print(runnningDuration);
  Serial.print("s");
  
  Serial.println ();

  d1 = Irms;
  f2 = Irms - d1;
  d2 = (f2 * 100);
  snprintf (str_Irms, 6,"%d.%02d", d1, d2);

  d1 = Prms;
  f2 = Prms - d1;
  d2 = (f2 * 100);
  snprintf (str_Prms, 9,"%d.%02d", d1, d2);

  char publishMessage [150] = "";
  snprintf(publishMessage, 150, "{\"Prms\":\"%s\",\"Pde\":\"kW\",\"Irms\":\"%s\",\"Ide\":\"A\",\"Vrms\":\"230\",\"Vde\":\"V\"}", str_Prms, str_Irms);
  Serial.print("Message send: ");
  Serial.println(publishMessage);
  client.publish(pubTopicGen, publishMessage, true);
  
  unsigned long currentMillis7 = millis();
   if (currentMillis7 - previousMillis7 > 60000) {
        previousMillis7 = currentMillis7;
        unsigned long currentMillis7 = millis();
        char publishMessage1 [120] = "";
        snprintf(publishMessage1, 120, "{\"Energy\":\"%s\",\"Ede\":\"kWh\",\"Duration\":\"%i\",\"time\":\"seconds\"}", str_Arms,runnningDuration);
        Serial.print("Message send: ");
        Serial.println(publishMessage1);
        client.publish(pubTopicGen, publishMessage1, true);
  }
}
/**
  keep-alive interval to update node status. There is 2 states of interval
  1_active when ping frequently
  2_reconnecting when re-connect
  @param char [] payload
  @return nope

  @version 1.0
  @author Tri Nguyen
  @date June12017

*/
void keepAlive (char inputString[]) {
  char publishMessage [100] = "";
  snprintf(publishMessage, 100, "{\"NodeMacAddress\":\"%02X:%02X:%02X:%02X:%02X:%02X\",\"State\":\"%s\"}",mac[0],mac[1],mac[2],mac[3],mac[4],mac[5], inputString);
  Serial.print("Message send: ");
  Serial.println(publishMessage);
  char keepAliveTopic [55] = "";
  snprintf(keepAliveTopic, 55,"%s/keepAlive",pubTopicGen);
  client.publish(keepAliveTopic, publishMessage, true);
}
/**
  callback use to read payload of subcribe topic
  If the client is used to subscribe to topics, a callback function must be provided in the constructor.
  This function is called when new messages arrive at the client.

  @param:
    topic - the topic the message arrived on (const char[])
    payload - the message payload (byte array)
    length - the length of the message payload (unsigned int)
  @return nope

  @version 1.0
  @author Tri Nguyen
  @date June12017

*/
void callback(char* topic, byte* payload, unsigned int length) {
  for (int i = 0; i < 30; i++) {
    subMsg[i] = '#';
  }
  Serial.print("Message arrived from [");
  Serial.print(mqtt_server);
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
  if (!isDefinedCommand) {  }
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
      keepAlive(temptCommand);
  }
}
/**
  callback notifying us of the need to save config
  @param: nope
  @return nope

  @version 1.0
  @author Tri Nguyen
  @date June12017

*/
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
  pixels.setPixelColor(0, pixels.Color(255, 0, 0)); //RED : setup
  pixels.show();
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(APssid);
  WiFi.begin(APssid, APpassword);
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
  if (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect(temptMac)) {
      Serial.println("connected");
      pixels.setPixelColor(0, pixels.Color(0, 0, 255)); // BLUE: done setup - GOOD STATE
      pixels.show();
      // Once connected, publish an announcement...
      snprintf (pubMsg, 70, "ID: %s; state: Reconnecting...", ssid);
      client.publish(pubTopicGen, pubMsg,true);
      // ... and resubscribe
      client.subscribe(subTopicName);
      wifiReconnecting = false;
    } else {
      pixels.setPixelColor(0, pixels.Color(255, 0, 255)); // PINK: error in connecting MQTT server
      pixels.show();
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 10 seconds");
      wifiReconnecting = true;
    }
  }
}


void setup() {
  // set the digital pin as output:
  //SET GPIO 0 IS SOFT RESET PIN AND IS INPUT
  pinMode(SOFT_RST_PIN, INPUT);
  // put your setup code here, to run once:
  //
  Serial.begin(115200);
  emon1.current(A0, 16);             // Current: input pin, calibration.
  //calibration is explained bellow
//  if R burden = 22 for Irms max = 97 A => calibration = 91
//  if R burden = 120 for Irms max = 19.5 A => calibration = 17
//  if R burden = 470 for Irms max = 4.9 A => calibration = 5

  pixels.begin(); // This initializes the NeoPixel library.
  //WiFiManager
  //Local intialization. Once its business is done, there is no need to keep it around
  //    WiFiManager wifiManager;
  
  //  //exit after config instead of connecting
  //  wifiManager.setBreakAfterConfig(true);

  //reset saved settings for testing only
  //    wifiManager.resetSettings();
  //    delay(1500);

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
          strcpy(mqtt_server, json["mqtt_server"]);
          strcpy(mqtt_port, json["mqtt_port"]);
          strcpy(pubTopicGen, json["pubTopicGen"]);
          strcpy(subTopicName, json["subTopicName"]);
        } else {
          Serial.println("failed to load json config");
        }
      }
    } else {
      Serial.println("File Config has not created");
      factoryReset = true;
    }
  }
  else {
    Serial.println("failed to mount FS");
  }
  //--------END Read Congiguration file-----------


  //  check wifi status is CONNECTED = 3; other values means disconnected to AP.
  setup_wifi();
  delay(5000);
  //if you get here you have connected to the WiFi
  if (WiFi.status() == WL_CONNECTED) {
    checkWifi = true;
    Serial.println(mqtt_server);
    client.setServer(mqtt_server, atoi(mqtt_port)); //April23
    client.setCallback(callback); //April23
    Serial.println("DONE Setup mqtt sever");
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
    snprintf(temptMac, 20, "%02X%02X%02X%02X%02X%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  }
  reconnect();
}


//-------Apirl 23 2016------------

void loop() {
  WiFiManager wifiManager;
  // read the state of the switch into a local variable:
  int reading = digitalRead(SOFT_RST_PIN);

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
    WiFiManagerParameter custom_mqtt_server("server", "mqtt server", mqtt_server, 40);
    WiFiManagerParameter custom_mqtt_port("port", "mqtt port", mqtt_port, 5);
//    WiFiManagerParameter custom_blynk_token("blynk", "blynk token", blynk_token, 32);
    WiFiManagerParameter AP_password("p", "abcd1234", password, 20);
    WiFiManagerParameter custom_subscribe_topic("subscribe", "subscribe_topic",subTopicName , 40);
    WiFiManagerParameter custom_GeneralPublish_topic("pubTopicGen", "GeneralPublish_topic", pubTopicGen, 40);
    //set config save notify callback
    wifiManager.setSaveConfigCallback(saveConfigCallback);
    //exit after config instead of connecting
    wifiManager.setBreakAfterConfig(true);
    
    //add all your parameters here
    wifiManager.addParameter(&AP_password);
    wifiManager.addParameter(&custom_mqtt_server);
    wifiManager.addParameter(&custom_mqtt_port);
//    wifiManager.addParameter(&custom_blynk_token);
    wifiManager.addParameter(&custom_subscribe_topic);
    wifiManager.addParameter(&custom_GeneralPublish_topic);


    if (!wifiManager.autoConnect(ssid, password)) {
      Serial.println("failed to connect existing SSID ...");
    }
    else {
      //read updated parameters
      WiFi.status();
      strcpy(mqtt_server, custom_mqtt_server.getValue());
      strcpy(mqtt_port, custom_mqtt_port.getValue());
//      strcpy(blynk_token, custom_blynk_token.getValue());
      strcpy(APpassword, AP_password.getValue());
      strcpy(subTopicName, custom_subscribe_topic.getValue());
      strcpy(pubTopicGen, custom_GeneralPublish_topic.getValue());
      //save the custom parameters to FS
      if (shouldSaveConfig) {
        Serial.println("saving config");
        DynamicJsonBuffer jsonBuffer;
        JsonObject& json = jsonBuffer.createObject();
        json["APssid"] = WiFi.SSID();
        json["APpassword"] = APpassword;
        json["mqtt_server"] = mqtt_server;
        json["mqtt_port"] = mqtt_port;
        json["subTopicName"] = subTopicName;
        json["pubTopicGen"] = pubTopicGen;
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
      delay(1000);
      ESP.reset();
      delay(5000);
    }
  }
  //-----------END RESET--------------------

  //---START WI4341-----------------------------------------------------
  //Description: Mode Wifi Client : Loop Authen until successful
  if (wifiReconnecting == true) {
    //      if (!wifiManager.autoConnect(SSIDNAME)) {
    if ((simulatedDropedWifi) or (WiFi.status() != WL_CONNECTED)) {
      checkWifi = false;
      pixels.setPixelColor(0, pixels.Color(255, 255, 0)); // RED+GREEN = YELLOW = Wifi DISCONNECTING...
      pixels.show();
    }
    else {
      Serial.print("connected at IP: ");
      Serial.println(WiFi.localIP());
      if (checkWifi == false) {
        WiFiClient espClient;
        PubSubClient client(espClient);
        client.setServer(mqtt_server, atoi(mqtt_port)); //April23
        client.setCallback(callback); //April23
      }
      checkWifi = true;
    }
  }
  //----STOP WI4341----------------------------------------------------
    if (checkWifi == true) {
      unsigned long currentMillis3 = millis();
      unsigned long currentMillis6 = millis();
      
      if (currentMillis6 - previousMillis6 > 2000) {
        previousMillis6 = currentMillis6;
        unsigned long currentMillis6 = millis();
        if (client.connected()) {
          sendPowerMeasureIndex(2);
          
        }
      }
      if (currentMillis3 - previousMillis3 > publishInveral) {
        previousMillis3 = currentMillis3;
        unsigned long currentMillis4 = millis();
        if (client.connected()) {
          char temptCommand[] = "Active";
          keepAlive(temptCommand);
      }
      else {
        if ((currentMillis4 - previousMillis4 > reconnectInveral)) {
        previousMillis4 = currentMillis4;
        reconnect();
        }
      }
    }
    client.loop();
  }
}
