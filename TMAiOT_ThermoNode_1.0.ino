/**
  TMAiOT SMART OFFICE PROJECT
  Name: TMAiOT_ThermoNode
  Purpose: Tracking Temperature and Humidity and send to MQTT Broker
  Status of LED:
  - Color RED: WiFi Node is starting up, please wait in 10 seconds.
  - Color BLUE: WiFi Node is working as expectation without error.
  - Color GREEN: WiFi Node is in Access Point Mode/Direct-WiFi for Configuration
  - Color PINK: WiFi is in Client Mode, but cannot connect to MQTT Broker Server due to Broker issue.
  - Color YELLOW: WiFi is in Client Mode, but cannot connect to WiFi due to WiFi Access Point issue.
  @author Tri Nguyen nductri@tma.com.vn
  @version 1.0 6/15/17
*/

#include <Adafruit_Sensor.h>
#include <DHT_U.h>                  //https://github.com/adafruit/DHT-sensor-library version 1.0.0
#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager version 0.11.0
#include <ESP8266WiFi.h>          //https://github.com/esp8266/Arduino version 1.0.0
#include <DNSServer.h>            // version 1.1.0
#include <ESP8266WebServer.h>     // version 1.0.0
#include <ArduinoJson.h>          //https://github.com/bblanchon/ArduinoJson version 5.1.0
#include <FS.h>
#include <PubSubClient.h> //April23 version 2.6.0
#include "math.h"
#include <Adafruit_NeoPixel.h>    //https://github.com/adafruit/Adafruit_NeoPixel version 1.1.1

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
bool mqttReconnecting = false;

//------------WIFI CONFIGURATION PARAMETERS--------------------
bool factoryReset = false;
char APssid[20] = ""; //AP is ESP will connect
char APpassword[20] = "";
char password[] = "password";
WiFiClient espClient; //April23
byte mac[6];
char temptMac[20] = "";
//--------------MQTT client PRAMETERS--------------
//define mqtt server default values here, if there are different values in config.json, they are overwritten.
char mqttServer[40] = ""; //April24
char mqttPort[6] = ""; //April24
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
//----------DTH CONFIGURATION---------------
#define DHTPIN D4     // what digital pin we're connected to
#define DHTTYPE DHT22   // DHT 22 (AM2302)
DHT_Unified dht(DHTPIN, DHTTYPE);
char bufTemp[9];
char bufHum[4];
char bufHeatIndex[9];
bool sensorState = false;
//----------ESP API in case using DTH21 CONFIGURATION---------------
bool isDefinedCommand = true;
bool simulatedDropedWifi = false;
////--------END VAR=-----------------------------------------------

//------------------------------------------------

/**
  Read DHT22 sensor

  @param nope
  @return bufHum, bufTemp, bufHeatIndex
  @version 1.0
  @author Tri Nguyen
  @date June12017

*/
void DTHCalculation() {
  sensors_event_t event;
  dht.humidity().getEvent(&event);
  float humidity = event.relative_humidity;
  Serial.println(humidity);
  // Read temperature as Celsius (the default)
  dht.temperature().getEvent(&event);
  float temperature = event.temperature;
  Serial.println(temperature);
  // Check if any reads failed and exit early (to try again).
  if (std::isnan(humidity) || std::isnan(temperature)) {
    sensorState = false;
  }
  else {
    // Compute heat index in Celsius (isFahreheit = false)
    float heatIndex = 0;
    sensorState = true;
    int d1 = humidity;
    snprintf (bufHum, 4, "%d", d1);
    d1 = temperature;
    float f2 = temperature - d1;
    int d2 = (f2 * 10);
    snprintf (bufTemp, 9, "%d.%01d", d1, d2);
    d1 = heatIndex;
    f2 = heatIndex - d1;
    d2 = (f2 * 10);
    snprintf (bufHeatIndex, 9, "%d.%01d", d1, d2);
  }
}

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
  send Thermo Index to update broker

  @param nope
  @return
  //1_Termperature
  //2_Humid
  //3_feelike
  @version 1.0
  @author Tri Nguyen
  @date June12017

*/
void sendThermoIndex () {

  firstTime = false;
  DTHCalculation();
  if (sensorState) {
   char publishMessage [100] = "";
   snprintf(publishMessage, 100, "{\"temperature\":\"%s\",\"humidity\":\"%s\"}", bufTemp, bufHum);
   Serial.print("Message send: ");
   Serial.println(publishMessage);
   client.publish(pubTopicGen, publishMessage, true);
  } else {
    Serial.println("Message DOES NOT SEND DUE TO SENSOR STATE IS FALSE!!!");
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
  char publishMessage [120] = "";
  snprintf(publishMessage, 120, "{\"NodeMacAddress\":\"%02X:%02X:%02X:%02X:%02X:%02X\",\"State\":\"%s\"}", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], inputString);
  Serial.print("Message send: ");
  Serial.println(publishMessage);
  char keepAliveTopic [55] = "";
  snprintf(keepAliveTopic, 55, "%s/keepAlive", pubTopicGen);
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
  for (int i = 0; i < 120; i++) {
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
  ////--------detect command--------- --
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
  }
  //----------END case 1.2 of NORMAL MODE -----------------------
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
    snprintf (pubTopicGen, 50, "SmartOffice/%s/state", temptMac);
    snprintf (subTopicGen, 50, "SmartOffice/%s/set", temptMac);
    Serial.print("Default Publish Topic: "); Serial.println(pubTopicGen);
    Serial.print("Default Subscribe Topic: "); Serial.println(subTopicGen);
    reconnect(); //run for 1st register message
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
      snprintf (registerMessage, 100, "{\"ID\":\"%s\",\"type\":\"%s\",\"project\":\"%s\"}", temptMac,"Thermo",projectName);
      char defaultTopic[50] = "";
      snprintf (defaultTopic, 50, "SmartOffice/RegisterDevices");
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
//-------end Apirl 23 2016------------
void setup() {
  pinMode(SOFT_RST_PIN, INPUT); //SET GPIO 0 IS SOFT RESET PIN AND IS INPUT
  Serial.begin(115200); //start Serial

  dht.begin(); //Start DTH
  sensor_t sensor; //Start DTH

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
//      SPIFFS.format();

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
    if (checkWifi == true) {
      unsigned long currentMillis3 = millis();
      if (currentMillis3 - previousMillis3 > publishInveral) {
        previousMillis3 = currentMillis3;
        unsigned long currentMillis4 = millis();
        if (client.connected()) {
          sendThermoIndex(); 
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
  } else {
    setup_wifi();
  }
}

