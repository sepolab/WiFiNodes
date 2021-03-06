/**
  TMAiOT SMART OFFICE PROJECT
  Name: TMAiOT_RelayNode
  Purpose: control AC Devices via MQTT Broker
  COMMAND = {"Relay":"1/2/3/4","Action":"on/off"}
  ESP PIN LAYOUT
 SOFT RESET BUTTON: GPIO 0 => use to reset WiFi or MQTT server connection
 LED INDICATER: GPIO 1 => use to inform status of Connection
 SWITCH 1 CONTROL PIN: D0
 SWITCH 2 CONTROL PIN: D5
 SWITCH 3 CONTROL PIN: D6
 SWITCH 4 CONTROL PIN: D7
 
  @author Tri Nguyen nductri@tma.com.vn
  @version 1.0 6/15/17
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
bool firstTime1 = true;
bool firstTime2 = true;
bool firstTime3 = true;
bool firstTime4 = true;
//----------ESP API in case using DTH21 CONFIGURATION---------------
bool isDefinedCommand = true;
bool simulatedDropedWifi = false;

char relayID[2] = "";
char relayCommand[5] = "";
//---------CONTROL1 CONFIGURATION---------------------------------------
int pinControl1 = D0;           // Use pinLED of Wifi indication.
int pinControl1State = HIGH;
//---------CONTROL2 CONFIGURATION---------------------------------------
int pinControl2 = D5;  
int pinControl2State = HIGH;
//---------CONTROL3 CONFIGURATION---------------------------------------
int pinControl3 = D6;           // Use pinLED of Wifi indication.
int pinControl3State = HIGH;
//---------CONTROL4 CONFIGURATION---------------------------------------
int pinControl4State = HIGH;
int pinControl4 = D7; 
//--------END VAR=-----------------------------------------------
unsigned long previousMillis5;
bool timeout = false;
bool oneTime = true;
//PROCEDURE: VERIFY RECEIVED MESSAGE IS FOR CONTROLLING NODE
//------------------------------------------------------------
//PROCESS BY DATE:
// MAY08,2017: CREATED DATE
// the general received message will following format:
// {"Relay":"1/2/3/4","Action":"on/off"}
// change pub/sub message to 150 char []
// Optional Parameter for AC: mode, temp, fan, swing
// Optional Parameter for WF: command
//------------------------------------------------------------

  /**
  Read readReceivedMsgInJson
  @param inputString: it is char[] that content all payload of sending message in JSON format
  @return relayID: contain Relay info, relayCommand: contain relay state
  @version 1.0
  @author Tri Nguyen
  @date June12017
*/
bool readReceivedMsgInJson (char inputString[]) {
  DynamicJsonBuffer  jsonBuffer;
  JsonObject& root = jsonBuffer.parseObject(inputString);

  // Test if parsing succeeds.
  if (!root.success()) {
    Serial.println("parseObject() failed");
    return false;
  }
  // Fetch values.
    snprintf(relayID, 2, root["Relay"]);
    snprintf(relayCommand, 5, root["Action"]);

    Serial.println("Parsed Received Message successfully!");
    Serial.println("Variables:");
    Serial.print("Relay ID:"); Serial.print(relayID);
    Serial.print(" ; Received Command:"); Serial.println(relayCommand);
    return true;
}

  /**
  Read isControlNode
  @param inputString: it is char[] that content all payload of received message in JSON format
  @return action to output pins
  @version 1.0
  @author Tri Nguyen
  @date June12017
*/

void isControlNode(char inputString[]) {
  char breakedValue[150] = "";
  char breakedValue1[150] = "";
  int check = 0;
  int countSpace = 0;
  int i = 0;
  int j = 0;
  int countOfChar = 0;
  Serial.println(inputString);
  //Remove All un-needed # command from received message
  while (inputString[i] != '#') {
                breakedValue[i] = inputString[i];
                breakedValue1[i] = inputString[i];
                i++;
  }

  //call parse data from payload under JSON
  if (readReceivedMsgInJson(breakedValue)) {
    if ((!firstTime1) and (!firstTime2) and (!firstTime3) and (!firstTime4))  {
      Serial.println(breakedValue1);
      sendConfirmtoRetained(breakedValue1);    
    }
    if (relayID[0] == '1') {
      if ((relayCommand[0] == 'o') and (relayCommand[1] == 'f') and (relayCommand[2] == 'f')) {
        pinControl1State = HIGH;
        digitalWrite(pinControl1, pinControl1State); //Active = LOW; InActive = HIGH
        isDefinedCommand = true;
        firstTime1 = false;
       }
       else if ((relayCommand[0] == 'o') and (relayCommand[1] == 'n')) {
        pinControl1State = LOW;
        digitalWrite(pinControl1, pinControl1State); //Active = LOW; InActive = HIGH;
        isDefinedCommand = true;
        firstTime1 = false;
       } else {
        isDefinedCommand = false;
       }

    } else
    if (relayID[0] == '2') {
      if ((relayCommand[0] == 'o') and (relayCommand[1] == 'f') and (relayCommand[2] == 'f')) {
        pinControl2State = HIGH;
        digitalWrite(pinControl2, pinControl2State); //Active = LOW; InActive = HIGH        
        isDefinedCommand = true;
        firstTime2 = false;
       }
       else if ((relayCommand[0] == 'o') and (relayCommand[1] == 'n')) {
        pinControl2State = LOW;
        digitalWrite(pinControl2, pinControl2State); //Active = LOW; InActive = HIGH
        isDefinedCommand = true;
        firstTime2 = false;
        } else {
          isDefinedCommand = false;
       }
    } else
    if (relayID[0] == '3') {
      if ((relayCommand[0] == 'o') and (relayCommand[1] == 'f') and (relayCommand[2] == 'f')) {
        pinControl3State = HIGH;
        digitalWrite(pinControl3, pinControl3State); //Active = LOW; InActive = HIGH
        isDefinedCommand = true;
        firstTime3 = false;
       }
       else if ((relayCommand[0] == 'o') and (relayCommand[1] == 'n')) {
        pinControl3State = LOW;
        digitalWrite(pinControl3, pinControl3State); //Active = LOW; InActive = HIGH
        isDefinedCommand = true;
        firstTime3 = false;
       }
       else {
          isDefinedCommand = false;
       }
    } else
    if (relayID[0] == '4') {
      if ((relayCommand[0] == 'o') and (relayCommand[1] == 'f') and (relayCommand[2] == 'f')) {
        pinControl4State = HIGH;
        digitalWrite(pinControl4, pinControl4State); //Active = LOW; InActive = HIGH
        isDefinedCommand = true;
        firstTime4 = false;
       }
       else if ((relayCommand[0] == 'o') and (relayCommand[1] == 'n')) {
        pinControl4State = LOW;
        digitalWrite(pinControl4, pinControl4State); //Active = LOW; InActive = HIGH
        isDefinedCommand = true;
        firstTime4 = false;
       } else {
          isDefinedCommand = false;
       }
    }    
    else {
      isDefinedCommand = false;
    }
  } else {
    isDefinedCommand = false;
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
  Serial.print("Message send: ");
  Serial.println(inputString);
  char newPubTopicGen [50] = "";
  snprintf(newPubTopicGen, 50,"%s/%s",pubTopicGen,relayID);
  Serial.print("Send confirmation to TOPIC: ");Serial.println(newPubTopicGen);
  client.publish(newPubTopicGen, inputString, true);
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
updateCurrentStatus is used to tracking old statement of Relay status after this Node is disconnected.
The mechanism is subscribed State Topics of all 4 Relays consequently

  @param char [] payload
  @return nope
  @version 1.0
  @author Tri Nguyen
  @date June12017
*/
void updateCurrentStatus () {
  Serial.println("START updateCurrentStatus ");
  char newPubTopicGen1 [45] = "";
  snprintf(newPubTopicGen1, 45,"%s/1",pubTopicGen);
  char newPubTopicGen2 [45] = "";
  snprintf(newPubTopicGen2, 45,"%s/2",pubTopicGen);
  char newPubTopicGen3 [45] = "";
  snprintf(newPubTopicGen3, 45,"%s/3",pubTopicGen);
  char newPubTopicGen4 [45] = "";
  snprintf(newPubTopicGen4, 45,"%s/4",pubTopicGen);
  oneTime = false;
  if (firstTime1) {    
    client.subscribe(newPubTopicGen1);
  } else {
    client.unsubscribe(newPubTopicGen1);
  }
  if (firstTime2) {
    client.subscribe(newPubTopicGen2);
  } else {
    client.unsubscribe(newPubTopicGen2);
  }
  if (firstTime3) {  
    client.subscribe(newPubTopicGen3);
  } else {
    client.unsubscribe(newPubTopicGen3);
  }
  if (firstTime4) {   
    client.subscribe(newPubTopicGen4);
  } else {
    client.unsubscribe(newPubTopicGen4);
  }
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
      keepAlive(temptCommand);
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
    snprintf (pubTopicGen, 50, "%s/%s/state", projectName, temptMac);
    snprintf (subTopicGen, 50, "%s/%s/set", projectName, temptMac);
    Serial.print("Default Publish Topic: "); Serial.println(pubTopicGen);
    Serial.print("Default Subscribe Topic: "); Serial.println(subTopicGen);
    reconnect(); //run for 1st register message
//    updateCurrentStatus();
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
      snprintf (registerMessage, 100, "{\"ID\":\"%s\",\"type\":\"%s\",\"project\":\"%s\"}", temptMac,"LightControl",projectName);
      char defaultTopic[50] = "";
      snprintf (defaultTopic, 50, "%s/RegisterDevices", projectName);
      client.publish(defaultTopic, registerMessage, true);
      // ... and resubscribe
      Serial.print("Sent register message: ");  Serial.print(registerMessage);  Serial.print(" to ");  Serial.println(defaultTopic);
      previousMillis5 = millis();
      mqttReconnecting = false;
    }
    else {
      pixels.setPixelColor(0, pixels.Color(255, 0, 255)); // PINK: error in connecting MQTT server
      pixels.show();
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 10 seconds");
      mqttReconnecting = true;
      firstTime1 = true;
      firstTime2 = true;
      firstTime3 = true;
      firstTime4 = true;
      timeout = false;
      oneTime = true;
      client.unsubscribe(subTopicGen);
    }
  }

//-------end Apirl 23 2016------------

void setup() {
  // set the digital pin as output:
  //SET GPIO 0 IS SOFT RESET PIN AND IS INPUT
  pinMode(SOFT_RST_PIN, INPUT);
  // put your setup code here, to run once:
  
  pixels.begin(); // This initializes the NeoPixel library.
  Serial.begin(115200);
  //WiFiManager
  //Local intialization. Once its business is done, there is no need to keep it around
  //    WiFiManager wifiManager;

  pinMode(pinControl1, OUTPUT);  
  digitalWrite(pinControl1, pinControl1State);
  pinMode(pinControl2, OUTPUT);  
  digitalWrite(pinControl2, pinControl2State);
  pinMode(pinControl3, OUTPUT);  
  digitalWrite(pinControl3, pinControl3State);
  pinMode(pinControl4, OUTPUT);  
  digitalWrite(pinControl4, pinControl4State);
  
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

    unsigned long currentMillis5 = millis();
    if (client.connected() and !timeout and (currentMillis5 - previousMillis5 > 5000)) {
        firstTime1 = false;
        firstTime2 = false;
        firstTime3 = false;
        firstTime4 = false;
        Serial.print("Timeout update last states, so Subcribe Topic: "); 
        Serial.println(subTopicGen);
        client.subscribe(subTopicGen);
        updateCurrentStatus();
        timeout = true;
      }

    if (client.connected() and firstTime1 and firstTime2 and firstTime3 and firstTime4 and oneTime) {
      Serial.print("First Time Connecting to MQTT, update status: "); 
      updateCurrentStatus();
    }
    
    if (checkWifi == true) {
      unsigned long currentMillis3 = millis();
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
  } else {
    setup_wifi();
  }
}

