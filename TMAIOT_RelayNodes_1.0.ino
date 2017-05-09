#include <WiFiManager.h>
#include <ESP8266WiFi.h>          //https://github.com/esp8266/Arduino
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <ArduinoJson.h>          //https://github.com/bblanchon/ArduinoJson
#include <FS.h>
#include <PubSubClient.h> //April23
#include "math.h"
//-------------------------------
//ESP PIN LAYOUT
// SOFT RESET BUTTON: GPIO 0 => use to reset WiFi or MQTT server connection
// LED INDICATER: GPIO 1 => use to inform status of Connection
// SWITCH 1 CONTROL PIN: GPIO 14
// SWITCH 2 CONTROL PIN: GPIO 12
// SWITCH 3 CONTROL PIN: GPIO 13
// SWITCH 4 CONTROL PIN: GPIO 15
// Current sensor read I2C
// PUBLISH to Trihome/indoor/bedroom1/sub
//  COMMAND = {"Relay":"1/2/3/4","Action":"on/off"}
//---------SOFT RESET BUTTON PARAMETERS---------------------
const int SOFT_RST_PIN =  0;      // SOFT RESET button is map to port 0
int buttonState;             // the current reading from the input pin
int lastButtonState = LOW;   // the previous reading from the input pin
// the following variables are long's because the time, measured in miliseconds,
// will quickly become a bigger number than can be stored in an int.
long lastDebounceTime = 0;  // the last time the output pin was toggled
long debounceDelay = 1000;    // the debounce time; increase if the output flickers

//--------LED INDICATION PARAMETERS--------------------
const int ledPin =  4; // the number of the LED pin
// Variables will change :
int ledState = HIGH;             // ledState used to set the LED
int configureState = 1;   //It is flag to recognize when is in configuration
//
//unsigned long previousMillis = 0;        // will store last time LED was updated
//unsigned long previousMillis1 = 0;       // will store last time LED was updated
//unsigned long previousMillis2 = 0;
//unsigned long previousMillis5 = 0;

// constants won't change :
const long interval1000 = 1000;           // interval at which to blink (milliseconds)
const long interval50 = 50;           // interval at which to blink (milliseconds)
const long interval100 = 100;           // interval at which to blink (milliseconds)
const long interval10000 = 10000;           // interval at which to blink (milliseconds)
const long interval300 = 300;           // interval at which to blink (milliseconds)
const long interval5000 = 5000;           // interval at which to blink (milliseconds)

//----------------WIFI CHECK STATE PAMAMETERS----------------
int check = 0;
bool checkWifi = false; //wifi connect indication 0: disconnect; 1: connected
bool wifiReconnecting = false;

//------------WIFI CONFIGURATION PARAMETERS--------------------
bool factoryReset = false;
char APssid[20] = "SePoLab"; //AP is ESP will connect
char APpassword[20] = "tn17691510 ";
const char ssid[] = "RelayNode"; // ESP in AP mode
const char password[] = "password";
WiFiClient espClient; //April23
byte mac[6];
//--------------MQTT client PRAMETERS--------------
//define mqtt server default values here, if there are different values in config.json, they are overwritten.
char mqtt_server[40] = "192.168.1.30"; //April24
char mqtt_port[6] = "1883"; //April24
char blynk_token[34] = "YOUR_BLYNK_TOKEN";
//flag for saving data
bool shouldSaveConfig = false;
PubSubClient client(espClient);//April23
unsigned long previousMillis3 = 0; //set time for interval publish
unsigned long previousMillis4 = 0; //set time for interval publish 
//char mqttClientID[] = ssid;
char pubTopicGen[40] = "Trihome/testbed/state";
char subTopicName[40] = "Trihome/testbed/set";
int valueOfSensor = 0;// value that will be published
char pubMsg[150] = "Hello Server,it is client's 1st message!";//payload of publishing message
char defaultMsg[] = "Hello its default message payload";//payload of publishing message
char subMsg[150]; //payload of subcribled message
long publishInveral = 15000;
bool mqttConnected = false;
const long reconnectInveral = 10000;
bool firstTime = true;
bool isDefinedCommand = true;
bool simulatedDropedWifi = false;

char relayID[2] = "";
char relayCommand[5] = "";
//---------CONTROL1 CONFIGURATION---------------------------------------
int pinControl1 = 14;           // Use pinLED of Wifi indication.
int pinControl1State = HIGH;
bool sendOnetime1 = false;
//---------CONTROL2 CONFIGURATION---------------------------------------
int pinControl2 = 12;  
int pinControl2State = HIGH;
bool sendOnetime2 = false;
//---------CONTROL3 CONFIGURATION---------------------------------------
int pinControl3 = 13;           // Use pinLED of Wifi indication.
int pinControl3State = HIGH;
bool sendOnetime3 = false;
//---------CONTROL4 CONFIGURATION---------------------------------------
int pinControl4 = 15;  
int pinControl4State = HIGH;
bool sendOnetime4 = false;
//--------END VAR=-----------------------------------------------

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

void isControlNode(char inputString[]) {
  char breakedValue[150];
  char breakedValue1[150];
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
    if (relayID[0] == '1') {
      if ((relayCommand[0] == 'o') and (relayCommand[1] == 'f') and (relayCommand[2] == 'f')) {
        pinControl1State = HIGH;
        digitalWrite(pinControl1, pinControl1State); //Active = LOW; InActive = HIGH
       }
       else if ((relayCommand[0] == 'o') and (relayCommand[1] == 'n')) {
        pinControl1State = LOW;
        digitalWrite(pinControl1, pinControl1State); //Active = LOW; InActive = HIGH;
       }
       sendOnetime1 = true;
       isDefinedCommand = true;
    } else
    if (relayID[0] == '2') {
      if ((relayCommand[0] == 'o') and (relayCommand[1] == 'f') and (relayCommand[2] == 'f')) {
        pinControl2State = HIGH;
        digitalWrite(pinControl2, pinControl2State); //Active = LOW; InActive = HIGH        
       }
       else if ((relayCommand[0] == 'o') and (relayCommand[1] == 'n')) {
        pinControl2State = LOW;
        digitalWrite(pinControl2, pinControl2State); //Active = LOW; InActive = HIGH
       }
       sendOnetime2 = true;
       isDefinedCommand = true;
    } else
    if (relayID[0] == '3') {
      if ((relayCommand[0] == 'o') and (relayCommand[1] == 'f') and (relayCommand[2] == 'f')) {
        pinControl3State = HIGH;
        digitalWrite(pinControl3, pinControl3State); //Active = LOW; InActive = HIGH
       }
       else if ((relayCommand[0] == 'o') and (relayCommand[1] == 'n')) {
        pinControl3State = LOW;
        digitalWrite(pinControl3, pinControl3State); //Active = LOW; InActive = HIGH
       }
       sendOnetime3 = true;
       isDefinedCommand = true;
    } else
    if (relayID[0] == '4') {
      if ((relayCommand[0] == 'o') and (relayCommand[1] == 'f') and (relayCommand[2] == 'f')) {
        pinControl4State = HIGH;
        digitalWrite(pinControl4, pinControl4State); //Active = LOW; InActive = HIGH
       }
       else if ((relayCommand[0] == 'o') and (relayCommand[1] == 'n')) {
        pinControl4State = LOW;
        digitalWrite(pinControl4, pinControl4State); //Active = LOW; InActive = HIGH
       }
       sendOnetime4 = true;
       isDefinedCommand = true;
    }    
    else {
      isDefinedCommand = false;
    }
    sendConfirm();    
  } else {
    isDefinedCommand = false;
  }
}
//---------------------

//---------------
//SendConfirm () will send feedback to MQTT to confirm the command 
//---------------
void sendConfirm () {
          firstTime = false;
        //-----SWITCH 1 CONFIRMATION----BEGIN-----{"Relay":"1/2/3/4","Action":"on/off"}
        if (pinControl1State == LOW) {
            if (sendOnetime1) {
              client.publish(pubTopicGen, "{\"Relay\":\"1\",\"Action\":\"on\"}", true);
              delay(50);
              sendOnetime1 = false;
            }
        } else if (pinControl1State == HIGH) {
            if (sendOnetime1) {
              client.publish(pubTopicGen, "{\"Relay\":\"1\",\"Action\":\"off\"}", true);
              delay(50);  
              sendOnetime1 = false;
            }
        }
        //-----SWITCH 1 CONFIRMATION----END-----
        //-----SWITCH 2 CONFIRMATION----BEGIN-----{"Relay":"1/2/3/4","Action":"on/off"}
        if (pinControl2State == LOW) {
            if (sendOnetime2) {
              client.publish(pubTopicGen, "{\"Relay\":\"2\",\"Action\":\"on\"}",true);
              delay(50);
              sendOnetime2 = false;
            }
        } else if (pinControl2State == HIGH) {
            if (sendOnetime2) {
              client.publish(pubTopicGen, "{\"Relay\":\"2\",\"Action\":\"off\"}",true);
              delay(50);
              sendOnetime2 = false;
            }
        }
        //-----SWITCH 2 CONFIRMATION----END-----
        //-----SWITCH 3 CONFIRMATION----BEGIN-----{"Relay":"1/2/3/4","Action":"on/off"}
        if (pinControl3State == LOW) {
            if (sendOnetime3) {
              client.publish(pubTopicGen, "{\"Relay\":\"3\",\"Action\":\"on\"}",true);
              delay(50);
              sendOnetime3 = false;
            }
        } else if (pinControl3State == HIGH) {
            if (sendOnetime3) {
              client.publish(pubTopicGen, "{\"Relay\":\"3\",\"Action\":\"off\"}",true);
              delay(50);
              sendOnetime3 = false;
            }
        }
        //-----SWITCH 3 CONFIRMATION----END-----
        //-----SWITCH 4 CONFIRMATION----BEGIN-----{"Relay":"1/2/3/4","Action":"on/off"}
        if (pinControl4State == LOW) {
            if (sendOnetime4) {
              client.publish(pubTopicGen, "{\"Relay\":\"4\",\"Action\":\"on\"}",true);
              delay(50);
              sendOnetime4 = false;
            }
        } else if (pinControl4State == HIGH) {
            if (sendOnetime4) {
              client.publish(pubTopicGen, "{\"Relay\":\"4\",\"Action\":\"off\"}",true);
              delay(50);
              sendOnetime4 = false;
            }
        }
        //-----SWITCH 4 CONFIRMATION----END-----
}

//-------------------------------------
//keep-alive interval to update node status
//There is 2 states of interval
//1_active when ping frequently
//2_reconnecting when re-connect
//------------------------------------
void keepAlive (char inputString[]) {
  char breakedValue[150];
  char breakedValue1[150];
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
  if (client.connected()) {
      snprintf(pubMsg, 100, "{\"NodeMacAddress\":\"%02X:%02X:%02X:%02X:%02X:%02X\",\"State\":\"%s\"}",mac[0],mac[1],mac[2],mac[3],mac[4],mac[5], breakedValue);
      Serial.print("Message send: ");
      Serial.println(pubMsg);
      client.publish(pubTopicGen, pubMsg, true);
  } 
}

//-------- END case 1.2 if NORMAL MODE-------------------------
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
  if (!isDefinedCommand) { isControlNode(subMsg); }
  if (!isDefinedCommand) {
//      snprintf (pubMsg, 30, "COMMAND NOT FOUND"); //strtok(inputString,"#"));
//      Serial.print("Message send: ");
//      Serial.println(pubMsg);
//      client.publish(pubTopicGen, pubMsg,true);
    char temptCommand[] = "COMMAND NOT FOUND";
    keepAlive(temptCommand);
  }
//----------END case 1.2 of NORMAL MODE -----------------------
}

//callback notifying us of the need to save config
void saveConfigCallback () {
  Serial.println("Should save config");
  shouldSaveConfig = true;
}

void setup_wifi() {
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(APssid);
  WiFi.begin(APssid, APpassword);
}

void reconnect() {
  // Loop until we're reconnected
  //  while (!client.connected()) {
  if (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect(ssid)) {
      Serial.println("connected");
      char temptCommand[] = "Reconnecting";
      keepAlive(temptCommand);
      // Once connected, publish an announcement...
      // ... and resubscribe
      client.subscribe(subTopicName);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 10 seconds");
    }
  }
}

//-------end Apirl 23 2016------------

void setup() {
  // set the digital pin as output:
  //SET GPIO 0 IS SOFT RESET PIN AND IS INPUT
  pinMode(SOFT_RST_PIN, INPUT);
  // put your setup code here, to run once:
  //
  Serial.begin(115200);
  //WiFiManager
  //Local intialization. Once its business is done, there is no need to keep it around
  //    WiFiManager wifiManager;
  ledState = LOW;
//  digitalWrite(ledPin, ledState);
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
    Serial.print(mac[5],HEX);
    Serial.print(":");
    Serial.print(mac[4],HEX);
    Serial.print(":");
    Serial.print(mac[3],HEX);
    Serial.print(":");
    Serial.print(mac[2],HEX);
    Serial.print(":");
    Serial.print(mac[1],HEX);
    Serial.print(":");
    Serial.println(mac[0],HEX);
  }
  ledState = HIGH;
//  digitalWrite(ledPin, ledState);
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
    // whatever the reading is at, it's been there for longer
    // than the debounce delay, so take it as the actual current state:
    Serial.println("ACCESSING WIFI DIRECT - AP");
    //WiFiManager

    //reset saved settings
    wifiManager.resetSettings();
    delay(2000);
    //sets timeout until configuration portal gets turned off
    //useful to make it all retry or go to sleep
    //in seconds
    ledState = LOW;
//    digitalWrite(ledPin, ledState);
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
      ledState = HIGH;
//      digitalWrite(ledPin, ledState);
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
      Serial.println("failed to connect, trying in seconds....");
      checkWifi = false;
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
//Send response after receiving command from broker
  if (checkWifi == true) {
    unsigned long currentMillis3 = millis();
    if (currentMillis3 - previousMillis3 > publishInveral) {
      previousMillis3 = currentMillis3;
      unsigned long currentMillis4 = millis();
      if (client.connected()) {
        keepAlive("Active");
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
