#include "EmonLib.h" 
#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager
#include <ESP8266WiFi.h>          //https://github.com/esp8266/Arduino
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <ArduinoJson.h>          //https://github.com/bblanchon/ArduinoJson
#include <FS.h>
#include <PubSubClient.h> //April23
#include "math.h"
//-------------------------------

//---------SOFT RESET BUTTON PARAMETERS---------------------
const int SOFT_RST_PIN =  0;      // SOFT RESET button is map to port 0
int buttonState;             // the current reading from the input pin
int lastButtonState = LOW;   // the previous reading from the input pin
// the following variables are long's because the time, measured in miliseconds,
// will quickly become a bigger number than can be stored in an int.
long lastDebounceTime = 0;  // the last time the output pin was toggled
long debounceDelay = 1000;    // the debounce time; increase if the output flickers

//--------LED INDICATION PARAMETERS--------------------
const int ledPin =  5; // the number of the LED pin
// Variables will change :
int ledState = HIGH;             // ledState used to set the LED
int configureState = 1;   //It is flag to recognize when is in configuration

unsigned long previousMillis = 0;        // will store last time LED was updated
unsigned long previousMillis1 = 0;       // will store last time LED was updated
unsigned long previousMillis2 = 0;
unsigned long previousMillis5 = 0;

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
char APpassword[20] = "tn17691510";
char ssid[] = "PowerMeasureNode"; // ESP in AP mode
char password[] = "password";
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
char pubTopicGen[40] = "Trihome/tri03/state";
char subTopicName[40] = "Trihome/tri03/set";
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


//---------------
//SendConfirm () will send feedback to MQTT to confirm the command 
//---------------
void sendConfirmtoRetained (char inputString[]) {
  firstTime = false;
  Serial.print("Message send: ");
  Serial.println(inputString);
  client.publish(pubTopicGen, inputString, true);
}
//-------------------------------------
//sendPowerMeasureIndex to update node status
//There is 2 states of interval
//1_Power Measure
//2_AC Current
//3_Voltage
//------------------------------------
void sendPowerMeasureIndex (int intervalSample) {
  double Irms = emon1.calcIrms(1480);  // Calculate Irms only
  double Prms = Irms*230.0/1000.0;
  double Ainstant = 0.0;
  double ArmsDisplay;
  char str_Irms[6];
  char str_Prms[8];
  char str_Arms[8];
    
  Serial.print("Prms = ");
  Serial.print(Prms);         // Apparent power
  Serial.print("kW"); 
  
  Serial.print("; Irms = ");
  Serial.print(Irms);
  Serial.print("A");

  Ainstant = Prms * intervalSample / 3600.0;
  Arms  = Arms + Ainstant;
  Serial.print("; Energy = ");
  
    ArmsDisplay = Arms / 1000.0;
    Serial.print(ArmsDisplay);
    Serial.print("kWh");          // Energy
    dtostrf(ArmsDisplay, 6, 2, str_Arms);
  
  Serial.print(" after ");
  runnningDuration = round (millis() / 1000);
  Serial.print(runnningDuration);
  Serial.print("s");
  
  Serial.println ();
  
  dtostrf(Irms, 4, 2, str_Irms);
  dtostrf(Prms, 8, 2, str_Prms);

  char publishMessage [150] = "";
  snprintf(publishMessage, 150, "{\"Prms\":\"%s\",\"Pde\":\"W\",\"Irms\":\"%s\",\"Ide\":\"A\",\"Vrms\":\"230\",\"Vde\":\"V\"}", str_Prms,str_Irms);
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
//-------------------------------------
//keep-alive interval to update node status
//There is 2 states of interval
//1_active when ping frequently
//2_reconnecting when re-connect
//------------------------------------
void keepAlive (char inputString[]) {
  char publishMessage [100] = "";
  snprintf(publishMessage, 100, "{\"NodeMacAddress\":\"%02X:%02X:%02X:%02X:%02X:%02X\",\"State\":\"%s\"}",mac[0],mac[1],mac[2],mac[3],mac[4],mac[5], inputString);
  Serial.print("Message send: ");
  Serial.println(publishMessage);
  char keepAliveTopic [55] = "";
  snprintf(keepAliveTopic, 55,"%s/keepAlive",pubTopicGen);
  client.publish(keepAliveTopic, publishMessage, true);
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
      // Once connected, publish an announcement...
      char temptCommand[] = "Reconnecting";
      keepAlive(temptCommand);
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
  emon1.current(A0, 16);             // Current: input pin, calibration.
  //calibration is explained bellow
//  if R burden = 22 for Irms max = 97 A => calibration = 91
//  if R burden = 120 for Irms max = 19.5 A => calibration = 17
//  if R burden = 470 for Irms max = 4.9 A => calibration = 5

  pinMode (ledPin, OUTPUT);
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
  }
  ledState = HIGH;
  digitalWrite(ledPin, ledState);
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
    ledState = LOW;
    digitalWrite(ledPin, ledState);
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
      digitalWrite(ledPin, ledState);
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
  else {
    setup_wifi();
  }
}
