# WiFiNodes
This folder contain all embedded codes for all kinds of wifi nodes
Release 2.0
Room has got 4 prototype Nodes.

Usage (using for all Nodes):
- 1_Provide DC power 5V-1A
- 2_Reset Soft Reset button or connect GPIO 0 to GND in 3 seconds to start WiFi Direct Mode
- 3_Use phone or PC, search WiFi network name: TRI01. Default password: passsword
- 4_Auto-popup configuration page appears or access to 192.168.4.1 via web browser
- 5_In pop-up windows; access "Configure WiFi"
- 6_Input:
  -- 6.1_WiFi network name in list that this node will connect
  -- 6.2_WiFi password that this node will connect
  -- 6.3_MQTT server name
  -- 6.4_MQTT server port
  -- 6.5_Subcribe topic that this node will lisen
  -- 6.6_Publish topic that this node will send message
- 7_Save setting, wait for 5 seconds =>Done

TRI01_IRController:
- Input/Output:
  - GPIO 0: NA
  - GPIO 1: Tx
  - GPIO 2: LED - WiFi Config
  - GPIO 3: RX
  - GPIO 4: NA
  - GPIO 5: NA
  - GPIO 12: NA
  - GPIO 13: IR-Emitted PIN
  - GPIO 14: NA
  - GPIO 15: NA
  - GPIO 16: NA
- Action: 
  - receive command from Broker then convert to IR signal to control devices: Toshiba AC; Daikin AC; Panasonic AC; Panasonic desk/wall fan; Mitshubishi Desk fan.
- API:
  - For Air Conditioner:
    - To turn ON: ACCONTROLLER ON 
    - To Turn OFF: ACCONTROLLER OFF
    - To edit the AC: ACCONTROLER {"ID":"Toshiba","state":"on","mode":"cool","temp":"27","fan":"max","swing":"yes"}
      - temp:16-->30
      - mode: cool/dry/fan only/auto
      - fan: auto/max/med/min
      - swing: yes/no

  - For Wall Fan:
    - CONTROLWF {"ID":"Panasonic","command":"power"} 
    - ID: Panasonic; Mitshubishi
      - command: power; oscil; speed; time; rhyth

TRI02_PowerOutlet:
- INput/Output:
  - GPIO 0: NA
  - GPIO 1: Tx
  - GPIO 2: LED - WiFi Config
  - GPIO 3: RX
  - GPIO 4: NA
  - GPIO 5: NA
  - GPIO 12: Relay1 Control
  - GPIO 13: Relay1 Control
  - GPIO 14: NA
  - GPIO 15: button for relay1
  - GPIO 16: button for relay2
- Action:
  - Turn ON/OFF Relay to enable/disable Power Outlet usage
  - Buttons to give user can perform directly plysical to devices - no depends on Network
  - Sync states of relay from buttons to server.
- API:
  - For Relay1:
    - payload_on: SETCONTROL1 ON
    - payload_off: SETCONTROL1 OFF
  - For Relay2:
    - payload_on: SETCONTROL2 ON
    - payload_off: SETCONTROL2 OFF

TRI03_SecurityNode:
- INput/Output:
  - GPIO 0: DHT22 Thermo Sensor
  - GPIO 1: Tx
  - GPIO 2: LED - WiFi Config
  - GPIO 3: RX
  - GPIO 4: NA
  - GPIO 5: Speaker -Alarm
  - GPIO 12: Radar Detected Sensor
  - GPIO 13: Light Sensor
  - GPIO 14: NA
  - GPIO 15: NA
  - GPIO 16: Door Sensor
- Action:
  - Keep tracking temperature and humid via DHT22
  - Keep tracking Door sensor via DoorSensor
  - keep tracking Radar to detect motion
  - keep tracking brightness of enviroment via light sensor
  - Alarm when receive command (ALARM ON/ALARM OFF) from subcribe topic
- API:
  - For Alarm:
    - payload_on: ALARM ON
    - payload_off: ALARM OFF
  - For sensor:
    - JSON: {\"temperature\":\"%s\",\"humidity\":\"%s\",\"light\":\"%s\",\"door\":\"%s\",\"ishuman\":\"%s\"}"

TRI04_MixedForRoom:
- INput/Output:
  - GPIO 0: DHT21 Thermo Sensor
  - GPIO 1: Tx
  - GPIO 2: LED - WiFi Config
  - GPIO 3: RX
  - GPIO 4: NA
  - GPIO 5: NA
  - GPIO 12: PinRelay
  - GPIO 13: Light Sensor
  - GPIO 14: button for relay1
  - GPIO 15: NA
  - GPIO 16: Door Sensor
- Action:
  - Keep tracking temperature and humid via DHT21
  - Keep tracking Door sensor via DoorSensor
  - Turn ON/OFF Relay to enable/disable Power Outlet usage
  - Buttons to give user can perform directly plysical to devices - no depends on Network
  - Sync states of relay from buttons to server.
  - keep tracking brightness of enviroment via light sensor
- API:
  - For Relay:
    - payload_on: SETCONTROLL ON
    - payload_off: SETCONTROLL OFF
  - For sensor:
    - JSON: {\"temperature\":\"%s\",\"humidity\":\"%s\",\"light\":\"%s\",\"door\":\"%s\",\"switch\":\"%s\"}"
