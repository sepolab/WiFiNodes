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
  - GPIO 15: Door Sensor
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

TRI03_OutDoorNode:
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
  - GPIO 15: Door Sensor
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

