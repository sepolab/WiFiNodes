# WiFiNodes
This folder contain all embedded codes for all kinds of wifi nodes
Release 2.0
Room has got 4 prototype Nodes.

Usage (using for all Nodes):
1_Provide DC power 5V-1A
2_Reset Soft Reset button or connect GPIO 0 to GND in 3 seconds to start WiFi Direct Mode
3_Use phone or PC, search WiFi network name: TRI01. Default password: passsword
4_Auto-popup configuration page appears or access to 192.168.4.1 via web browser
5_In pop-up windows; access "Configure WiFi"
6_Input:
6.1_WiFi network name in list that this node will connect
6.2_WiFi password that this node will connect
6.3_MQTT server name
6.4_MQTT server port
6.5_Subcribe topic that this node will lisen
6.6_Publish topic that this node will send message
7_Save setting, wait for 5 seconds =>Done

TRI01_IRController:

Action: receive command from Broker then convert to IR signal to control devices: Toshiba AC; Daikin AC; Panasonic AC; Panasonic desk/wall fan; Mitshubishi Desk fan.
Format received message to control AC:
To turn ON: ACCONTROLLER ON
to Turn OFF: ACCONTROLLER OFF
To edit the AC: ACCONTROLER {"ID":"Toshiba","state":"on","mode":"cool","temp":"27","fan":"max","swing":"yes"}
- temp:16-->30
- mode: cool/dry/fan only/auto
- fan: auto/max/med/min
- swing: yes/no

Format received message to control Fan:
CONTROLWF {"ID":"Panasonic","command":"power"} 
- ID: Panasonic; Mitshubishi
command: power; oscil; speed; time; rhyth
