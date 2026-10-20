#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include "driver/twai.h"

// =====================================================
// PINLER
// =====================================================

#define CAN_TX 2
#define CAN_RX 1

#define XY_TX 17
#define XY_RX 18

HardwareSerial XYSerial(1);
WebServer server(80);
DNSServer dnsServer;
const byte DNS_PORT = 53;
IPAddress apIP(192, 168, 4, 1);
IPAddress gateway(192, 168, 4, 1);
IPAddress subnet(255, 255, 255, 0);

// =====================================================
// WIFI
// =====================================================

const char* AP_SSID = "ELEKTRON-TERBİYECİSİ";
const char* AP_PASS = "12345678";

// =====================================================
// FLATPACK
// =====================================================

uint8_t flatpackSerial[6];
bool flatpackFound = false;

float fpVoltage = 0;
float fpCurrent = 0;
float fpPower = 0;
float fpInputVoltage = 0;
float fpInletTemp = 0;
float fpOutletTemp = 0;

float fpVoltageSet = 55.0;
float fpCurrentSet = 40.0;

bool fpCurrentLimiting = false;

String fpAlarm = "WAITING FOR FLATPACK";

unsigned long lastFPLogin = 0;
unsigned long lastFPCommand = 0;

// =====================================================
// XY6020L
// =====================================================

float xyVoltage = 0;
float xyCurrent = 0;
float xyPower = 0;
float xyInputVoltage = 0;

float xyVoltageSet = 12.0;
float xyCurrentSet = 1.0;

bool xyOutput = false;

uint16_t xyRegisters[20];

unsigned long lastXYRead = 0;

// =====================================================
// MODBUS CRC
// =====================================================

uint16_t modbusCRC(uint8_t *data, uint8_t len)
{
  uint16_t crc = 0xFFFF;

  for (uint8_t i = 0; i < len; i++)
  {
    crc ^= data[i];

    for (uint8_t j = 0; j < 8; j++)
    {
      if (crc & 1)
        crc = (crc >> 1) ^ 0xA001;
      else
        crc >>= 1;
    }
  }

  return crc;
}

// =====================================================
// XY6020L MODBUS WRITE
// =====================================================

bool xyWriteRegister(uint16_t reg, uint16_t value)
{
  uint8_t frame[6];

  frame[0] = 0x01;
  frame[1] = 0x06;

  frame[2] = reg >> 8;
  frame[3] = reg & 0xFF;

  frame[4] = value >> 8;
  frame[5] = value & 0xFF;

  uint16_t crc = modbusCRC(frame, 6);

  while (XYSerial.available())
    XYSerial.read();

  XYSerial.write(frame, 6);
  XYSerial.write(crc & 0xFF);
  XYSerial.write(crc >> 8);

  XYSerial.flush();

  delay(50);

  uint8_t response[16];
  int n = 0;

  unsigned long start = millis();

  while (millis() - start < 150)
  {
    while (XYSerial.available() && n < 16)
    {
      response[n++] = XYSerial.read();
    }
  }

  if (n < 8)
    return false;

  return true;
}

// =====================================================
// XY6020L READ REGISTERS
// =====================================================

bool xyReadRegisters(uint16_t startReg, uint16_t count)
{
  uint8_t frame[6];

  frame[0] = 0x01;
  frame[1] = 0x03;

  frame[2] = startReg >> 8;
  frame[3] = startReg & 0xFF;

  frame[4] = count >> 8;
  frame[5] = count & 0xFF;

  uint16_t crc = modbusCRC(frame, 6);

  while (XYSerial.available())
    XYSerial.read();

  XYSerial.write(frame, 6);
  XYSerial.write(crc & 0xFF);
  XYSerial.write(crc >> 8);

  XYSerial.flush();

  delay(50);

  uint8_t response[64];
  int n = 0;

  unsigned long start = millis();

  while (millis() - start < 150)
  {
    while (XYSerial.available() && n < 64)
    {
      response[n++] = XYSerial.read();
    }
  }

  if (n < 5)
    return false;

  if (response[0] != 0x01 ||
      response[1] != 0x03)
    return false;

  uint8_t byteCount = response[2];

  if (byteCount != count * 2)
    return false;

  if (n < 3 + byteCount)
    return false;

  for (uint16_t i = 0; i < count; i++)
  {
    xyRegisters[startReg + i] =
      ((uint16_t)response[3 + i * 2] << 8) |
      response[4 + i * 2];
  }

  return true;
}

// =====================================================
// XY6020L UPDATE
// =====================================================

void updateXY()
{
  if (xyReadRegisters(0x0000, 6))
  {
    xyVoltageSet =
      xyRegisters[0x0000] / 100.0;

    xyCurrentSet =
      xyRegisters[0x0001] / 100.0;

    xyVoltage =
      xyRegisters[0x0002] / 100.0;

    xyCurrent =
      xyRegisters[0x0003] / 100.0;

    xyPower =
      xyRegisters[0x0004] / 10.0;

    xyInputVoltage =
      xyRegisters[0x0005] / 100.0;
  }

  if (xyReadRegisters(0x0012, 1))
  {
    xyOutput =
      xyRegisters[0x0012] != 0;
  }
}

// =====================================================
// XY SET VOLTAGE
// =====================================================

void xySetVoltage(float voltage)
{
  if (voltage < 0 || voltage > 60)
    return;

  uint16_t value =
    round(voltage * 100.0);

  if (xyWriteRegister(0x0000, value))
  {
    xyVoltageSet = voltage;

    Serial.print("XY VSET = ");
    Serial.println(voltage);
  }
}

// =====================================================
// XY PRESET VOLTAGE
// =====================================================

void xyPresetVoltage(float voltage)
{
  if (voltage < 0 || voltage > 60)
    return;

  uint16_t value =
    round(voltage * 100.0);

  if (xyWriteRegister(0x0000, value))
  {
    xyVoltageSet = voltage;

    Serial.print("XY PRESET VSET = ");
    Serial.println(voltage);
  }
}

// =====================================================
// XY SET CURRENT
// =====================================================

void xySetCurrent(float current)
{
  if (current < 0 || current > 20)
    return;

  uint16_t value =
    round(current * 100.0);

  if (xyWriteRegister(0x0001, value))
  {
    xyCurrentSet = current;

    Serial.print("XY ISET = ");
    Serial.println(current);
  }
}

// =====================================================
// XY ON
// =====================================================

void xyON()
{
  if (xyWriteRegister(0x0012, 1))
  {
    xyOutput = true;
    Serial.println("XY OUTPUT ON");
  }
}

// =====================================================
// XY OFF
// =====================================================

void xyOFF()
{
  if (xyWriteRegister(0x0012, 0))
  {
    xyOutput = false;
    Serial.println("XY OUTPUT OFF");
  }
}

// =====================================================
// CAN SEND
// =====================================================

void sendCAN(
  uint32_t id,
  uint8_t *data,
  uint8_t len)
{
  twai_message_t msg = {};

  msg.identifier = id;
  msg.extd = 1;
  msg.data_length_code = len;

  for (int i = 0; i < len; i++)
    msg.data[i] = data[i];

  twai_transmit(
    &msg,
    pdMS_TO_TICKS(100)
  );
}

// =====================================================
// FLATPACK LOGIN
// =====================================================

void flatpackLogin()
{
  if (!flatpackFound)
    return;

  uint8_t data[8];

  for (int i = 0; i < 6; i++)
    data[i] = flatpackSerial[i];

  data[6] = 0;
  data[7] = 0;

  sendCAN(
    0x05004804,
    data,
    8
  );
}

// =====================================================
// FLATPACK SETPOINT
// =====================================================

void flatpackSetpoint()
{
  if (!flatpackFound)
    return;

  uint16_t currentRaw =
    round(fpCurrentSet * 10.0);

  uint16_t voltageRaw =
    round(fpVoltageSet * 100.0);

  uint16_t ovpRaw = 5760;

  uint8_t data[8];

  data[0] = currentRaw & 0xFF;
  data[1] = currentRaw >> 8;

  data[2] = voltageRaw & 0xFF;
  data[3] = voltageRaw >> 8;

  data[4] = voltageRaw & 0xFF;
  data[5] = voltageRaw >> 8;

  data[6] = ovpRaw & 0xFF;
  data[7] = ovpRaw >> 8;

  sendCAN(
    0x05FF4004,
    data,
    8
  );
}

// =====================================================
// FLATPACK STATUS
// =====================================================

void decodeFlatpack(
  twai_message_t &msg)
{
  fpInletTemp =
    msg.data[0];

  uint16_t currentRaw =
    ((uint16_t)msg.data[2] << 8) |
    msg.data[1];

  uint16_t voltageRaw =
    ((uint16_t)msg.data[4] << 8) |
    msg.data[3];

  uint16_t inputRaw =
    ((uint16_t)msg.data[6] << 8) |
    msg.data[5];

  fpOutletTemp =
    msg.data[7];

  fpCurrent =
    currentRaw / 10.0;

  fpVoltage =
    voltageRaw / 100.0;

  fpInputVoltage =
    inputRaw;

  fpPower =
    fpVoltage * fpCurrent;
}

// =====================================================
// CAN RECEIVE
// =====================================================

void readCAN()
{
  twai_message_t msg;

  while (
    twai_receive(
      &msg,
      0
    ) == ESP_OK)
  {
    uint32_t id =
      msg.identifier;

    if (
      id == 0x05014400 &&
      msg.data_length_code >= 6)
    {
      for (int i = 0; i < 6; i++)
        flatpackSerial[i] =
          msg.data[i];

      flatpackFound = true;

      fpAlarm =
        "EVERYTHING OK";
    }

    else if (
      id == 0x05014004 &&
      msg.data_length_code >= 8)
    {
      fpCurrentLimiting = false;

      decodeFlatpack(msg);

      fpAlarm =
        "EVERYTHING OK";
    }

    else if (
      id == 0x05014008 &&
      msg.data_length_code >= 8)
    {
      fpCurrentLimiting = true;

      decodeFlatpack(msg);

      fpAlarm =
        "CURRENT LIMITING";
    }

    else if (
      id == 0x0501400C)
    {
      if (msg.data_length_code >= 8)
        decodeFlatpack(msg);

      fpAlarm =
        "ALARM";
    }

    else if (
      id == 0x05014010)
    {
      if (msg.data_length_code >= 8)
        decodeFlatpack(msg);

      fpAlarm =
        "WALK-IN";
    }
  }
}

// =====================================================
// WEB DATA
// =====================================================

void handleData()
{
  String json = "{";

  json += "\"fpV\":";
  json += String(fpVoltage, 2);

  json += ",\"fpI\":";
  json += String(fpCurrent, 1);

  json += ",\"fpP\":";
  json += String(fpPower, 1);

  json += ",\"fpVin\":";
  json += String(fpInputVoltage, 0);

  json += ",\"fpTin\":";
  json += String(fpInletTemp, 0);

  json += ",\"fpTout\":";
  json += String(fpOutletTemp, 0);

  json += ",\"fpVSet\":";
  json += String(fpVoltageSet, 2);

  json += ",\"fpISet\":";
  json += String(fpCurrentSet, 1);

  json += ",\"fpAlarm\":\"";
  json += fpAlarm;
  json += "\"";

  json += ",\"xyV\":";
  json += String(xyVoltage, 2);

  json += ",\"xyI\":";
  json += String(xyCurrent, 2);

  json += ",\"xyP\":";
  json += String(xyPower, 1);

  json += ",\"xyVin\":";
  json += String(xyInputVoltage, 2);

  json += ",\"xyVSet\":";
  json += String(xyVoltageSet, 2);

  json += ",\"xyISet\":";
  json += String(xyCurrentSet, 2);

  json += ",\"xyOn\":";
  json += xyOutput ? "true" : "false";

  json += "}";

  server.send(
    200,
    "application/json",
    json
  );
}

// =====================================================
// FLATPACK WEB SET VOLTAGE
// =====================================================

void handleFPVoltage()
{
  if (!server.hasArg("v"))
  {
    server.send(400, "text/plain", "ERROR");
    return;
  }

  float v =
    server.arg("v").toFloat();

  if (v < 44.5 || v > 57.6)
  {
    server.send(
      400,
      "text/plain",
      "44.5 - 57.6 V"
    );

    return;
  }

  fpVoltageSet = v;

  server.send(
    200,
    "text/plain",
    "OK"
  );
}

// =====================================================
// FLATPACK WEB SET CURRENT
// =====================================================

void handleFPCurrent()
{
  if (!server.hasArg("i"))
  {
    server.send(400, "text/plain", "ERROR");
    return;
  }

  float i =
    server.arg("i").toFloat();

  if (i < 0.1 || i > 41.7)
  {
    server.send(
      400,
      "text/plain",
      "0.1 - 41.7 A"
    );

    return;
  }

  fpCurrentSet = i;

  server.send(
    200,
    "text/plain",
    "OK"
  );
}

// =====================================================
// XY WEB SET VOLTAGE
// =====================================================

void handleXYVoltage()
{
  if (!server.hasArg("v"))
  {
    server.send(400, "text/plain", "ERROR");
    return;
  }

  float v =
    server.arg("v").toFloat();

  if (v < 0 || v > 60)
  {
    server.send(
      400,
      "text/plain",
      "0 - 60 V"
    );

    return;
  }

  xySetVoltage(v);

  server.send(
    200,
    "text/plain",
    "OK"
  );
}

// =====================================================
// XY WEB PRESET VOLTAGE
// =====================================================

void handleXYPreset()
{
  if (!server.hasArg("v"))
  {
    server.send(400, "text/plain", "ERROR");
    return;
  }

  float v =
    server.arg("v").toFloat();

  if (v < 0 || v > 60)
  {
    server.send(
      400,
      "text/plain",
      "0 - 60 V"
    );

    return;
  }

  xyPresetVoltage(v);

  server.send(
    200,
    "text/plain",
    "OK"
  );
}

// =====================================================
// XY WEB SET CURRENT
// =====================================================

void handleXYCurrent()
{
  if (!server.hasArg("i"))
  {
    server.send(400, "text/plain", "ERROR");
    return;
  }

  float i =
    server.arg("i").toFloat();

  if (i < 0 || i > 20)
  {
    server.send(
      400,
      "text/plain",
      "0 - 20 A"
    );

    return;
  }

  xySetCurrent(i);

  server.send(
    200,
    "text/plain",
    "OK"
  );
}

// =====================================================
// WEB PAGE
// =====================================================

const char PAGE[] PROGMEM = R"rawliteral(

<!DOCTYPE html>

<html>

<head>

<meta charset="UTF-8">

<meta name="viewport"
content="width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no">

<title>Power Controller</title>

<style>

*{
box-sizing:border-box;
}

body{
margin:0;
background:#0d1117;
color:#fff;
font-family:Arial,sans-serif;
}

.container{
max-width:1100px;
margin:auto;
padding:18px;
}

h1{
text-align:center;
margin:5px 0 20px;
font-size:28px;
cursor:pointer;
user-select:none;
-webkit-tap-highlight-color:transparent;
}

h1:active{
opacity:.65;
transform:scale(.98);
}

.panels{
display:grid;
grid-template-columns:1fr 1fr;
gap:18px;
}

.panel{
background:#171c24;
border-radius:18px;
padding:18px;
box-shadow:0 4px 20px rgba(0,0,0,.25);
}

.title{
font-size:23px;
font-weight:bold;
text-align:center;
margin-bottom:15px;
}

.row{
display:flex;
justify-content:space-between;
align-items:center;
padding:12px 4px;
border-bottom:1px solid #2c333d;
}

.label{
color:#aab3c0;
}

.value{
font-size:21px;
font-weight:bold;
}

.setbox{
background:#222933;
border-radius:12px;
padding:14px;
margin-top:12px;
}

.settitle{
font-size:13px;
color:#aab3c0;
}

.setvalue{
font-size:25px;
font-weight:bold;
margin-top:5px;
}

.fakeInput{
width:100%;
margin-top:8px;
padding:12px;
background:#111;
color:white;
border:1px solid #555;
border-radius:7px;
font-size:18px;
min-height:46px;
cursor:pointer;
}

.buttons{
display:grid;
grid-template-columns:1fr 1fr;
gap:10px;
margin-top:15px;
}

.btn{
border:0;
border-radius:10px;
padding:15px;
font-size:18px;
font-weight:bold;
color:white;
cursor:pointer;
}

.on{
background:#b52b32;
}

.off{
background:#278b52;
}

.alarm{
margin-top:15px;
padding:14px;
border-radius:10px;
text-align:center;
background:#183b24;
font-weight:bold;
}

.presetGrid{
display:grid;
grid-template-columns:repeat(4,1fr);
gap:8px;
margin-top:12px;
}

.presetBtn{
border:0;
border-radius:9px;
padding:12px 4px;
font-size:15px;
font-weight:bold;
color:#17120a;
background:#d99a20;
cursor:pointer;
box-shadow:
0 2px 6px rgba(0,0,0,.3),
inset 0 1px rgba(255,255,255,.25);
}

.presetBtn:active{
background:#f0b52e;
transform:scale(.97);
}

.presetLabel{
font-size:12px;
color:#aab3c0;
margin-top:12px;
}

#keypadOverlay{
display:none;
position:fixed;
left:0;
top:0;
width:100%;
height:100%;
background:rgba(0,0,0,.72);
z-index:9999;
align-items:flex-end;
justify-content:center;
}

#keypad{
width:100%;
max-width:500px;
background:#171c24;
border-radius:20px 20px 0 0;
padding:16px;
box-shadow:0 -5px 30px rgba(0,0,0,.5);
}

.keypadTitle{
font-size:18px;
font-weight:bold;
text-align:center;
margin-bottom:5px;
}

.keypadRange{
font-size:13px;
color:#7f8b99;
text-align:center;
margin-bottom:10px;
}

#keypadDisplay{
width:100%;
height:52px;
background:#0d1117;
border:1px solid #555;
border-radius:9px;
color:white;
font-size:26px;
font-weight:bold;
text-align:right;
padding:8px 12px;
margin-bottom:10px;
}

.keyGrid{
display:grid;
grid-template-columns:repeat(3,1fr);
gap:8px;
}

.key{
height:58px;
border:0;
border-radius:10px;
background:#2a313c;
color:white;
font-size:23px;
font-weight:bold;
cursor:pointer;
-webkit-tap-highlight-color:transparent;
}

.key:active{
background:#414b59;
transform:scale(.97);
}

.keySpecial{
background:#3a424e;
font-size:18px;
}

.keySet{
background:#2969ff;
}

.keyCancel{
background:#b52b32;
}

.keyClear{
background:#555f6d;
}

@media(max-width:700px){

.panels{
grid-template-columns:1fr;
}

#keypad{
max-width:none;
}

.presetBtn{
font-size:14px;
padding:12px 2px;
}

}

</style>

</head>

<body>

<div class="container">

<h1 id="pageTitle"
onclick="toggleLanguage()">
POWER CONTROLLER
</h1>

<div class="panels">

<!-- =================================================
     FLATPACK
================================================= -->

<div class="panel">

<div class="title"
id="flatpackTitle">
FLATPACK 2
</div>

<div class="row">
<span class="label" id="fpVLabel">
Output Voltage
</span>
<span class="value" id="fpV">--</span>
</div>

<div class="row">
<span class="label" id="fpILabel">
Output Current
</span>
<span class="value" id="fpI">--</span>
</div>

<div class="row">
<span class="label" id="fpPLabel">
Output Power
</span>
<span class="value" id="fpP">--</span>
</div>

<div class="row">
<span class="label" id="fpVinLabel">
Grid Voltage
</span>
<span class="value" id="fpVin">--</span>
</div>

<div class="row">
<span class="label" id="fpTinLabel">
Inlet Temperature
</span>
<span class="value" id="fpTin">--</span>
</div>

<div class="row">
<span class="label" id="fpToutLabel">
Outlet Temperature
</span>
<span class="value" id="fpTout">--</span>
</div>

<div class="setbox">

<div class="settitle"
id="fpVSetLabel">
Voltage Setpoint
</div>

<div class="setvalue"
id="fpVSet">
--
</div>

<div
class="fakeInput"
id="fpVInput"
onclick="openKeypad('voltage')">
Tap to enter voltage
</div>

<button
class="btn"
id="fpVButton"
style="
width:100%;
margin-top:8px;
background:#2969ff;
"
onclick="openKeypad('voltage')">
SET VOLTAGE
</button>

</div>

<div class="setbox">

<div class="settitle"
id="fpISetLabel">
Current Limit
</div>

<div class="setvalue"
id="fpISet">
--
</div>

<div
class="fakeInput"
id="fpIInput"
onclick="openKeypad('current')">
Tap to enter current
</div>

<button
class="btn"
id="fpIButton"
style="
width:100%;
margin-top:8px;
background:#2969ff;
"
onclick="openKeypad('current')">
SET CURRENT
</button>

</div>

<div class="alarm"
id="fpAlarm">
WAITING
</div>

</div>

<!-- =================================================
     XY6020L
================================================= -->

<div class="panel">

<div class="title">
XY6020L
</div>

<div class="row">
<span class="label" id="xyVLabel">
Output Voltage
</span>
<span class="value" id="xyV">--</span>
</div>

<div class="row">
<span class="label" id="xyILabel">
Output Current
</span>
<span class="value" id="xyI">--</span>
</div>

<div class="row">
<span class="label" id="xyPLabel">
Output Power
</span>
<span class="value" id="xyP">--</span>
</div>

<div class="row">
<span class="label" id="xyVinLabel">
Input Voltage
</span>
<span class="value" id="xyVin">--</span>
</div>

<div class="setbox">

<div class="settitle"
id="xyVSetLabel">
Voltage Setpoint
</div>

<div class="setvalue"
id="xyVSet">
--
</div>

<div class="presetLabel"
id="presetLabel">
Quick Voltage
</div>

<div class="presetGrid">

<button
class="presetBtn"
onclick="xyPreset(3.3)">
3.3 V
</button>

<button
class="presetBtn"
onclick="xyPreset(5)">
5 V
</button>

<button
class="presetBtn"
onclick="xyPreset(12)">
12 V
</button>

<button
class="presetBtn"
onclick="xyPreset(24)">
24 V
</button>

</div>

<div
class="fakeInput"
id="xyVInput"
onclick="openXYKeypad('voltage')">
Tap to enter voltage
</div>

<button
class="btn"
id="xyVButton"
style="
width:100%;
margin-top:8px;
background:#2969ff;
"
onclick="openXYKeypad('voltage')">
SET VOLTAGE
</button>

</div>

<div class="setbox">

<div class="settitle"
id="xyISetLabel">
Current Limit
</div>

<div class="setvalue"
id="xyISet">
--
</div>

<div
class="fakeInput"
id="xyIInput"
onclick="openXYKeypad('current')">
Tap to enter current
</div>

<button
class="btn"
id="xyIButton"
style="
width:100%;
margin-top:8px;
background:#2969ff;
"
onclick="openXYKeypad('current')">
SET CURRENT
</button>

</div>

<div class="buttons">

<button
class="btn on"
id="xyOnButton"
onclick="xyON()">
ON
</button>

<button
class="btn off"
id="xyOffButton"
onclick="xyOFF()">
OFF
</button>

</div>

<div class="alarm"
id="xyStatus">
OUTPUT OFF
</div>

</div>

</div>

</div>

<!-- =================================================
     NUMERIC KEYPAD
================================================= -->

<div id="keypadOverlay">

<div id="keypad">

<div class="keypadTitle"
id="keypadTitle">
Voltage Setpoint
</div>

<div class="keypadRange"
id="keypadRange">
Range: 44.5 - 57.6 V
</div>

<div id="keypadDisplay">
0
</div>

<div class="keyGrid">

<button class="key"
onclick="keyPress('1')">1</button>

<button class="key"
onclick="keyPress('2')">2</button>

<button class="key"
onclick="keyPress('3')">3</button>

<button class="key"
onclick="keyPress('4')">4</button>

<button class="key"
onclick="keyPress('5')">5</button>

<button class="key"
onclick="keyPress('6')">6</button>

<button class="key"
onclick="keyPress('7')">7</button>

<button class="key"
onclick="keyPress('8')">8</button>

<button class="key"
onclick="keyPress('9')">9</button>

<button class="key keySpecial"
onclick="keyPress('.')">.</button>

<button class="key"
onclick="keyPress('0')">0</button>

<button class="key keySpecial"
onclick="keyBackspace()">⌫</button>

<button class="key keyClear"
id="keyClear"
onclick="keyClear()">CLR</button>

<button class="key keyCancel"
id="keyCancel"
onclick="keyCancel()">CANCEL</button>

<button class="key keySet"
id="keySet"
onclick="keySet()">SET</button>

</div>

</div>

</div>

<script>

// =====================================================
// LANGUAGE
// =====================================================

let isTurkish = false;

function toggleLanguage()
{
  isTurkish = !isTurkish;

  applyLanguage();

  update();
}

function applyLanguage()
{
  if(isTurkish)
  {
    document.getElementById('pageTitle').innerText =
      'GÜÇ KONTROL SİSTEMİ';

    document.getElementById('fpVLabel').innerText =
      'Çıkış Gerilimi';

    document.getElementById('fpILabel').innerText =
      'Çıkış Akımı';

    document.getElementById('fpPLabel').innerText =
      'Çıkış Gücü';

    document.getElementById('fpVinLabel').innerText =
      'Şebeke Gerilimi';

    document.getElementById('fpTinLabel').innerText =
      'Giriş Sıcaklığı';

    document.getElementById('fpToutLabel').innerText =
      'Çıkış Sıcaklığı';

    document.getElementById('fpVSetLabel').innerText =
      'Gerilim Referansı';

    document.getElementById('fpISetLabel').innerText =
      'Akım Sınırı';

    document.getElementById('xyVLabel').innerText =
      'Çıkış Gerilimi';

    document.getElementById('xyILabel').innerText =
      'Çıkış Akımı';

    document.getElementById('xyPLabel').innerText =
      'Çıkış Gücü';

    document.getElementById('xyVinLabel').innerText =
      'Giriş Gerilimi';

    document.getElementById('xyVSetLabel').innerText =
      'Gerilim Referansı';

    document.getElementById('xyISetLabel').innerText =
      'Akım Sınırı';

    document.getElementById('presetLabel').innerText =
      'Hızlı Gerilim Seçimi';

    document.getElementById('fpVInput').innerText =
      'Gerilim değeri girmek için dokunun';

    document.getElementById('fpIInput').innerText =
      'Akım değeri girmek için dokunun';

    document.getElementById('xyVInput').innerText =
      'Gerilim değeri girmek için dokunun';

    document.getElementById('xyIInput').innerText =
      'Akım değeri girmek için dokunun';

    document.getElementById('fpVButton').innerText =
      'GERİLİMİ AYARLA';

    document.getElementById('fpIButton').innerText =
      'AKIMI AYARLA';

    document.getElementById('xyVButton').innerText =
      'GERİLİMİ AYARLA';

    document.getElementById('xyIButton').innerText =
      'AKIMI AYARLA';

    document.getElementById('xyOnButton').innerText =
      'AÇ';

    document.getElementById('xyOffButton').innerText =
      'KAPAT';

    document.getElementById('keyClear').innerText =
      'TEMİZLE';

    document.getElementById('keyCancel').innerText =
      'İPTAL';

    document.getElementById('keySet').innerText =
      'AYARLA';
  }

  else
  {
    document.getElementById('pageTitle').innerText =
      'POWER CONTROLLER';

    document.getElementById('fpVLabel').innerText =
      'Output Voltage';

    document.getElementById('fpILabel').innerText =
      'Output Current';

    document.getElementById('fpPLabel').innerText =
      'Output Power';

    document.getElementById('fpVinLabel').innerText =
      'Grid Voltage';

    document.getElementById('fpTinLabel').innerText =
      'Inlet Temperature';

    document.getElementById('fpToutLabel').innerText =
      'Outlet Temperature';

    document.getElementById('fpVSetLabel').innerText =
      'Voltage Setpoint';

    document.getElementById('fpISetLabel').innerText =
      'Current Limit';

    document.getElementById('xyVLabel').innerText =
      'Output Voltage';

    document.getElementById('xyILabel').innerText =
      'Output Current';

    document.getElementById('xyPLabel').innerText =
      'Output Power';

    document.getElementById('xyVinLabel').innerText =
      'Input Voltage';

    document.getElementById('xyVSetLabel').innerText =
      'Voltage Setpoint';

    document.getElementById('xyISetLabel').innerText =
      'Current Limit';

    document.getElementById('presetLabel').innerText =
      'Quick Voltage';

    document.getElementById('fpVInput').innerText =
      'Tap to enter voltage';

    document.getElementById('fpIInput').innerText =
      'Tap to enter current';

    document.getElementById('xyVInput').innerText =
      'Tap to enter voltage';

    document.getElementById('xyIInput').innerText =
      'Tap to enter current';

    document.getElementById('fpVButton').innerText =
      'SET VOLTAGE';

    document.getElementById('fpIButton').innerText =
      'SET CURRENT';

    document.getElementById('xyVButton').innerText =
      'SET VOLTAGE';

    document.getElementById('xyIButton').innerText =
      'SET CURRENT';

    document.getElementById('xyOnButton').innerText =
      'ON';

    document.getElementById('xyOffButton').innerText =
      'OFF';

    document.getElementById('keyClear').innerText =
      'CLR';

    document.getElementById('keyCancel').innerText =
      'CANCEL';

    document.getElementById('keySet').innerText =
      'SET';
  }
}

// =====================================================
// ALARM TRANSLATION
// =====================================================

function translateAlarm(alarm)
{
  if(!isTurkish)
    return alarm;

  if(alarm === "EVERYTHING OK")
    return "NORMAL";

  if(alarm === "CURRENT LIMITING")
    return "AKIM SINIRLAMASI";

  if(alarm === "ALARM")
    return "ALARM";

  if(alarm === "WALK-IN")
    return "YUMUŞAK BAŞLATMA";

  if(alarm === "WAITING FOR FLATPACK")
    return "FLATPACK BEKLENİYOR";

  return alarm;
}

// =====================================================
// UPDATE
// =====================================================

function update()
{
  fetch('/data')
  .then(r => r.json())
  .then(d =>
  {
    document.getElementById('fpV').innerText =
      d.fpV.toFixed(2) + ' V';

    document.getElementById('fpI').innerText =
      d.fpI.toFixed(1) + ' A';

    document.getElementById('fpP').innerText =
      d.fpP.toFixed(1) + ' W';

    document.getElementById('fpVin').innerText =
      d.fpVin.toFixed(0) + ' V';

    document.getElementById('fpTin').innerText =
      d.fpTin.toFixed(0) + ' °C';

    document.getElementById('fpTout').innerText =
      d.fpTout.toFixed(0) + ' °C';

    document.getElementById('fpVSet').innerText =
      d.fpVSet.toFixed(2) + ' V';

    document.getElementById('fpISet').innerText =
      d.fpISet.toFixed(1) + ' A';

    document.getElementById('fpAlarm').innerText =
      translateAlarm(d.fpAlarm);

    document.getElementById('xyV').innerText =
      d.xyV.toFixed(2) + ' V';

    document.getElementById('xyI').innerText =
      d.xyI.toFixed(2) + ' A';

    document.getElementById('xyP').innerText =
      d.xyP.toFixed(1) + ' W';

    document.getElementById('xyVin').innerText =
      d.xyVin.toFixed(2) + ' V';

    document.getElementById('xyVSet').innerText =
      d.xyVSet.toFixed(2) + ' V';

    document.getElementById('xyISet').innerText =
      d.xyISet.toFixed(2) + ' A';

    document.getElementById('xyStatus').innerText =
      d.xyOn
      ? (isTurkish ? 'ÇIKIŞ AKTİF' : 'OUTPUT ON')
      : (isTurkish ? 'ÇIKIŞ DEVRE DIŞI' : 'OUTPUT OFF');
  });
}

// =====================================================
// FLATPACK KEYPAD
// =====================================================

function openKeypad(mode)
{
  keypadMode = mode;
  keypadValue = "";

  if(mode === 'voltage')
  {
    document.getElementById('keypadTitle').innerText =
      isTurkish ?
      'Gerilim Referansı' :
      'Voltage Setpoint';

    document.getElementById('keypadRange').innerText =
      isTurkish ?
      'Aralık: 44.5 - 57.6 V' :
      'Range: 44.5 - 57.6 V';
  }
  else
  {
    document.getElementById('keypadTitle').innerText =
      isTurkish ?
      'Akım Sınırı' :
      'Current Limit';

    document.getElementById('keypadRange').innerText =
      isTurkish ?
      'Aralık: 0.1 - 41.7 A' :
      'Range: 0.1 - 41.7 A';
  }

  updateKeypadDisplay();

  document.getElementById('keypadOverlay').style.display =
    'flex';

  document.body.style.overflow =
    'hidden';
}

// =====================================================
// XY KEYPAD
// =====================================================

function openXYKeypad(mode)
{
  keypadMode =
    'xy_' + mode;

  keypadValue = "";

  if(mode === 'voltage')
  {
    document.getElementById('keypadTitle').innerText =
      isTurkish ?
      'XY6020L Gerilim' :
      'XY6020L Voltage';

    document.getElementById('keypadRange').innerText =
      isTurkish ?
      'Aralık: 0 - 60 V' :
      'Range: 0 - 60 V';
  }
  else
  {
    document.getElementById('keypadTitle').innerText =
      isTurkish ?
      'XY6020L Akım' :
      'XY6020L Current';

    document.getElementById('keypadRange').innerText =
      isTurkish ?
      'Aralık: 0 - 20 A' :
      'Range: 0 - 20 A';
  }

  updateKeypadDisplay();

  document.getElementById('keypadOverlay').style.display =
    'flex';

  document.body.style.overflow =
    'hidden';
}

// =====================================================
// XY PRESET
// =====================================================

function xyPreset(voltage)
{
  fetch(
    '/xyPreset?v=' +
    encodeURIComponent(voltage)
  )
  .then(() =>
  {
    document.getElementById('xyVInput').innerText =
      voltage.toFixed(2) + ' V';

    document.getElementById('xyVSet').innerText =
      voltage.toFixed(2) + ' V';
  });
}

// =====================================================
// KEYPAD DISPLAY
// =====================================================

function updateKeypadDisplay()
{
  document.getElementById('keypadDisplay').innerText =
    keypadValue === "" ?
    "0" :
    keypadValue;
}

// =====================================================
// KEYPRESS
// =====================================================

function keyPress(key)
{
  if(
    key === '.' &&
    keypadValue.includes('.')
  )
    return;

  if(
    key === '.' &&
    keypadValue === ""
  )
  {
    keypadValue = "0.";
  }
  else
  {
    keypadValue += key;
  }

  if(keypadValue.length > 6)
  {
    keypadValue =
      keypadValue.substring(0, 6);
  }

  updateKeypadDisplay();
}

// =====================================================
// BACKSPACE
// =====================================================

function keyBackspace()
{
  if(keypadValue.length > 0)
  {
    keypadValue =
      keypadValue.substring(
        0,
        keypadValue.length - 1
      );
  }

  updateKeypadDisplay();
}

// =====================================================
// CLEAR
// =====================================================

function keyClear()
{
  keypadValue = "";

  updateKeypadDisplay();
}

// =====================================================
// CANCEL
// =====================================================

function keyCancel()
{
  keypadValue = "";

  document.getElementById(
    'keypadOverlay'
  ).style.display = 'none';

  document.body.style.overflow = '';
}

// =====================================================
// SET
// =====================================================

function keySet()
{
  if(keypadValue === "")
    return;

  let value =
    parseFloat(keypadValue);

  if(isNaN(value))
    return;

  if(keypadMode === "voltage")
  {
    if(
      value < 44.5 ||
      value > 57.6
    )
    {
      alert(
        isTurkish ?
        "Gerilim aralığı: 44.5 - 57.6 V" :
        "Voltage range: 44.5 - 57.6 V"
      );

      return;
    }

    fetch(
      '/fpVoltage?v=' +
      encodeURIComponent(value)
    );

    document.getElementById(
      'fpVInput'
    ).innerText =
      value.toFixed(1) + " V";
  }

  else if(keypadMode === "current")
  {
    if(
      value < 0.1 ||
      value > 41.7
    )
    {
      alert(
        isTurkish ?
        "Akım aralığı: 0.1 - 41.7 A" :
        "Current range: 0.1 - 41.7 A"
      );

      return;
    }

    fetch(
      '/fpCurrent?i=' +
      encodeURIComponent(value)
    );

    document.getElementById(
      'fpIInput'
    ).innerText =
      value.toFixed(1) + " A";
  }

  else if(keypadMode === "xy_voltage")
  {
    if(
      value < 0 ||
      value > 60
    )
    {
      alert(
        isTurkish ?
        "Gerilim aralığı: 0 - 60 V" :
        "Voltage range: 0 - 60 V"
      );

      return;
    }

    fetch(
      '/xyVoltage?v=' +
      encodeURIComponent(value)
    );

    document.getElementById(
      'xyVInput'
    ).innerText =
      value.toFixed(2) + " V";
  }

  else if(keypadMode === "xy_current")
  {
    if(
      value < 0 ||
      value > 20
    )
    {
      alert(
        isTurkish ?
        "Akım aralığı: 0 - 20 A" :
        "Current range: 0 - 20 A"
      );

      return;
    }

    fetch(
      '/xyCurrent?i=' +
      encodeURIComponent(value)
    );

    document.getElementById(
      'xyIInput'
    ).innerText =
      value.toFixed(2) + " A";
  }

  document.getElementById(
    'keypadOverlay'
  ).style.display = 'none';

  document.body.style.overflow = '';

  keypadValue = "";
}

// =====================================================
// XY ON / OFF
// =====================================================

function xyON()
{
  fetch('/xyON');
}

function xyOFF()
{
  fetch('/xyOFF');
}

// =====================================================
// START
// =====================================================

setInterval(
  update,
  1000
);

update();

</script>

</body>

</html>

)rawliteral";

// =====================================================
// CAPTIVE PORTAL
// =====================================================

void captivePortalRedirect()
{
  server.sendHeader(
    "Location",
    "http://192.168.4.1/",
    true
  );

  server.send(
    302,
    "text/plain",
    ""
  );
}

// =====================================================
// SETUP
// =====================================================

void setup()
{
  Serial.begin(115200);

  delay(1000);

  // ===================================================
  // CAN
  // ===================================================

  twai_general_config_t g_config =
    TWAI_GENERAL_CONFIG_DEFAULT(
      (gpio_num_t)CAN_TX,
      (gpio_num_t)CAN_RX,
      TWAI_MODE_NORMAL
    );

  twai_timing_config_t t_config =
    TWAI_TIMING_CONFIG_125KBITS();

  twai_filter_config_t f_config =
    TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if(
    twai_driver_install(
      &g_config,
      &t_config,
      &f_config
    ) != ESP_OK)
  {
    Serial.println(
      "CAN INSTALL ERROR"
    );

    return;
  }

  if(
    twai_start() != ESP_OK)
  {
    Serial.println(
      "CAN START ERROR"
    );

    return;
  }

  Serial.println(
    "FLATPACK CAN OK"
  );

  // ===================================================
  // XY6020L
  // ===================================================

  XYSerial.begin(
    115200,
    SERIAL_8N1,
    XY_RX,
    XY_TX
  );

  Serial.println(
    "XY6020L UART OK"
  );

  // ===================================================
  // WIFI
  // ===================================================

  WiFi.mode(WIFI_AP);

  WiFi.softAPConfig(
    apIP,
    gateway,
    subnet
  );

  WiFi.softAP(
    AP_SSID,
    AP_PASS
  );

  dnsServer.start(
    DNS_PORT,
    "*",
    apIP
  );

  Serial.print(
    "WiFi IP: "
  );

  Serial.println(
    WiFi.softAPIP()
  );

  // ===================================================
  // WEB
  // ===================================================

  server.on(
    "/",
    []()
    {
      server.send_P(
        200,
        "text/html; charset=UTF-8",
        PAGE
      );
    }
  );

  server.on(
    "/data",
    handleData
  );

  server.on(
    "/fpVoltage",
    handleFPVoltage
  );

  server.on(
    "/fpCurrent",
    handleFPCurrent
  );

  server.on(
    "/xyVoltage",
    handleXYVoltage
  );

  server.on(
    "/xyPreset",
    handleXYPreset
  );

  server.on(
    "/xyCurrent",
    handleXYCurrent
  );

  server.on(
    "/xyON",
    []()
    {
      xyON();

      server.send(
        200,
        "text/plain",
        "OK"
      );
    }
  );

  server.on(
    "/xyOFF",
    []()
    {
      xyOFF();

      server.send(
        200,
        "text/plain",
        "OK"
      );
    }
  );

  // ===================================================
  // CAPTIVE PORTAL ENDPOINTS
  // ===================================================

  server.on(
    "/generate_204",
    captivePortalRedirect
  );

  server.on(
    "/gen_204",
    captivePortalRedirect
  );

  server.on(
    "/hotspot-detect.html",
    captivePortalRedirect
  );

  server.on(
    "/connecttest.txt",
    captivePortalRedirect
  );

  server.on(
    "/ncsi.txt",
    captivePortalRedirect
  );

  server.on(
    "/canonical.html",
    captivePortalRedirect
  );

  server.on(
    "/fwlink",
    captivePortalRedirect
  );

  server.on(
    "/success.txt",
    captivePortalRedirect
  );

  server.onNotFound(
    captivePortalRedirect
  );

  server.begin();

  Serial.println(
    "WEB SERVER STARTED"
  );

  Serial.println(
    "CAPTIVE PORTAL STARTED"
  );
}

// =====================================================
// LOOP
// =====================================================

void loop()
{
  dnsServer.processNextRequest();

  server.handleClient();

  // ---------------------------------------------
  // Flatpack CAN
  // ---------------------------------------------

  readCAN();

  if(
    millis() - lastFPLogin > 2000)
  {
    if(flatpackFound)
    {
      flatpackLogin();

      lastFPLogin =
        millis();
    }
  }

  if(
    millis() - lastFPCommand > 1000)
  {
    if(flatpackFound)
    {
      flatpackSetpoint();

      lastFPCommand =
        millis();
    }
  }

  // ---------------------------------------------
  // XY6020L
  // ---------------------------------------------

  if(
    millis() - lastXYRead > 500)
  {
    updateXY();

    lastXYRead =
      millis();
  }
}
