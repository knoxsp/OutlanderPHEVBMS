/*
  Copyright (c) 2019 Simp ECO Engineering
  Permission is hereby granted, free of charge, to any person obtaining
  a copy of this software and associated documentation files (the
  "Software"), to deal in the Software without restriction, including
  without limitation the rights to use, copy, modify, merge, publish,
  distribute, sublicense, and/or sell copies of the Software, and to
  permit persons to whom the Software is furnished to do so, subject to
  the following conditions:
  The above copyright notice and this permission notice shall be included
  in all copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
  MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
  IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
  CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
  TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
  SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

  Thank you to James Warner for proving out canbus decoding and finding balancing bits
/////////////////////////////////////////////////////////////////////////////////////////////////

░██████╗██╗███╗░░░███╗██████╗░██████╗░███╗░░░███╗░██████╗  
██╔════╝██║████╗░████║██╔══██╗██╔══██╗████╗░████║██╔════╝  
╚█████╗░██║██╔████╔██║██████╔╝██████╦╝██╔████╔██║╚█████╗░  
░╚═══██╗██║██║╚██╔╝██║██╔═══╝░██╔══██╗██║╚██╔╝██║░╚═══██╗  
██████╔╝██║██║░╚═╝░██║██║░░░░░██████╦╝██║░╚═╝░██║██████╔╝  
╚═════╝░╚═╝╚═╝░░░░░╚═╝╚═╝░░░░░╚═════╝░╚═╝░░░░░╚═╝╚═════╝░  

░██████╗██████╗░░█████╗░░█████╗░███████╗  ██████╗░░█████╗░██╗░░░░░██╗░░░░░░██████╗
██╔════╝██╔══██╗██╔══██╗██╔══██╗██╔════╝  ██╔══██╗██╔══██╗██║░░░░░██║░░░░░██╔════╝
╚█████╗░██████╔╝███████║██║░░╚═╝█████╗░░  ██████╦╝███████║██║░░░░░██║░░░░░╚█████╗░
░╚═══██╗██╔═══╝░██╔══██║██║░░██╗██╔══╝░░  ██╔══██╗██╔══██║██║░░░░░██║░░░░░░╚═══██╗
██████╔╝██║░░░░░██║░░██║╚█████╔╝███████╗  ██████╦╝██║░░██║███████╗███████╗██████╔╝
╚═════╝░╚═╝░░░░░╚═╝░░╚═╝░╚════╝░╚══════╝  ╚═════╝░╚═╝░░╚═╝╚══════╝╚══════╝╚═════╝░

███████╗██████╗░██╗████████╗██╗░█████╗░███╗░░██╗
██╔════╝██╔══██╗██║╚══██╔══╝██║██╔══██╗████╗░██║
█████╗░░██║░░██║██║░░░██║░░░██║██║░░██║██╔██╗██║
██╔══╝░░██║░░██║██║░░░██║░░░██║██║░░██║██║╚████║
███████╗██████╔╝██║░░░██║░░░██║╚█████╔╝██║░╚███║
╚══════╝╚═════╝░╚═╝░░░╚═╝░░░╚═╝░╚════╝░╚═╝░░╚══╝


This version of SimpBMS has been modified as the Space Balls edition utilising the Teensey 3.6, alowingfor upto 4 Canbus'
  2 Native(Flexcans) + 2 MCP2515/SPI Cans
*/

//#include "BMSModuleManager.h"
#include <Arduino.h>
#include "config.h"
//#include "SerialConsole.h"
//#include "Logger.h"
#include <EEPROM.h>
#include <SPI.h>
#include "BMSCan.h"
// How do make it so that we don't have to include each charger individually?
#include "OutlanderCharger.h"
#include "Kangoo36.h"

#include "BMSWebServer.h"
// #include <esp_task_wdt.h>
#include <SPIFFS.h>

#define CPU_REBOOT (ESP.restart());
#define WDT_TIMEOUT 5
#define CONFIG_ESP_TASK_WDT_IDLE_TIMEOUT 1
#define HOSTNAME "BALLSBMS"

EEPROMSettings settings;
BMSCan bmscan;
BMS_CAN_MESSAGE msg;
BMS_CAN_MESSAGE inMsg;

// TODO: How do we make this web-configurable?
OutlanderCharger charger(bmscan, settings); // on the charger can index (can 1)
KangooCan bms(bmscan, settings); // On the VE Can Index (can 0)

BMSWebServer bmsWebServer(settings, bms);


/////Version Identifier/////////
int firmver = 230720;

// Simple BMS V2 wiring//
const int FUEL_GAUGE_PWM = 13;  //
const int PP_DETECT = 34;  // DIN1 (Digital In 1) - high active //PP Detect
const int CRANK_IN = 35;    // DIN2 - high active //repurpose for Crank signal
const int OUT_FAN = 32;     // DOUT1 - switched ground. high active // for fan
const int OUT_DCDC_ENABLE = 33; // DOUT2 - switched ground. high active //DC_DC Enable

const int OUT_NEG_CONTACTOR = 21; // DOUT7 (v1.0) - switched ground. high active // NEG CONTACTOR
//const int OUT_NEG_CONTACTOR = 22; // DOUT6 (v1.0) - switched ground. high active // NEG CONTACTOR
const int led = 2;
const int BMBfault = 11;

const int ppInputPin = 36;  // PP analog input pin

uint8_t bmsstatus = 0;

// bms status values
#define Boot 0
#define Ready 1
#define Drive 2
#define Charge 3
#define Precharge 4
#define RapidCharge 5
#define Error 6
//
// Current sensor values
#define Undefined 0
#define Canbus 1

// Charger Types
// Charger Types
#define NoCharger 0
#define Outlander 1

bool crankSeen = false;

int Discharge;

/*TODO: Explain the different uses of ErrorReason*/
int ErrorReason = 0;

// variables for VE can
uint16_t chargevoltage = 49100; // max charge voltage in mv
int chargecurrent;
int evse_duty;
uint16_t disvoltage = 42000; // max discharge voltage in mv
int discurrent;

unsigned char bmsAlarm[4] = {0, 0, 0, 0};
unsigned char bmsWarning[4] = {0, 0, 0, 0};
unsigned char mes[8] = {0, 0, 0, 0, 0, 0, 0, 0};

long unsigned int rxId;
unsigned char len = 0;
uint8_t rxBuf[8];
char msgString[128]; // Array to store serial string
uint32_t inbox;
int32_t CANmilliamps;
signed long voltage1, voltage2, voltage3 = 0; // mV only with ISAscale sensor
double amphours, kilowatthours, kilowatts;    // only with ISAscale sensor

// variables for current calulation
float currentact, RawCur;
int inverterTemp;
int motorTemp;
int driveMode; // can be 0 (park), 1(reverse), 2(neutral), 3(drive)
int prechargeContactor = 0;
int mainContactor = 0;
bool rapidCharging = false;
unsigned long statustimer, hvstatustimer = 0; /*this resets every time a message is recieved from the VCU containing precharge and main contactor data*/
unsigned long looptime, chargerSpeedLoopTime, bmsSpeedLoopTime, UnderTime, OverTime = 0; // ms

// variables
int x = 0;
int storagemode = 0;

// Debugging modes//////////////////
int debug = 1;
int candebug = 0;    // view can frames
int debugdigits = 2; // amount of digits behind decimal for voltage reading

bool checkPPDetect(uint8_t index)
{
  return digitalRead(PP_DETECT) == LOW;
}

bool negContactorClosed()
{
  return digitalRead(OUT_NEG_CONTACTOR) == HIGH;
}

bool carInDrive()
{
  /*Car is in drive (3) or reverse (1)*/
  return driveMode == 3 || driveMode == 1;
}

void loadSettings()
{
//  Logger::console("Resetting to factory defaults");
  settings.version = EEPROM_VERSION;
  settings.checksum = 2;
  settings.canSpeed = 500000;
  settings.batteryID = 0x01; // in the future should be 0xFF to force it to ask for an address
  settings.OverVSetpoint = 4.2f;
  settings.UnderVSetpoint = 3.0f;
  settings.ChargeVsetpoint = 4.1f;
  settings.ChargeHys = 0.2f; // voltage drop required for charger to kick back on
  settings.WarnOff = 0.1f;   // voltage offset to raise a warning
  settings.DischVsetpoint = 3.2f;
  settings.DischHys = 0.2f; // Discharge voltage offset
  settings.CellGap = 0.2f;  // max delta between high and low cell
  settings.OverTSetpoint = 65.0f;
  settings.UnderTSetpoint = -10.0f;
  settings.ChargeTSetpoint = 0.0f;
  settings.triptime = 500; // mS of delay before counting over or undervoltage
  settings.DisTSetpoint = 40.0f;
  settings.WarnToff = 5.0f;  // temp offset before raising warning
  settings.IgnoreTemp = 0;   // 0 - use both sensors, 1 or 2 only use that sensor
  settings.IgnoreVolt = 0.5; //
  settings.balanceVoltage = 3.9f;
  settings.balanceHyst = 0.04f;
  settings.balanceDuty = 60;
  settings.logLevel = 2;
  settings.CAP = 37.5;              // battery size in Ah
  settings.Pstrings = 1;           // strings in parallel used to divide voltage of pack
  settings.Scells = 96;            // Cells in series
  settings.StoreVsetpoint = 3.8;   // V storage mode charge max
  settings.discurrentmax = 300;    // max discharge current in 0.1A
  settings.DisTaper = 0.3f;        // V offset to bring in discharge taper to Zero Amps at settings.DischVsetpoint
  settings.chargecurrentmax = 300; // max charge current in 0.1A
  settings.rapidchargecurrentmax = 1200;
  settings.chargecurrentend = 50;                         // end charge current in 0.1A
  settings.socvolt[0] = 3100;                             // Voltage and SOC curve for voltage based SOC calc
  settings.socvolt[1] = 10;                               // Voltage and SOC curve for voltage based SOC calc
  settings.socvolt[2] = 4100;                             // Voltage and SOC curve for voltage based SOC calc
  settings.socvolt[3] = 90;                               // Voltage and SOC curve for voltage based SOC calc
  settings.invertcur = 0;                                 // Invert current sensor direction
  settings.chargerCanIndex = SECOND_CAN_INTERFACE_INDEX; // default to can1
  settings.veCanIndex = DEFAULT_CAN_INTERFACE_INDEX;      // default to can0
  settings.voltsoc = 0;                                   // SOC purely voltage based
  settings.conthold = 50;                                 // holding duty cycle for contactor 0-255
  settings.Precurrent = 1000;                             // ma before closing main contator
  settings.convhigh = 580;                                // mV/A current sensor high range channel//SK WHY dffernet numbers compared to BMW?
  settings.convlow = 6430;                                // mV/A current sensor low range channel//SK WHY dffernet numbers compared to BMW?
  settings.changecur = 20000;                             // mA change overpoint
  settings.offset1 = 1750;                                // mV mid point of channel 1
  settings.offset2 = 1750;                                // mV mid point of channel 2
  settings.gaugelow = 50;                                 // empty fuel gauge pwm
  settings.gaugehigh = 255;                               // full fuel gauge pwm
  settings.ncur = 1;                                      // number of multiples to use for current measurement
  settings.chargertype = 2;                               // 1 - Brusa NLG5xx 2 - Volt charger 0 -No Charger
  settings.chargerspd = 100;                              // ms per message
  settings.bmsspeed = 200;                                // ms between bms keep-alive message
  settings.CurDead = 5;                                   // mV of dead band on current sensor
  settings.ChargerDirect = 1;                             // 1 - charger is always connected to HV battery // 0 - Charger is behind the contactors
  settings.TempOff = -52;                                 // Temperature offset
  settings.tripcont = 1;                                  // in ESSmode 1 - Main contactor function, 0 - Trip function
  settings.triptime = 5000;                               // mS of delay before counting over or undervoltage
  settings.TempConv = 0.0038;                             // Temperature scale
  settings.chargecurrentcold = 1;                         // Max allowed charging current below under temperature
  settings.numberOfChargers = 1;                          // The number of chargers.
  settings.numberOfModules = 12;                          // Number of modules in the battery pack
  settings.inverterTempSetpoint = 40;                     //Inverter temp at which to turn on the fan
  settings.motorTempSetpoint = 40;                        // Motor temp at which to turn on the fan
  settings.inverterTempHys = 5;                           // Hysteresis buffer for inverter temp (for fan control)
  settings.motorTempHys = 5;                              // Hysteresis buffer for motor temp (for fan control)
}

void setup()
{
  Serial.begin(115200);
  // SERIALCONSOLE.begin(115200);
  Serial.println("Starting up!");
  Serial.println("SimpBMS V2 Outlander");
  delay(2000); // just for easy debugging. It takes a few seconds for USB to come up properly on most OS's

  pinMode(PP_DETECT, INPUT);

  pinMode(CRANK_IN, INPUT);
  digitalWrite(CRANK_IN, LOW);

  pinMode(OUT_FAN, OUTPUT); // fan relay
  digitalWrite(OUT_FAN, LOW); // disable by default

  pinMode(OUT_NEG_CONTACTOR, OUTPUT); // negative contactor
  digitalWrite(OUT_NEG_CONTACTOR, LOW); // disable by default

  pinMode(OUT_DCDC_ENABLE, OUTPUT);    // DCDC enable
  digitalWrite(OUT_DCDC_ENABLE, LOW); // enable by when main contactor is closed

  pinMode(OUT_DCDC_ENABLE, OUTPUT);    // DCDC enable

  pinMode(FUEL_GAUGE_PWM, OUTPUT);

  pinMode(led, OUTPUT);

  analogReadResolution(12);  // Set ADC resolution to 12 bits
  analogSetAttenuation(ADC_11db);  // Set attenuation (0db, 2.5db, 6db, 11db)
  // // enable WDT
  // noInterrupts();  // don't allow interrupts while setting up WDOG

  // static esp_task_wdt_config_t wdt_config = {
  //     .timeout_ms = WDT_TIMEOUT * 1000,  // Convert seconds to milliseconds
  //     // .idle_timeout_ms = (CONFIG_ESP_TASK_WDT_IDLE_TIMEOUT * 1000),  // Optional, set idle timeout (see ESP-IDF docs)
  //     // .auto_restart = true,
  // };

  // esp_task_wdt_init(&wdt_config);

  // esp_task_wdt_add(NULL);               // add current thread to WDT watch
  // interrupts();
  /////////////////
  EEPROM.begin(sizeof(settings));
  EEPROM.get(0, settings);
  if (settings.version != EEPROM_VERSION)
  {
    Serial.print("Version");
    Serial.println(settings.version);
    Serial.println();
    loadSettings();
  }


  SPI.begin(MCP2515_SCK, MCP2515_MISO, MCP2515_MOSI, MCP2515_CS);
  bmscan.can1 = new ACAN2515(MCP2515_CS, SPI, MCP2515_INT);
  ACAN2515Settings cansettings(16 * 1000 * 1000, 500000);
  cansettings.mRequestedMode = ACAN2515Settings::LoopBackMode;

  const ACAN2515Mask rxm0 = standard2515Mask(0xFFF, 0xFF, 0); // For filter #0 and #1
  const ACAN2515Mask rxm1 = standard2515Mask(0x000, 0, 0); // For filter #2 to #
  const ACAN2515AcceptanceFilter filters[] = {
      {standard2515Filter(0x010, 0, 0), receivedFiltered}, // VCU
       {standard2515Filter(0x200, 0, 0), receivedFiltered},
       {standard2515Filter(0x520, 0, 0), receivedFiltered} // ISA shunt
      // {standard2515Filter(0x423, 0, 0), receivedFiltered}, // ISA shunt
      // {standard2515Filter(0x600, 0, 0), receivedFiltered} // outlander pack 2
  };

  bmscan.can1->begin(
      cansettings, []
      { bmscan.can1->isr(); },
      rxm0, rxm1, filters, 3);

  bmscan.begin(500000, settings.chargerCanIndex);//can1
  bmscan.begin(500000, settings.veCanIndex); // can0

  // Initialize SPIFFS
  if (!SPIFFS.begin(true)) {
    Serial.println("SPIFFS Mount Failed, Formatting...");
    if (SPIFFS.format()) {
      Serial.println("SPIFFS Formatted Successfully");
    } else {
      Serial.println("SPIFFS Format Failed");
    }
  } else {
    Serial.println("SPIFFS Mounted Successfully");
  }

  WiFi.setHostname(HOSTNAME);
  WiFi.hostname(HOSTNAME);

  //AP and Station Mode
  WiFi.mode(WIFI_AP_STA);

  WiFi.softAP("BALLSBMS", "spaceballs"); // Create access point
  //Connect to Wi-Fi
  WiFi.begin("BT-JNF6TR", "qauFKtE7GVRPMh");
  // WiFi.begin();

  Serial.println(WiFi.localIP());
  Serial.println(WiFi.softAPIP());

  bmsWebServer.setup();
  digitalWrite(led, HIGH);
}

static void receivedFiltered(const CANMessage &inMsg)
{
  /*This does hardware filtering of can messages to avoid
  processing them in softare. This can / should be made generic.*/
  /*SK THIS IS TAKEN FROM BMW CODE -- NEEDS TO BE CHANGED TO OUTLANDER*/
  printf("candebug %d",candebug);
  if (candebug == 1)
  {
    Serial.print(millis());
    if ((inMsg.id & 0x80000000) == 0x80000000) // Determine if ID is standard (11 bits) or extended (29 bits)
      sprintf(msgString, "Extended ID: 0x%.8lX  DLC: %1d  Data:", (inMsg.id & 0x1FFFFFFF), inMsg.len);
    else
      sprintf(msgString, ",0x%.3lX,false,%1d", inMsg.id, inMsg.len);

    Serial.print(msgString);

    Serial.println(" Filtered Can ");
  }

  if (inMsg.id == 0x527)
  {
    long ampseconds = inMsg.data[2] + (inMsg.data[3] << 8) + (inMsg.data[4] << 16) + (inMsg.data[5] << 24);
    amphours = ampseconds / 3600.0f;
  }
  else if (inMsg.id == 0x521)
  {
    CANmilliamps = inMsg.data[2] + (inMsg.data[3] << 8) + (inMsg.data[4] << 16) + (inMsg.data[5] << 24);
    RawCur = CANmilliamps;
    getcurrent();
  }
  else if (inMsg.id == 0x522)
  {
    voltage1 = inMsg.data[2] + (inMsg.data[3] << 8) + (inMsg.data[4] << 16) + (inMsg.data[5] << 24);
  }
  else if (inMsg.id == 0x523)
  {
    voltage2 = inMsg.data[2] + (inMsg.data[3] << 8) + (inMsg.data[4] << 16) + (inMsg.data[5] << 24);
  }
  else if (inMsg.id == 0x526)
  {
    long watt = inMsg.data[2] + (inMsg.data[3] << 8) + (inMsg.data[4] << 16) + (inMsg.data[5] << 24);
    kilowatts = watt / 1000.0f;
  }
  else if (inMsg.id == 0x527)
  {
    long ampseconds = inMsg.data[2] + (inMsg.data[3] << 8) + (inMsg.data[4] << 16) + (inMsg.data[5] << 24);
    amphours = ampseconds / 3600.0f;
  }
  else if (inMsg.id == 0x528)
  {
    long wh = inMsg.data[2] + (inMsg.data[3] << 8) + (inMsg.data[4] << 16) + (inMsg.data[5] << 24);
    kilowatthours = wh / 1000.0f;
  }
  else if (inMsg.id == 0x01)
  {
    inverterTemp    = inMsg.data[0] | (inMsg.data[1] << 8);
    motorTemp       = inMsg.data[2] | (inMsg.data[3] << 8);
    driveMode       = int(inMsg.data[4]);  // 0: park, 1: reverse, 2: neutral, 3: drive
    prechargeContactor = int(inMsg.data[5]); // 0: open, 1: closed
    mainContactor      = int(inMsg.data[6]); // 0: open, 1: closed
    hvstatustimer = millis();
    printf("0x01 received");
  }
  // chademo
  else if (inMsg.id == 0x354 && inMsg.data[0] == 0x01)
  {
    rapidCharging = true;
  }
  else if (inMsg.id == 0x52A)
  { // param set value
    if (inMsg.data[0] == 0x01)
    { // set ac current max
      settings.chargecurrentmax = inMsg.data[1] * 10;
      Serial.print("Setting max current ");
      Serial.println(settings.chargecurrentmax);
    }
  }
  else
  {
    BMS_CAN_MESSAGE modifiedMessage = bmscan.convert(inMsg);
    charger.handleIncomingCAN(modifiedMessage);
    evse_duty = charger.evse_duty; // HACK to make the Web service run

    bms.handleIncomingCAN(modifiedMessage);
  }
}

void loop()
{

  // int ppValue = analogRead(ppInputPin);
  // Serial.print("PP Analog Value: ");
  // Serial.print(ppValue);

  canread(DEFAULT_CAN_INTERFACE_INDEX);

  //bmscan.can1->dispatchReceivedMessage();

   if (crankSeen == false){
     if (digitalRead(CRANK_IN) == LOW){
       digitalWrite(OUT_NEG_CONTACTOR, HIGH);
       digitalWrite(OUT_DCDC_ENABLE, HIGH);

       crankSeen = true;
     }
   }

  //When there's PP detect and the mid-pack contactor is open, close it, and enable DCDC
  if (digitalRead(PP_DETECT) == LOW && digitalRead(OUT_NEG_CONTACTOR) == LOW){
    digitalWrite(OUT_NEG_CONTACTOR, HIGH);
    digitalWrite(OUT_DCDC_ENABLE, HIGH);

  }

  switch (bmsstatus)
  {
  case (Boot):
    Discharge = 0;
    bmsstatus = Ready;
    break;

  case (Ready):
    Discharge = 0;
    if (checkPPDetect(1) && (bms.getHighCellVolt() < (settings.ChargeVsetpoint - settings.ChargeHys))) // detect AC present for charging and check not balancing
    {
      if (negContactorClosed())
      {
        bmsstatus = Charge;
      }
      else
      {
        bmsstatus = Precharge;
      }
    }
    if (carInDrive())
    {
      if (prechargeContactor == 1)
      {
        bmsstatus = Drive;
      }
      else
      {
        bmsstatus = Precharge;
      }
    }
    if (rapidCharging)
    {
      if (negContactorClosed())
      {
        bmsstatus = RapidCharge;
      }
      else
      {
        bmsstatus = Precharge;
      }
    }

    break;

  case (Precharge):
    Discharge = 0;
    if (!rapidCharging && negContactorClosed() && checkPPDetect(2))
    {
      bmsstatus = Charge;
    }
    if (!rapidCharging && negContactorClosed() && carInDrive())
    {
      bmsstatus = Drive;
    }
    if (rapidCharging && negContactorClosed())
    {
      bmsstatus = RapidCharge;
    }
    break;

  case (Drive):
    Discharge = 1;
    if (!carInDrive()) // Key OFF
    {
      bmsstatus = Ready;
    }
    //don't allow charging while the car is in 'D' or 'R'
    if (carInDrive() && checkPPDetect(3)) // detect AC present for charging and check not balancing
    {
      printf("ERROR: Car in Drive and PP Detected. ");
      bmsstatus = Error;
    }
  
    //If the car's in drive, and the motor or inverter are getting too hot
    //turn on the fan.
    if (inverterTemp > settings.inverterTempSetpoint ||
        motorTemp > settings.motorTempSetpoint){
      digitalWrite(OUT_FAN, HIGH);
    }

    //avoid hysteresis by setting the off signal to be lower than
    //the on signal.
    if (inverterTemp <= settings.inverterTempSetpoint - settings.inverterTempHys  ||
        motorTemp <= settings.motorTempSetpoint - settings.motorTempHys){
      digitalWrite(OUT_FAN, LOW);
    }

    break;

  case (Charge):
    Discharge = 0;
    digitalWrite(OUT_FAN, HIGH); // enable fan
    if (bms.getHighCellVolt() > settings.ChargeVsetpoint)
    {
      bmsstatus = Ready;
    }
    if (rapidCharging)
    {
      bmsstatus = RapidCharge;
    }
    if (!checkPPDetect(4) || !negContactorClosed()) // detect AC not present for charging or inverter not closed the contactors
    {
      // send a 0 amp request to charger
      chargecurrent = 0;
      charger.sendChargeMsg(msg, chargecurrent);
      bmsstatus = Ready;
    }
    break;
  case (RapidCharge):
    digitalWrite(OUT_FAN, LOW); // disable fan
    break;
  case (Error):
    Discharge = 0;
    break;
  }

  if (millis() - looptime > 500)
  {
    if ((bms.getHighCellVolt() > 0 || bms.getLowCellVolt() > 0 )&& (bms.getLowCellVolt() < settings.UnderVSetpoint || bms.getHighCellVolt() < settings.UnderVSetpoint))
    {
      if (UnderTime < millis()) // check is last time not undervoltage is longer thatn triptime ago
      {
        printf("ERROR: Under Voltage. Low: %.2f, High: %.2f < Setpoint: %.2f",
          (float) bms.getLowCellVolt(),
          (float) bms.getHighCellVolt(),
          (float) settings.UnderVSetpoint);

        bmsstatus = Error;
        ErrorReason = ErrorReason | 0x02;
      }
    }
    else
    {
      UnderTime = millis() + settings.triptime;
      ErrorReason = ErrorReason & ~0x02;
    }
    if (bms.getHighCellVolt() > 0 && bms.getHighCellVolt() > settings.OverVSetpoint)
    {
      if (OverTime < millis()) // check is last time not undervoltage is longer thatn triptime ago
      {
        printf("ERROR: Over Voltage.  High: %.2f > Setpont %.2f", (float) bms.getHighCellVolt(), (float) settings.OverVSetpoint);
        bmsstatus = Error;
        ErrorReason = ErrorReason | 0x01;
      }
    }
    else
    {
      OverTime = millis() + settings.triptime;
      ErrorReason = ErrorReason & ~0x01;
    }

    currentlimit();
    alarmupdate();
    resetwdog();

    looptime = millis();
  }
  if (millis() - chargerSpeedLoopTime > settings.chargerspd)
  {
    chargerSpeedLoopTime = millis();

    if (bmsstatus == Charge)
    {
      charger.sendChargeMsg(msg, chargecurrent);
    }
  }

  if (millis() - bmsSpeedLoopTime > settings.bmsspeed)
  {
    bmsSpeedLoopTime = millis();

    bms.sendKeepAliveFrame(msg, bmsstatus);

    sendStatusMessages();

  }

  if (millis() - hvstatustimer > 1000)
  {
    /*If haven't received a message for 1 second, assume the HV is not available*/
    prechargeContactor = 0; // open
    mainContactor = 0; // open
  }

  if (millis() - statustimer > 5000)
  {
    statustimer = millis();
    printbmsstat();
  }

  bmsWebServer.execute();

  analogWrite(FUEL_GAUGE_PWM, map(bms.stateOfCharge, 0, 100, 0, 255));
} // end loop()

void alarmupdate()
{
  bmsAlarm[0] = 0x00;
  if (settings.OverVSetpoint < bms.getHighCellVolt())
  {
    bmsAlarm[0] = 0x04;
  }
  if (bms.getLowCellVolt() < settings.UnderVSetpoint)
  {
    bmsAlarm[0] |= 0x10;
  }
  if (bms.getHighTemperature() > settings.OverTSetpoint)
  {
    bmsAlarm[0] |= 0x40;
  }
  bmsAlarm[1] = 0;
  if (bms.getLowTemperature() < settings.UnderTSetpoint)
  {
    bmsAlarm[1] = 0x01;
  }
  bmsAlarm[3] = 0;
  if ((bms.getHighCellVolt() - bms.getLowCellVolt()) > settings.CellGap)
  {
    bmsAlarm[3] = 0x01;
  }

  /// warnings///
  bmsWarning[0] = 0;

  if (bms.getHighCellVolt() > (settings.OverVSetpoint - settings.WarnOff))
  {
    bmsWarning[0] = 0x04;
  }
  if (bms.getLowCellVolt() < (settings.UnderVSetpoint + settings.WarnOff))
  {
    bmsWarning[0] |= 0x10;
  }

  if (bms.getHighTemperature() > (settings.OverTSetpoint - settings.WarnToff))
  {
    bmsWarning[0] |= 0x40;
  }
  bmsWarning[1] = 0;
  if (bms.getLowTemperature() < (settings.UnderTSetpoint + settings.WarnToff))
  {
    bmsWarning[1] = 0x01;
  }
}

void printbmsstat()
{

  bms.printData();

  if (bmsstatus == Charge)
  {
    charger.printChargerStatus();
  }

}

void getcurrent()
{

  if (settings.invertcur == 1)
  {
    RawCur = RawCur * -1;
  }

  currentact = settings.ncur * RawCur;

  RawCur = 0;

}

void sendStatusMessages() // communication with Victron system over CAN
{
  /*Send status messages over can*/

    /*Send status messages over can*/
  delay(2);
  msg.id = 0x011;
  msg.len = 8;
  msg.buf[0] = 0;
  msg.buf[1] = 0;
  msg.buf[2] = 0;
  msg.buf[3] = 0;
  msg.buf[4] = 0;
  msg.buf[5] = 0;
  msg.buf[6] = 0;
  msg.buf[7] = 0;
  bmscan.write(msg, settings.chargerCanIndex);


  delay(2);
  msg.id = 0x351;
  msg.len = 8;
  msg.buf[0] = lowByte(uint16_t((settings.ChargeVsetpoint * settings.Scells) * 10));
  msg.buf[1] = highByte(uint16_t((settings.ChargeVsetpoint * settings.Scells) * 10));
  msg.buf[2] = lowByte(chargecurrent);
  msg.buf[3] = highByte(chargecurrent);
  msg.buf[4] = lowByte(discurrent);
  msg.buf[5] = highByte(discurrent);
  msg.buf[6] = lowByte(uint16_t((settings.DischVsetpoint * settings.Scells) * 10));
  msg.buf[7] = highByte(uint16_t((settings.DischVsetpoint * settings.Scells) * 10));
  bmscan.write(msg, settings.veCanIndex);

  // delay(2);
  // msg.id = 0x355;
  // msg.len = 8;
  // msg.buf[0] = lowByte(bms.getSOC());
  // msg.buf[1] = highByte(bms.getSOC());
  // msg.buf[2] = lowByte(bms.getSOH());
  // msg.buf[3] = highByte(bms.getSOH());
  // msg.buf[4] = lowByte(bms.getSOC() * 10);
  // msg.buf[5] = highByte(bms.getSOC() * 10);

  // // Send Charge if in Precharge for VCU
  // if (bmsstatus == Precharge)
  // {
  //   msg.buf[6] = lowByte(Charge);
  //   msg.buf[7] = highByte(Charge);
  // }
  // else
  // {
  //   msg.buf[6] = lowByte(bmsstatus);
  //   msg.buf[7] = highByte(bmsstatus);
  // }

  // bmscan.write(msg, settings.veCanIndex);
  // delay(2);
  // msg.id = 0x356;
  // msg.len = 8;
  // msg.buf[0] = lowByte(uint16_t(bms.getPackVoltage() * 100));
  // msg.buf[1] = highByte(uint16_t(bms.getPackVoltage() * 100));
  // msg.buf[2] = lowByte(long(currentact / 100));
  // msg.buf[3] = highByte(long(currentact / 100));
  // msg.buf[4] = lowByte(int16_t(bms.getAvgTemperature() * 10));
  // msg.buf[5] = highByte(int16_t(bms.getAvgTemperature() * 10));
  // msg.buf[6] = lowByte(uint16_t(bms.getAvgCellVolt() * 1000));
  // msg.buf[7] = highByte(uint16_t(bms.getAvgCellVolt() * 1000));
  // bmscan.write(msg, settings.veCanIndex);

  // delay(2);
  // msg.id = 0x35A;
  // msg.len = 8;
  // msg.buf[0] = bmsAlarm[0];   // High temp  Low Voltage | High Voltage
  // msg.buf[1] = bmsAlarm[1];   // High Discharge Current | Low Temperature
  // msg.buf[2] = bmsAlarm[2];   // Internal Failure | High Charge current
  // msg.buf[3] = bmsAlarm[3];   // Cell Imbalance
  // msg.buf[4] = bmsWarning[0]; // High temp  Low Voltage | High Voltage
  // msg.buf[5] = bmsWarning[1]; // High Discharge Current | Low Temperature
  // msg.buf[6] = bmsWarning[2]; // Internal Failure | High Charge current
  // msg.buf[7] = bmsWarning[3]; // Cell Imbalance
  // bmscan.write(msg, settings.veCanIndex);

  // delay(2);
  // msg.id = 0x373;
  // msg.len = 8;
  // msg.buf[0] = lowByte(uint16_t(bms.getLowCellVolt() * 1000));
  // msg.buf[1] = highByte(uint16_t(bms.getLowCellVolt() * 1000));
  // msg.buf[2] = lowByte(uint16_t(bms.getHighCellVolt() * 1000));
  // msg.buf[3] = highByte(uint16_t(bms.getHighCellVolt() * 1000));
  // msg.buf[4] = lowByte(uint16_t(bms.getLowTemperature() + 273.15));
  // msg.buf[5] = highByte(uint16_t(bms.getLowTemperature() + 273.15));
  // msg.buf[6] = lowByte(uint16_t(bms.getHighTemperature() + 273.15));
  // msg.buf[7] = highByte(uint16_t(bms.getHighTemperature() + 273.15));
  // bmscan.write(msg, settings.veCanIndex);

  // delay(2);
  // msg.id = 0x379; // Installed capacity
  // msg.len = 4;
  // msg.buf[0] = lowByte(settings.CAP);
  // msg.buf[1] = highByte(settings.CAP);
  // msg.buf[2] = lowByte(uint16_t(amphours));
  // msg.buf[3] = highByte(uint16_t(amphours));
  // // (remaining amp hours * 1.2) * 10
  // uint16_t rangeEst = (settings.CAP - amphours) * 12;
  // msg.buf[4] = lowByte(rangeEst);
  // msg.buf[5] = highByte(rangeEst);
  // bmscan.write(msg, settings.veCanIndex);

  // delay(2);
  // msg.id = 0x372;
  // msg.len = 8;
  // msg.buf[0] = lowByte(bms.getNumModules());
  // msg.buf[1] = highByte(bms.getNumModules());
  // msg.buf[2] = 0x00;
  // msg.buf[3] = 0x00;
  // msg.buf[4] = 0x00;
  // msg.buf[5] = 0x00;
  // msg.buf[6] = 0x00;
  // msg.buf[7] = 0x00;
  // bmscan.write(msg, settings.veCanIndex);
}

// ID offset is only applied to battery module frames
void canread(int canInterfaceOffset)
{

  while (bmscan.read(inMsg, canInterfaceOffset))
  {
    if (candebug == 1)
    {
      Serial.print(millis());
      if ((inMsg.id & 0x80000000) == 0x80000000) // Determine if ID is standard (11 bits) or extended (29 bits)
        sprintf(msgString, "Extended ID: 0x%.8lX  DLC: %1d  Data:", (inMsg.id & 0x1FFFFFFF), inMsg.len);
      else
        sprintf(msgString, ",0x%.3lX,false,%1d", inMsg.id, inMsg.len);

      Serial.print(msgString);

      if ((inMsg.id & 0x40000000) == 0x40000000)
      { // Determine if message is a remote request frame.
        sprintf(msgString, " REMOTE REQUEST FRAME");
        // Serial.print(msgString);
      }
      else
      {
        for (uint8_t i = 0; i < inMsg.len; i++)
        {
          sprintf(msgString, ", 0x%.2X", inMsg.buf[i]);
          // Serial.print(msgString);
        }
      }
      Serial.print(" Can Interface: ");
      Serial.print(canInterfaceOffset);
      Serial.println();
    }
    bms.handleIncomingCAN(inMsg);
  }
}

void currentlimit()
{
  if (bmsstatus == Error)
  {
    discurrent = 0;
    chargecurrent = 0;
  }
  else
  {
    /// Start at no derating///
    discurrent = settings.discurrentmax;
    int maxchargingcurrent;
    if (bmsstatus == RapidCharge)
    {
      maxchargingcurrent = chargecurrent = settings.rapidchargecurrentmax;
    }
    else
    {
      maxchargingcurrent = chargecurrent = settings.chargecurrentmax;
    }

    ///////All hard limits to into zeros
    if (bms.getLowTemperature() < settings.UnderTSetpoint)
    {
      // discurrent = 0; Request Daniel
      chargecurrent = settings.chargecurrentcold;
    }
    if (bms.getHighTemperature() > settings.OverTSetpoint)
    {
      discurrent = 0;
      chargecurrent = 0;
    }
    if (bms.getHighCellVolt() > settings.OverVSetpoint)
    {
      chargecurrent = 0;
    }
    if (bms.getLowCellVolt() < settings.UnderVSetpoint || bms.getLowCellVolt() < settings.DischVsetpoint)
    {
      discurrent = 0;
    }

    // Modifying discharge current///
    if (discurrent > 0)
    {
      // Temperature based///
      if (bms.getHighTemperature() > settings.DisTSetpoint)
      {
        discurrent = discurrent - map(bms.getHighTemperature(), settings.DisTSetpoint, settings.OverTSetpoint, 0, settings.discurrentmax);
      }
      // Voltagee based///
      if (bms.getLowCellVolt() < (settings.DischVsetpoint + settings.DisTaper))
      {
        discurrent = discurrent - map(bms.getLowCellVolt(), settings.DischVsetpoint, (settings.DischVsetpoint + settings.DisTaper), settings.discurrentmax, 0);
      }
    }

    // Modifying Charge current///
    if (chargecurrent > settings.chargecurrentcold)
    {
      // Temperature based///
      if (bms.getLowTemperature() < settings.ChargeTSetpoint)
      {
        chargecurrent = chargecurrent - map(bms.getLowTemperature(), settings.UnderTSetpoint, settings.ChargeTSetpoint, (maxchargingcurrent - settings.chargecurrentcold), 0);
      }
      // Voltagee based///
      if (storagemode == 1)
      {
        if (bms.getHighCellVolt() > (settings.StoreVsetpoint - settings.ChargeHys))
        {
          chargecurrent = chargecurrent - map(bms.getHighCellVolt(), (settings.StoreVsetpoint - settings.ChargeHys), settings.StoreVsetpoint, settings.chargecurrentend, maxchargingcurrent);
        }
      }
      else
      {
        if (bms.getHighCellVolt() > (settings.ChargeVsetpoint - settings.ChargeHys))
        {
          chargecurrent = chargecurrent - map(bms.getHighCellVolt(), (settings.ChargeVsetpoint - settings.ChargeHys), settings.ChargeVsetpoint, 0, (maxchargingcurrent - settings.chargecurrentend));
        }
      }
    }
  }

  // extra safety check
  if (settings.chargertype == Outlander)
  {
    uint16_t fullVoltage = uint16_t(settings.ChargeVsetpoint * settings.Scells * settings.numberOfModules);
    if (charger.reported_voltage > fullVoltage)
    {
      chargecurrent = 0;
    }
  }

  /// No negative currents///

  if (discurrent < 0)
  {
    discurrent = 0;
  }
  if (chargecurrent < 0)
  {
    chargecurrent = 0;
  }
}

void resetISACounters() {
  msg.id  = 0x411;
  msg.len = 8;
  msg.buf[0] = 0x3F;
  msg.buf[1] = 0x00;
  msg.buf[2] = 0x00;
  msg.buf[3] = 0x00;
  msg.buf[4] = 0x00;
  msg.buf[5] = 0x00;
  msg.buf[6] = 0x00;
  msg.buf[7] = 0x00;
  bmscan.write(msg, settings.veCanIndex);
}

void resetwdog()
{
  // noInterrupts(); //   No - reset WDT
  // esp_task_wdt_reset();
  // interrupts();
}
////////END///////////
