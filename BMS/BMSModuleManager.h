#pragma once
#include "config.h"
#include "BMSModule.h"
// #include <FlexCAN.h>
#include "BMSCan.h"
#include <ArduinoJson.h>

class BMSModuleManager
{
public:
  BMSModuleManager();
  int seriescells();
  void clearmodules();
  void decodecan(BMS_CAN_MESSAGE &msg, int canChannel, int debug);
  void decodetemp(BMS_CAN_MESSAGE &msg, int canChannel, int debug);
  void balanceCells();
  void setupBoards();
  bool checkcomms();
  bool checkstatus();
  void findBoards();
  void renumberBoardIDs();
  void clearFaults();
  void sleepBoards();
  void wakeBoards();
  void getAllVoltTemp();
  void readSetpoints();
  void setBatteryID(int id);
  void setBalIgnore(bool BalIgn);
  void setPstrings(int Pstrings);
  void setUnderVolt(float newVal);
  void setOverVolt(float newVal);
  void setOverTemp(float newVal);
  void setBalanceV(float newVal);
  void setBalanceHyst(float newVal);
  void setSensors(int sensor, float Ignore,float tempconvin, int tempoffin);
  float getPackVoltage();
  float getAvgTemperature();
  float getHighTemperature();
  float getLowTemperature();
  float getAvgCellVolt();
  float getLowCellVolt();
  float getHighCellVolt();
  float getHighVoltage();
  float getLowVoltage();
  int getBalancing();
  void resetLowCellV();
  /*
    void processCANMsg(CAN_FRAME &frame);
  */
  void printAllCSV(unsigned long timestamp, float current, int SOC);
  void printPackSummary();
  void printPackDetails(int digits,bool showbal);
  void printPackDetailsJson(DynamicJsonDocument &root);
  int getNumModules();

private:
  float packVolt; // All modules added together
  bool BalIgnore;
  int Pstring;
  float avg;
  float avgsmooth;
  float LowCellVolt;
  float LowCellVoltsmooth;
  float HighCellVolt;
  float HighCellVoltsmooth;
  float MeasurementStep;
  float lowestPackVolt;
  float highestPackVolt;
  float lowestPackTemp;
  float highestPackTemp;
  float highTemp;
  float lowTemp;
  BMSModule modules[MAX_MODULE_ADDR + 1]; // store data for as many modules as we've configured for.
  int batteryID;
  int numFoundModules; // The number of modules that seem to exist
  int numFoundModulesOLD;
  bool isFaulted;
  int spack;
  float ignorevolt;
  int tempsens;
  float tempconv;
  int tempoff;
  float avgcell[8];
  float avgtotal;
  int avgindex;
  float lowcell[8];
  float lowtotal;
  int lowindex;
  float highcell[8];
  float hightotal;
  int highindex;
  int CellsBalancing;
  /*
    void sendBatterySummary();
    void sendModuleSummary(int module);
    void sendCellDetails(int module, int cell);
  */
};
