#pragma once
#include <iostream>
using namespace std;
#include "config.h"
#include <ArduinoJson.h>  
#include "BMSCan.h"

class KangooCan
{
  public:
    KangooCan(BMSCan &bmscan, EEPROMSettings &settings);
    void sendKeepAliveFrame(BMS_CAN_MESSAGE &msg, uint8_t &status);
    void sendIsoTpMessage(const uint8_t *data, uint8_t length);
    // v  oid receiveISOTPMessage(uint32_t expectedCanId, uint8_t *data, uint8_t &length);
    bool ProcessISOTPResponse(const uint8_t* data);
    bool handlePID61Frame(const uint8_t* data);
    bool handlePID66Frame(const uint8_t* data);
    bool handlePID03Frame(const uint8_t* data);
    bool handlePID04Frame(const uint8_t* data);
    bool handlePID41Frame(const uint8_t* data);
    bool handlePID42Frame(const uint8_t* data);
    bool handlePID01Frame(const uint8_t* data);

    void handleFrame155(const uint8_t* data);
    void handleFrame424(const uint8_t* data);
    void handleFrame425(const uint8_t* data);

    uint8_t getLowestCellTemp();
    uint8_t getHighestCellTemp();
    float getPackVoltage();
    int16_t getAvgTemperature();
    int16_t getHighTemperature();
    int16_t getLowTemperature();
    float getAvgCellVolt();
    float getLowCellVolt();
    float getHighCellVolt();
    uint16_t getHighVoltage();
    uint16_t   getLowVoltage();
    float getSOC();
    float getSOH();
    uint8_t getNumModules();
    void printPackDetailsJson(JsonDocument &root);


    void printData();

    void handleIncomingCAN(BMS_CAN_MESSAGE &inMsg);

    //data from frames
    int16_t batteryTemperature;
    uint32_t ampHoursRaw;
    uint16_t packHealthRaw;
    uint16_t quickchargeCount;
    uint16_t normalchargeCount;
    uint16_t fullchargeCount;

    uint16_t partialchargeCount;
    uint32_t totalKilometers;
    uint32_t lowestMilV;
    uint32_t highestMilV;
    uint16_t maxChargingRaw;
    uint16_t cellVoltages[96];
    uint16_t fullPackVoltage; //ISOTP
    uint16_t daigPackVoltage; //ISOTP
    uint16_t maxInputPowerRaw;
    uint16_t maxOutputPowerRaw;
    int32_t batteryCurrent;
    int32_t numModules;


    //data from free can messages
    float stateOfCharge;
    float stateOfHealth;
    float lowestCellVolt;
    float highestCellVolt;
    float remainingKHW;
    float packVoltage;
    float current;

    uint8_t lowestCellTemp;
    uint8_t highestCellTemp;
    uint16_t maxCharging;
    uint16_t maxInputPower;
    uint16_t maxOutputPower;

  private:
    BMSCan bmscan;
    EEPROMSettings settings;
    uint8_t currentPID;
    uint16_t byteCount;
    uint16_t remainingByteCount;
    bool looped;  
};
