#pragma once
#include <FlexCAN.h>

class BMSModule
{
  public:
    BMSModule();
    void decodecan(int Id, CAN_message_t &msg);
    void readStatus();
    void clearmodule();
    int getscells();
    bool readModuleValues();
    float getCellVoltage(int cell);
    float getLowCellV();
    float getHighCellV();
    float getAverageV();
    float getLowTemp();
    float getHighTemp();
    float getHighestModuleVolt();
    float getLowestModuleVolt();
    float getHighestCellVolt(int cell);
    float getLowestCellVolt(int cell);
    float getHighestTemp();
    float getLowestTemp();
    float getAvgTemp();
    float getModuleVoltage();
    float getTemperature(int temp);
    uint8_t getFaults();
    uint8_t getAlerts();
    uint8_t getCOVCells();
    uint8_t getCUVCells();
    void setAddress(int newAddr);
    int getAddress();
    bool isExisting();
    void setExists(bool ex);
    bool isReset();
    void setReset(bool ex);
    void settempsensor(int tempsensor);
    void setIgnoreCell(float Ignore);
    void setTempconv(float tempconvin, int tempoffin);
    uint8_t getBalStat();



  private:
    float cellVolt[8];          // calculated as 16 bit value * 6.250 / 16383 = volts
    float lowestCellVolt[8];
    float highestCellVolt[8];
    float moduleVolt;      // calculated as 16 bit value * 33.333 / 16383 = volts
    float temperatures[3];     // Don't know the proper scaling at this point
    float lowestTemperature;
    float highestTemperature;
    float lowestModuleVolt;
    float highestModuleVolt;
    float IgnoreCell;
    bool exists;
    bool reset;
    int alerts;
    int faults;
    int COVFaults;
    int CUVFaults;
    int sensor;
    uint8_t moduleAddress; // 1 to 0x3E
    int scells;
    uint8_t balstat;
    uint32_t lasterror;
    uint8_t cmuerror;
    int16_t TempOff;
    uint16_t timeout;
    float tempconv;
    int tempoff;
};
