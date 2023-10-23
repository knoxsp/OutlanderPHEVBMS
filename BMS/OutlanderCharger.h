#pragma once
// #include <FlexCAN.h>
#include "config.h"

//TODO: How to subclass the charger class? 
class OutlanderCharger
{
  public:
    OutlanderCharger(BMSCan& bmscan, EEPROMSettings& settings);
    void sendChargeMsg(BMS_CAN_MESSAGE &msg, int &chargecurrent);
    void printChargerStatus();
    int reported_voltage;
    int reported_current;
    int reported_temp1;
    int reported_temp2;
    byte reported_status;
    byte evse_duty;
    void handleIncomingCAN(BMS_CAN_MESSAGE &inMsg);
  private:
    BMSCan bmscan;
    EEPROMSettings settings;
};
