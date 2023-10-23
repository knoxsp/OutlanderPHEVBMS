#pragma once
// #include <FlexCAN.h>

#include "BMSCan.h"
#include "config.h"
/*A generic class to be inherited which allows an interface to 
different types of chargers
*/
class Charger
{
public:
  Charger(BMSCan &bmscan, EEPROMSettings &settings);
  //Send CAN message to the charger to charge at the requested charge current.
  void sendChargeMsg(BMS_CAN_MESSAGE &msg, int &chargecurrent);
  

private:
  BMSCan bmscan;
  EEPROMSettings settings;
};
