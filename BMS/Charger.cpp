#include <Arduino.h>
#include "BMSCan.h"
#include <ACAN_ESP32.h>
#include <ACAN2515.h>
#include "Charger.h"

Charger::Charger(BMSCan& b, EEPROMSettings& s) : bmscan(b), settings{s}
{
}

void Charger::sendChargeMsg(BMS_CAN_MESSAGE &msg, int &chargecurrent)
{
//pass
}
