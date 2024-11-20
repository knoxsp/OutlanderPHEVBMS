#include <Arduino.h>
#include "BMSCan.h"
#include <ACAN_ESP32.h>
#include <ACAN2515.h>
#include "OutlanderCharger.h"
#include "config.h"


OutlanderCharger::OutlanderCharger(BMSCan &b, EEPROMSettings &s) : bmscan(b), settings{s}
{
    printf("Initializing charger: number of chargers: %d \n", settings.numberOfChargers);

}


void OutlanderCharger::handleIncomingCAN(BMS_CAN_MESSAGE &inMsg)
{
  if (inMsg.id == 0x389)
  {
    this->reported_voltage = inMsg.buf[0] * 2;
    this->reported_current = inMsg.buf[2];
    this->reported_temp1 = inMsg.buf[3] - 40;
    this->reported_temp2 = inMsg.buf[4] - 40;
  }
  else if (inMsg.id == 0x38A)
  {
    this->reported_status = inMsg.buf[4];
    this->evse_duty = inMsg.buf[3];
  }
}

void OutlanderCharger::printChargerStatus()
{
    printf("Outlander Charger - Reported Voltage: %d\n", this->reported_voltage);
    printf("V Reported Current: %d\n", this->reported_current / 10);
    printf("A Reported Temp1: %d\n", this->reported_temp1);
    printf("C Reported Temp2: %d\n", this->reported_temp2);
    printf("C status: %s\n", this->reported_status == 0 ? "Not Charging" :
                             this->reported_status == 0x04 ? "Wait for Mains" :
                             this->reported_status == 0x08 ? "Ready/Charging" : "Unknown");
}

void OutlanderCharger::sendChargeMsg(BMS_CAN_MESSAGE &msg, int &chargecurrent)
{
    printf("Sendig charge Msg");
    msg.id = 0x285;
    msg.len = 8;
    msg.buf[0] = 0x0;
    msg.buf[1] = 0x0;
    msg.buf[2] = 0xb6;
    msg.buf[3] = 0x0;
    msg.buf[4] = 0x0;
    msg.buf[5] = 0x0;
    msg.buf[6] = 0x0;
    bmscan.write(msg, settings.veCanIndex);
    delay(2);
    msg.id = 0x286;
    msg.len = 8;
    msg.buf[0] = highByte(uint16_t(settings.ChargeVsetpoint * settings.Scells * 10)); // volage
    msg.buf[1] = lowByte(uint16_t(settings.ChargeVsetpoint * settings.Scells * 10));
    msg.buf[2] = lowByte(chargecurrent / settings.numberOfChargers);
    msg.buf[3] = 0x0;
    msg.buf[4] = 0x0;
    msg.buf[5] = 0x0;
    msg.buf[6] = 0x0;
    bmscan.write(msg, settings.chargerCanIndex);
    delay(2);
}
