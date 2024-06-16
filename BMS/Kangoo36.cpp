#include <Arduino.h>
#include "BMSCan.h"
#include <ACAN_ESP32.h>
#include <ACAN2515.h>
#include "config.h"
#include "Kangoo36.h"



// bms status values
#define Boot 0
#define Ready 1
#define Drive 2
#define Charge 3
#define Precharge 4
#define RapidCharge 5
#define Error 6

KangooCan::KangooCan(BMSCan& b, EEPROMSettings& s) : bmscan(b), settings{s}
{
};

void KangooCan::sendKeepAliveFrame(BMS_CAN_MESSAGE &msg, uint8_t &status)
{  
    //Default to '0s' so we can check later if it has changed.
    uint8_t defautMsg[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    memcpy(msg.buf, defautMsg, 8);

    msg.id = 0x423;
    msg.len = 8;

    if (status == Drive){
        uint8_t driveMsg[] = {0x07, 0x1D, 0x00, 0x02, 0x5D, 0x80, 0x5D, 0xD8};
        memcpy(msg.buf, driveMsg, 8);
    }else if (status == Charge){
        uint8_t chargeMsg[] = {0x0B, 0x1D, 0x00, 0x02, 0xB2, 0x20, 0xB2, 0xD9};
        memcpy(msg.buf, chargeMsg, 8);
    }else if (status == RapidCharge){
        uint8_t rapidChargeMsg[] = {0x07, 0x1E, 0x00, 0x01, 0x5D, 0x20, 0xB2, 0xC7};
        memcpy(msg.buf, rapidChargeMsg, 8);
    }
    //Only send when in a valid status
    if (msg.buf[0] != 0x00){
        bmscan.write(msg, settings.veCanIndex);
    }
}

void KangooCan::handleIncomingCAN(BMS_CAN_MESSAGE &inMsg)
{
  if (inMsg.id == 0x155)
  {
    this->handleFrame155(inMsg.buf);
  }
  if (inMsg.id == 0x424)
  {
    this->handleFrame424(inMsg.buf);
  }
  if (inMsg.id == 0x424)
  {
    this->handleFrame424(inMsg.buf);
  }
  else if (inMsg.id ==  0x7BB)
  {
    this->ProcessISOTPResponse(inMsg.buf);
  }
}


void KangooCan::handleFrame155(const uint8_t* data) {
    this->stateOfCharge = (float) ((data[4] << 8) + data[5]) * 0.0025;
    this->maxCharging = data[0] * 300;
}

void KangooCan::handleFrame424(const uint8_t* data) {
    this->stateOfHealth = data[5];
    this->lowestCellTemp = (uint8_t)(data[4]) - 40;
    this->highestCellTemp = (uint8_t)(data[7]) - 40;
    this->maxInputPower = data[2] / 2;
    this->maxOutputPower = data[3] / 2;

}

void KangooCan::handleFrame425(const uint8_t* data) {
    this->lowestCellVolt = (float)((((data[6] & 0x01) << 8) + data[7]) * 0.01) + 1;
    this->highestCellVolt = (float)((((data[6] & 0x03) << 8) + ((data[5] & 0xFE) >> 1)) * 0.01) + 1;
    this->remainingKHW = (float)(data[1] * 0.1);

}

bool KangooCan::ProcessISOTPResponse(const uint8_t* data) {
    uint8_t length = sizeof(data);
    //start of response
    if (data[0] == 0x10 && data[2] == 0x61) {
        remainingByteCount = data[1];
        byteCount = 0;
        currentPID = data[3];
    }
   if (currentPID == 0x01) {
        return handlePID01Frame(data);
    }
    if (currentPID == 0x61) {
        return handlePID61Frame(data);
    }
    if (currentPID == 0x66) {
        return handlePID66Frame(data);
    }
    if (currentPID == 0x03) {
        return handlePID03Frame(data);
    }
    if (currentPID == 0x41) {
        return handlePID41Frame(data);
    }
    if (currentPID == 0x42) {
        return handlePID42Frame(data);
    }
    return false;
}

bool KangooCan::handlePID61Frame(const uint8_t* data) {
    if (data[0] == 0x10) {
        return true; //requst next data
    } else if (data[0] == 0x21) {
        //amphours
        this->ampHoursRaw = (data[1] << 24) + (data[2] << 16) + (data[3] << 8) + data[4];
        this->packHealthRaw = data[5];
        this->totalKilometers = (data[6] << 24) + (data[7] << 16);
        return true; //requst next data
    } else if (data[0] == 0x22) {
        this->totalKilometers = this->totalKilometers + data[1];

        return false; //requst next data
    }

    return false;
}

bool KangooCan::handlePID66Frame(const uint8_t* data) {
    if (data[0] == 0x10) {
        this->quickchargeCount = (data[4] << 8) + data[5];
        this->normalchargeCount = (data[6] << 8) + data[7];
        return true; //requst next data
    } else if (data[0] == 0x21) {
        this->fullchargeCount = (data[0] << 8) + data[1];
        this->partialchargeCount = (data[2] << 8) + data[3];
        return true; //requst next data
    } else if (data[0] == 0x22) {
        return false; //requst next data
    }

    return false;
}

bool KangooCan::handlePID03Frame(const uint8_t* data) {
    if (data[0] == 0x10) {
        return true;
    } else if (data[0] == 0x21) {
        this->batteryTemperature = (data[1] << 8) + data[2];
        this->highestMilV = (data[7] << 8);
        return true;
    } else if (data[0] == 0x22) {
        this->highestMilV = highestMilV + data[1];
        this->lowestMilV = (data[2] << 8) + data[3];
        return true;
    }  else if (data[0] == 0x23) {
        return true;
    } else if (data[0] == 0x24) {
        //SOC?
        return true;
    } else if (data[0] == 0x25) {
        //internal resistance?]
        //real SOC?
        return true;
    } else if (data[0] == 0x26) {
        //real SOC?
        //ah?
        return true;
    } else if (data[0] == 0x27) {
        //max charge? isolation?
        this->maxChargingRaw = (data[1] << 8) + data[2];
        return false;
    }

    return false;
}

bool KangooCan::handlePID04Frame(const uint8_t* data) {
    return false;
}

bool KangooCan::handlePID01Frame(const uint8_t* data) {
    if (data[0] == 0x10) {
        return true;
    } else if (data[0] == 0x21) {
        this->batteryCurrent = (data[3] << 24) + (data[4] << 16) + (data[5] << 8) + (data[6]);
        return true;
    } else if (data[0] == 0x22) {
        return true;
    }  else if (data[0] == 0x23) {
        this->maxInputPowerRaw = (data[3] << 8) + (data[4]);
        this->maxOutputPowerRaw = (data[5] << 8) + (data[6]);

        return true;
    } else if (data[0] == 0x24) {
        return true;
    } else if (data[0] == 0x25) {
        return true;
    } else if (data[0] == 0x26) {
        return true;
    } else if (data[0] == 0x27) {
        return false;
    }

    return false;
}

bool KangooCan::handlePID41Frame(const uint8_t* data) {
    if (data[0] == 0x10) {
        looped = false;
        this->cellVoltages[0] = (data[4] << 8) + data[5];
        this->cellVoltages[1] = (data[6] << 8) + data[7];
        return true;
    } else if (!looped && data[0] == 0x21) {
        this->cellVoltages[2] = (data[1] << 8) + data[2];
        this->cellVoltages[3] = (data[3] << 8) + data[4];
        this->cellVoltages[4] = (data[5] << 8) + data[6];
        this->cellVoltages[5] = (data[7] << 8);
        return true;
    } else if (!looped && data[0] == 0x22) {
        this->cellVoltages[5] = this->cellVoltages[5] + data[1];
        this->cellVoltages[6] = (data[2] << 8) + data[3];
        this->cellVoltages[7] = (data[4] << 8) + data[5];
        this->cellVoltages[8] = (data[6] << 8) + data[7];
        return true;
    } else if (data[0] == 0x23) {
        this->cellVoltages[9] = (data[1] << 8) + data[2];
        this->cellVoltages[10] = (data[3] << 8) + data[4];
        this->cellVoltages[11] = (data[5] << 8) + data[6];
        this->cellVoltages[12] = (data[7] << 8);
        return true;
    } else if (data[0] == 0x24) {
        this->cellVoltages[12] = this->cellVoltages[12] + data[1];
        this->cellVoltages[13] = (data[2] << 8) + data[3];
        this->cellVoltages[14] = (data[4] << 8) + data[5];
        this->cellVoltages[15] = (data[6] << 8) + data[7];
        return true;
    } else if (data[0] == 0x25) {
        this->cellVoltages[16] = (data[1] << 8) + data[2];
        this->cellVoltages[17] = (data[3] << 8) + data[4];
        this->cellVoltages[18] = (data[5] << 8) + data[6];
        this->cellVoltages[19] = (data[7] << 8);
        return true;
    } else if (data[0] == 0x26) {
        this->cellVoltages[19] = this->cellVoltages[19] + data[1];
        this->cellVoltages[20] = (data[2] << 8) + data[3];
        this->cellVoltages[21] = (data[4] << 8) + data[5];
        this->cellVoltages[22] = (data[6] << 8) + data[7];
        return true;
    } else if (data[0] == 0x27) {
        this->cellVoltages[23] = (data[1] << 8) + data[2];
        this->cellVoltages[24] = (data[3] << 8) + data[4];
        this->cellVoltages[25] = (data[5] << 8) + data[6];
        this->cellVoltages[26] = (data[7] << 8);
        return true;
    } else if (data[0] == 0x28) {
        this->cellVoltages[26] = this->cellVoltages[26] + data[1];
        this->cellVoltages[27] = (data[2] << 8) + data[3];
        this->cellVoltages[28] = (data[4] << 8) + data[5];
        this->cellVoltages[29] = (data[6] << 8) + data[7];
        return true;
    } else if (data[0] == 0x29) {
        this->cellVoltages[30] = (data[1] << 8) + data[2];
        this->cellVoltages[31] = (data[3] << 8) + data[4];
        this->cellVoltages[32] = (data[5] << 8) + data[6];
        this->cellVoltages[33] = (data[7] << 8);
        return true;
    } else if (data[0] == 0x2A) {
        this->cellVoltages[33] = this->cellVoltages[33] + data[1];
        this->cellVoltages[34] = (data[2] << 8) + data[3];
        this->cellVoltages[35] = (data[4] << 8) + data[5];
        this->cellVoltages[36] = (data[6] << 8) + data[7];
        return true;
    } else if (data[0] == 0x2B) {
        this->cellVoltages[37] = (data[1] << 8) + data[2];
        this->cellVoltages[38] = (data[3] << 8) + data[4];
        this->cellVoltages[39] = (data[5] << 8) + data[6];
        this->cellVoltages[40] = (data[7] << 8);
        return true;
    } else if (data[0] == 0x2C) {
        this->cellVoltages[40] = this->cellVoltages[40] + data[1];
        this->cellVoltages[41] = (data[2] << 8) + data[3];
        this->cellVoltages[42] = (data[4] << 8) + data[5];
        this->cellVoltages[43] = (data[6] << 8) + data[7];
        return true;
    } else if (data[0] == 0x2D) {
        this->cellVoltages[44] = (data[1] << 8) + data[2];
        this->cellVoltages[45] = (data[3] << 8) + data[4];
        this->cellVoltages[46] = (data[5] << 8) + data[6];
        this->cellVoltages[47] = (data[7] << 8);
        return true;
    } else if (data[0] == 0x2E) {
        this->cellVoltages[47] = this->cellVoltages[47] + data[1];
        this->cellVoltages[48] = (data[2] << 8) + data[3];
        this->cellVoltages[49] = (data[4] << 8) + data[5];
        this->cellVoltages[50] = (data[6] << 8) + data[7];
        return true;
    } else if (data[0] == 0x2F) {
        this->cellVoltages[51] = (data[1] << 8) + data[2];
        this->cellVoltages[52] = (data[3] << 8) + data[4];
        this->cellVoltages[53] = (data[5] << 8) + data[6];
        this->cellVoltages[54] = (data[7] << 8);
        return true;
    } else if (data[0] == 0x20) {
        looped = true;
        this->cellVoltages[54] = this->cellVoltages[54] + data[1];
        this->cellVoltages[55] = (data[2] << 8) + data[3];
        this->cellVoltages[56] = (data[4] << 8) + data[5];
        this->cellVoltages[57] = (data[6] << 8) + data[7];
        return true;
        //21 and 22 will come again
    } else if (looped && data[0] == 0x21) {
        //21 and 22 will come again
        this->cellVoltages[58] = (data[1] << 8) + data[2];
        this->cellVoltages[59] = (data[3] << 8) + data[4];
        this->cellVoltages[60] = (data[5] << 8) + data[6];
        this->cellVoltages[61] = (data[7] << 8);
        return true;
    } else if (looped && data[0] == 0x22) {
        //21 and 22 will come again
        looped = false;
        this->cellVoltages[61] = this->cellVoltages[61] + data[1];
        return false;
    }
    return false;
}
bool KangooCan::handlePID42Frame(const uint8_t* data) {
    if (data[0] == 0x10) {
        this->cellVoltages[62] = (data[4] << 8) + data[5];
        this->cellVoltages[63] = (data[6] << 8) + data[7];
        return true;
    } else if (data[0] == 0x21) {
        this->cellVoltages[64] = (data[1] << 8) + data[2];
        this->cellVoltages[65] = (data[3] << 8) + data[4];
        this->cellVoltages[66] = (data[5] << 8) + data[6];
        this->cellVoltages[67] = (data[7] << 8);
        return true;
    } else if (data[0] == 0x22) {
        this->cellVoltages[67] = this->cellVoltages[67] + data[1];
        this->cellVoltages[68] = (data[2] << 8) + data[3];
        this->cellVoltages[69] = (data[4] << 8) + data[5];
        this->cellVoltages[70] = (data[6] << 8) + data[7];
        return true;
    } else if (data[0] == 0x23) {
        this->cellVoltages[71] = (data[1] << 8) + data[2];
        this->cellVoltages[72] = (data[3] << 8) + data[4];
        this->cellVoltages[73] = (data[5] << 8) + data[6];
        this->cellVoltages[74] = (data[7] << 8);
        return true;
    } else if (data[0] == 0x24) {
        this->cellVoltages[74] = this->cellVoltages[74] + data[1];
        this->cellVoltages[75] = (data[2] << 8) + data[3];
        this->cellVoltages[76] = (data[4] << 8) + data[5];
        this->cellVoltages[77] = (data[6] << 8) + data[7];
        return true;
    } else if (data[0] == 0x25) {
        this->cellVoltages[78] = (data[1] << 8) + data[2];
        this->cellVoltages[79] = (data[3] << 8) + data[4];
        this->cellVoltages[80] = (data[5] << 8) + data[6];
        this->cellVoltages[81] = (data[7] << 8);
        return true;
    } else if (data[0] == 0x26) {
        this->cellVoltages[81] = this->cellVoltages[81] + data[1];
        this->cellVoltages[82] = (data[2] << 8) + data[3];
        this->cellVoltages[83] = (data[4] << 8) + data[5];
        this->cellVoltages[84] = (data[6] << 8) + data[7];
        return true;
    } else if (data[0] == 0x27) {
        this->cellVoltages[85] = (data[1] << 8) + data[2];
        this->cellVoltages[86] = (data[3] << 8) + data[4];
        this->cellVoltages[87] = (data[5] << 8) + data[6];
        this->cellVoltages[88] = (data[7] << 8);
        return true;
    } else if (data[0] == 0x28) {
        this->cellVoltages[88] = this->cellVoltages[88] + data[1];
        this->cellVoltages[89] = (data[2] << 8) + data[3];
        this->cellVoltages[90] = (data[4] << 8) + data[5];
        this->cellVoltages[91] = (data[6] << 8) + data[7];
        return true;
    } else if (data[0] == 0x29) {
        this->cellVoltages[92] = (data[1] << 8) + data[2];
        this->cellVoltages[93] = (data[3] << 8) + data[4];
        this->cellVoltages[94] = (data[5] << 8) + data[6];
        this->cellVoltages[95] = (data[7] << 8);
        return true;
    } else if (data[0] == 0x2A) {
        this->cellVoltages[95] = this->cellVoltages[95] + data[1];
        this->packVoltage = (data[2] << 8) + data[3];
        this->daigPackVoltage = (data[4] << 8) + data[5]; //? not sure what this is for
        return false;
    }
    return false;
}

void KangooCan::printData() {
    //not sure about comented out values 
    cout << "------ Free Frames Data --------" << endl << endl;
    cout << "SoC: " << this->stateOfCharge << "%" << endl;
    cout << "SoH: " << this->stateOfHealth << "%" << endl;
    cout << "Lowest Cell: " << this->lowestCellVolt << "V" <<endl;
    cout << "Highest Cell: " << this->highestCellVolt << "V" <<endl;
    cout << "Remaining KWH: " << this->remainingKHW << "kWh" <<endl;
    cout << "Highest Temp: " << (int)this->highestCellTemp << "C" << endl;
    cout << "Lowest Temp: " << (int)this->lowestCellTemp << "C" << endl;
    cout << "Max Charing Rate: " << (int) this->maxCharging << "W" << endl;
    cout << "Max Input Power: " << (int) this->maxInputPower << "kW" << endl;
    cout << "Max Output Power: " << (int) this->maxOutputPower << "kW" << endl;

    cout << endl << endl;
    cout << "------ ISOTP Kangoo Data -------" << endl << endl;

    //cout << "AmpHours: " << (ampHoursRaw / 10000) << "?" << endl;
    cout << "Battery Current: " << this->batteryCurrent << "mA" << endl;
    cout << "State Of Health: " << (this->packHealthRaw / 2) << "%" << endl;
    cout << "QuickCharge Count: " << this->quickchargeCount << endl;
    cout << "NormalCharge Count: " << this->normalchargeCount << endl;
    //cout << "FullCharge Count: " << fullchargeCount << "?" << endl;
    //cout << "PartialCharge Count: " << partialchargeCount << "?" << endl;
    //cout << "Total Kilometers: " << totalKilometers << endl;
    cout << "Max Input: " << (float)this->maxInputPowerRaw / 100 << " kW" << endl;
    cout << "Max Output: " << (float)this->maxOutputPowerRaw / 100 << " kW" << endl;

    //cout << "Battery Temperature: " << (float)batteryTemperature / 1000 << " C" << endl;

    cout << "Lowest Cell: " <<  (float)this->lowestMilV / 100 << " V" << endl;
    cout << "Highest Cell: " <<  (float)this->highestMilV / 100 << " V" << endl;

    cout << "Max Charging: " <<  this->maxChargingRaw / 10 << " kW" << endl;
    cout << "Pack Voltage: " << (float) this->packVoltage/100 << " V" << endl;
    cout << "Diagnostic Pack Voltage: " << (float) this->daigPackVoltage/100 << " V" << endl;

    // for (int i = 0; i < 96; i++) {
    //     cout << "Cell " <<  i << ": " << this->cellVoltages[i] << "mV" << endl;
    // }

}