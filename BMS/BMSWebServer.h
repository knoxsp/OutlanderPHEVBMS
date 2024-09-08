#ifndef BMS_WEB_SERVER_H
#define BMS_WEB_SERVER_H
#include "ESPAsyncWebServer.h"
#include "AsyncJson.h"
#include "ArduinoJson.h"
#include "config.h"
#include "BMSModuleManager.h"
#include "Kangoo36.h"

class BMSWebServer
{
public:
  BMSWebServer(EEPROMSettings &settings, KangooCan &bms);
  void setup();
  void execute();
  void broadcast(const char *message);
  AsyncWebSocket &getWebSocket();

private:
  EEPROMSettings &settings;
  KangooCan &bms;
};
#endif
