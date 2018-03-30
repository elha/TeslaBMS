#pragma once
#include "config.h"
#include "BMSModule.h"

class BMSModuleManager
{
  public:
    BMSModuleManager();
    bool balanceCells(float minDiffV);
    void setupBoards();
    void findBoards();
    void renumberBoardIDs();
    void clearFaults();
    void sleepBoards();
    void wakeBoards();
    void getAllVoltTemp();
    void readSetpoints();
    void setBatteryID(int id);
    void setUnderVolt(float newVal);
    void setOverVolt(float newVal);
    void setOverTemp(float newVal);
    void setBalanceV(float newVal);
    void setBalanceHyst(float newVal);
    float getPackVoltage();
    float getAvgTemperature();
    float getAvgCellVolt();
    float getLowCellVolt();
    float getHighCellVolt();
    float getHighVoltage();
    float getLowVoltage();
    void printPackSummary();
    void printPackDetails();

  private:
    BMSModule modules[MAX_MODULE_ADDR + 1]; // store data for as many modules as we've configured for.
    int batteryID;
    int numFoundModules; // The number of modules that seem to exist
    bool isFaulted;
};
