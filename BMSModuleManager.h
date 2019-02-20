#pragma once
#include "config.h"
#include "BMSModule.h"

class BMSModuleManager
{
  public:
    BMSModuleManager();
    bool balanceCells(double minDiffV);
    void setupBoards();
    void findBoards();
    void renumberBoardIDs();
    bool isFaulted;  
    void clearFaults();
    void sleepBoards();
    void wakeBoards();
    void getAllVoltTemp();
    void readSetpoints();
    void setBatteryID(int id); 
    void setUnderVolt(double newVal);
    void setOverVolt(double newVal);
    void setOverTemp(double newVal);
    void setBalanceV(double newVal);
    void setBalanceHyst(double newVal);
    double getPackVoltage();
    double getLowTemperature();
    double getHighTemperature();
    double getAvgTemperature();
    double getAvgCellVolt();
    double getLowCellVolt();
    double getHighCellVolt();
    double getHighVoltage();
    double getLowVoltage();
    void printPackSummary();
    void printPackDetails();
    long getCommunicationErrors();
    void resetCommunicationErrors();

  private:
    BMSModule modules[MAX_MODULE_ADDR + 1]; // store data for as many modules as we've configured for.
    int batteryID;
    int numFoundModules; // The number of modules that seem to exist
    long communicationErrors;
};
