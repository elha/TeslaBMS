#pragma once

class BMSModule
{
  public:
    BMSModule();
    void readStatus();
    bool readModuleValues();
    bool needsBalancing(double minDiffV);
    void balanceCells();
    double getCellVoltage(int cell);
    double getLowCellV();
    double getHighCellV();
    double getAverageV();
    double getLowTemp();
    double getHighTemp();
    double getAvgTemp();
    double getModuleVoltage();
    double getTemperature(int temp);
    uint8_t getFaults();
    uint8_t getAlerts();
    uint8_t getCOVCells();
    uint8_t getCUVCells();
    void setAddress(int newAddr);
    int getAddress();
    bool isExisting();
    void setExists(bool ex);

  private:
    double cellVolt[6]; // calculated as 16 bit value * 6.250 / 16383 = volts
    double moduleVolt;      // calculated as 16 bit value * 33.333 / 16383 = volts
    double temperatures[2]; // Don't know the proper scaling at this point
    bool exists;
    int alerts;
    int faults;
    int COVFaults;
    int CUVFaults;

    uint8_t moduleAddress; //1 to 0x3E
};
