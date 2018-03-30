#pragma once

class BMSModule
{
  public:
    BMSModule();
    void readStatus();
    bool readModuleValues();
    bool needsBalancing(float minDiffV);
    void balanceCells();
    float getCellVoltage(int cell);
    float getLowCellV();
    float getHighCellV();
    float getAverageV();
    float getLowTemp();
    float getHighTemp();
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

  private:
    float cellVolt[6]; // calculated as 16 bit value * 6.250 / 16383 = volts
    float moduleVolt;      // calculated as 16 bit value * 33.333 / 16383 = volts
    float temperatures[2]; // Don't know the proper scaling at this point
    bool exists;
    int alerts;
    int faults;
    int COVFaults;
    int CUVFaults;

    uint8_t moduleAddress; //1 to 0x3E
};
