#include "config.h"
#include "BMSModule.h"
#include "BMSUtil.h"
#include "Logger.h"

BMSModule::BMSModule()
{
    for (int i = 0; i < 6; i++)
    {
        cellVolt[i] = 0.0f;
    }
    moduleVolt = 0.0f;
    temperatures[0] = 0.0f;
    temperatures[1] = 0.0f;
    exists = false;
    moduleAddress = 0;
}

/*
Reading the status of the board to identify any flags, will be more useful when implementing a sleep cycle
*/
void BMSModule::readStatus()
{
    uint8_t payload[3];
    uint8_t buff[8];
    payload[0] = moduleAddress << 1; //adresss
    payload[1] = REG_ALERT_STATUS;   //Alert Status start
    payload[2] = 0x04;
    BMSUtil::sendDataWithReply(payload, 3, false, buff, 7);
    alerts = buff[3];
    faults = buff[4];
    COVFaults = buff[5];
    CUVFaults = buff[6];
}

uint8_t BMSModule::getFaults()
{
    return faults;
}

uint8_t BMSModule::getAlerts()
{
    return alerts;
}

uint8_t BMSModule::getCOVCells()
{
    return COVFaults;
}

uint8_t BMSModule::getCUVCells()
{
    return CUVFaults;
}

/*
Reading the setpoints, after a reset the default tesla setpoints are loaded
Default response : 0x10, 0x80, 0x31, 0x81, 0x08, 0x81, 0x66, 0xff
*/
/*
void BMSModule::readSetpoint()
{
  uint8_t payload[3];
  uint8_t buff[12];
  payload[0] = moduleAddress << 1; //adresss
  payload[1] = 0x40;//Alert Status start
  payload[2] = 0x08;//two registers
  sendData(payload, 3, false);
  delay(2);
  getReply(buff);

  OVolt = 2.0+ (0.05* buff[5]);
  UVolt = 0.7 + (0.1* buff[7]);
  Tset = 35 + (5 * (buff[9] >> 4));
} */

bool BMSModule::readModuleValues()
{
    uint8_t payload[4];
    uint8_t buff[50];
    uint8_t calcCRC;
    bool retVal = false;
    int retLen;
    double tempCalc;
    double tempTemp;

    payload[0] = moduleAddress << 1;

    readStatus();
    Logger::debug("Module %i   alerts=%X   faults=%X   COV=%X   CUV=%X", moduleAddress, alerts, faults, COVFaults, CUVFaults);

    payload[1] = REG_ADC_CTRL;
    payload[2] = 0b00111101; //ADC Auto mode, read every ADC input we can (Both Temps, Pack, 6 cells)
    BMSUtil::sendDataWithReply(payload, 3, true, buff, 3);

    payload[1] = REG_IO_CTRL;
    payload[2] = 0b00000011; //enable temperature measurement VSS pins
    BMSUtil::sendDataWithReply(payload, 3, true, buff, 3);

    payload[1] = REG_ADC_CONV; //start all ADC conversions
    payload[2] = 1;
    BMSUtil::sendDataWithReply(payload, 3, true, buff, 3);

    payload[1] = REG_GPAI; //start reading registers at the module voltage registers
    payload[2] = 0x12;     //read 18 bytes (Each value takes 2 - ModuleV, CellV1-6, Temp1, Temp2)
    retLen = BMSUtil::sendDataWithReply(payload, 3, false, buff, 22);

    calcCRC = BMSUtil::genCRC(buff, retLen - 1);
    Logger::debug("Sent CRC: %x     Calculated CRC: %x", buff[21], calcCRC);

    //18 data bytes, address, command, length, and CRC = 22 bytes returned
    //Also validate CRC to ensure we didn't get garbage data.
    if ((retLen == 22) && (buff[21] == calcCRC))
    {
        if (buff[0] == (moduleAddress << 1) && buff[1] == REG_GPAI && buff[2] == 0x12) //Also ensure this is actually the reply to our intended query
        {
            //payload is 2 bytes gpai, 2 bytes for each of 6 cell voltages, 2 bytes for each of two temperatures (18 bytes of data)
            moduleVolt = ((double)(buff[3] * 256 + buff[4]) * 0.002048544678082l); // was 4% off (against sum(cellVolts) and measurements)
            
            for (int i = 0; i < 6; i++)
            {
                cellVolt[i] = ((double)(buff[5 + (i * 2)] * 256 + buff[6 + (i * 2)]) * 0.000381493l);
            }
            
            for (int i = 0; i < 2; i++)
            {
                //Now using steinhart/hart equation for temperatures. We'll see if it is better than old code.
                tempTemp = (1.78l / ((double)(buff[17 + (i * 2)] * 256 + buff[18  + (i * 2)] + 2) / 33046.0l) - 3.57l);
                tempTemp *= 1000.0l;
                tempCalc = 1.0l / (0.0007610373573l + (0.0002728524832l * logf(tempTemp)) + (powf(logf(tempTemp), 3) * 0.0000001022822735l));
                temperatures[i] = tempCalc - 273.15f;
            }

            Logger::debug("Got voltage and temperature readings");
            retVal = true;
        }
    }
    else
    {
        Logger::error("Invalid module response received for module %i  len: %i   crc: %i   calc: %i",
                      moduleAddress, retLen, buff[21], calcCRC);
    }

    //turning the temperature wires off here seems to cause weird temperature glitches
    // payload[1] = REG_IO_CTRL;
    // payload[2] = 0b00000000; //turn off temperature measurement pins
    // BMSUtil::sendData(payload, 3, true);
    // delay(3);
    // BMSUtil::getReply(buff, 50);    //TODO: we're not validating the reply here. Perhaps check to see if a valid reply came back

    return retVal;
}

// balancing only needed if CellVoltageDiff is highter than supplied value
bool BMSModule::needsBalancing(float minDiffV)
{
    return getHighCellV() - getLowCellV() >= minDiffV ; 
}

void BMSModule::balanceCells()
{
    uint8_t balance = 0; //bit 0 - 5 are to activate cell balancing 1-6

    for (int i = 0; i < 6; i++)
    {
        if (getLowCellV() < getCellVoltage(i))
        {
            balance = balance | (1 << i); //set bit on cells with higher voltage than lowest cell
        }
    }

    if (balance != 0) //only send balance command when needed
    {
        uint8_t payload[4];
        uint8_t buff[30];
    
        payload[0] = moduleAddress << 1;
        payload[1] = REG_BAL_TIME;
        payload[2] = 0x05; //5 second balance limit, if not triggered to balance it will stop after 5 seconds
        BMSUtil::sendData(payload, 3, true);
        delay(2);
        BMSUtil::getReply(buff, 30);

        payload[0] = moduleAddress << 1;
        payload[1] = REG_BAL_CTRL;
        payload[2] = balance; //write balance state to register
        BMSUtil::sendData(payload, 3, true);
        delay(2);
        BMSUtil::getReply(buff, 30);

        if (Logger::isDebug()) //read registers back out to check if everything is good
        {
            delay(50);
            payload[0] = moduleAddress << 1;
            payload[1] = REG_BAL_TIME;
            payload[2] = 1; //
            BMSUtil::sendData(payload, 3, false);
            delay(2);
            BMSUtil::getReply(buff, 30);

            payload[0] = moduleAddress << 1;
            payload[1] = REG_BAL_CTRL;
            payload[2] = 1; //
            BMSUtil::sendData(payload, 3, false);
            delay(2);
            BMSUtil::getReply(buff, 30);
        }
    }
}

float BMSModule::getCellVoltage(int cell)
{
    if (cell < 0 || cell > 5)
        return 0.0f;
    return cellVolt[cell];
}

float BMSModule::getLowCellV()
{
    float lowVal = 10.0f;
    for (int i = 0; i < 6; i++)
        if (cellVolt[i] < lowVal)
            lowVal = cellVolt[i];
    return lowVal;
}

float BMSModule::getHighCellV()
{
    float hiVal = 0.0f;
    for (int i = 0; i < 6; i++)
        if (cellVolt[i] > hiVal)
            hiVal = cellVolt[i];
    return hiVal;
}

float BMSModule::getAverageV()
{
    float avgVal = 0.0f;
    for (int i = 0; i < 6; i++)
        avgVal += cellVolt[i];
    avgVal /= 6.0f;
    return avgVal;
}

float BMSModule::getLowTemp()
{
    return (temperatures[0] < temperatures[1]) ? temperatures[0] : temperatures[1];
}

float BMSModule::getHighTemp()
{
    return (temperatures[0] < temperatures[1]) ? temperatures[1] : temperatures[0];
}

float BMSModule::getAvgTemp()
{
    return (temperatures[0] + temperatures[1]) / 2.0f;
}

float BMSModule::getModuleVoltage()
{
    return moduleVolt;
}

float BMSModule::getTemperature(int temp)
{
    if (temp < 0 || temp > 1)
        return 0.0f;
    return temperatures[temp];
}

void BMSModule::setAddress(int newAddr)
{
    if (newAddr < 0 || newAddr > MAX_MODULE_ADDR)
        return;
    moduleAddress = newAddr;
}

int BMSModule::getAddress()
{
    return moduleAddress;
}

bool BMSModule::isExisting()
{
    return exists;
}

void BMSModule::setExists(bool ex)
{
    exists = ex;
}
