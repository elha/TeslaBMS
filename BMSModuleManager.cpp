#include "config.h"
#include "BMSModuleManager.h"
#include "BMSUtil.h"
#include "Logger.h"

BMSModuleManager::BMSModuleManager()
{
    for (int i = 1; i <= MAX_MODULE_ADDR; i++)
    {
        modules[i].setExists(false);
        modules[i].setAddress(i);
    }
    isFaulted = false;
}

bool BMSModuleManager::balanceCells(float avgCellV, float triggerDiffCellV)
{
    bool out = false;
    for (int x = 1; x <= MAX_MODULE_ADDR; x+=2)
    {
        if (modules[x].isExisting() && modules[x+1].isExisting() && (modules[x].needsBalancing(avgCellV + triggerDiffCellV) || modules[x+1].needsBalancing(avgCellV + triggerDiffCellV)))
            {
                out = true;
                modules[x].balanceCells(avgCellV);
                modules[x+1].balanceCells(avgCellV);
            }
    }
    return out;
}

void BMSModuleManager::balanceInfo()
{
    for (int x = 1; x <= MAX_MODULE_ADDR; x++)
    {
        if (modules[x].isExisting() && modules[x].isBalancing())
            {
                modules[x].print();
            }
    }
}
/*
 * Try to set up any unitialized boards. Send a command to address 0 and see if there is a response. If there is then there is
 * still at least one unitialized board. Go ahead and give it the first ID not registered as already taken.
 * If we send a command to address 0 and no one responds then every board is inialized and this routine stops.
 * Don't run this routine until after the boards have already been enumerated.\
 * Note: The 0x80 conversion it is looking might in theory block the message from being forwarded so it might be required
 * To do all of this differently. Try with multiple boards. The alternative method would be to try to set the next unused
 * address and see if any boards respond back saying that they set the address. 
 */
void BMSModuleManager::setupBoards()
{
    Logger::debug("setupBoards starting");
    uint8_t payload[3];
    uint8_t buff[10];
    int retLen;

    payload[0] = 0;
    payload[1] = 0;
    payload[2] = 1;

    while (1 == 1)
    {
        payload[0] = 0;
        payload[1] = 0;
        payload[2] = 1;
        retLen = BMSUtil::sendDataWithReply(payload, 3, false, buff, 4);
        if (retLen == 4)
        {
            if (buff[0] == 0x80 && buff[1] == 0 && buff[2] == 1)
            {
                Logger::debug("00 found");
                //look for a free address to use
                for (int y = 1; y < 63; y++)
                {
                    Logger::debug("Addresstest %X", y);
                    if (!modules[y].isExisting())
                    {
                        payload[0] = 0;
                        payload[1] = REG_ADDR_CTRL;
                        payload[2] = y | 0x80;
                        BMSUtil::sendData(payload, 3, true);
                        delay(3);
                        if (BMSUtil::getReply(buff, 10) > 2)
                        {
                            Logger::debug("got reply %X", y);
                            if (buff[0] == (0x81) && buff[1] == REG_ADDR_CTRL && buff[2] == (y + 0x80))
                            {
                                modules[y].setExists(true);
                                numFoundModules++;
                                Logger::debug("Address assigned %X", y);
                            }
                            else
                            {
                                Logger::debug("Address no response %X", y);
                            }
                        }
                        break; //quit the for loop
                    }
                }
            }
            else
            {
              Logger::debug("no response from bms 1");
              break; //nobody responded properly to the zero address so our work here is done.
            }
        }
        else
        {
            Logger::debug("no response from bms 2");
            break; //nobody responded properly to the zero address so our work here is done.
        }    
    }
}

/*
 * Iterate through all 62 possible board addresses (1-62) to see if they respond
 */
void BMSModuleManager::findBoards()
{
    uint8_t payload[3];
    uint8_t buff[8];

    numFoundModules = 0;
    payload[0] = 0;
    payload[1] = 0; //read registers starting at 0
    payload[2] = 1; //read one byte
    for (int x = 1; x <= MAX_MODULE_ADDR; x++)
    {
        modules[x].setExists(false);
        payload[0] = x << 1;
        BMSUtil::sendData(payload, 3, false);
        delay(20);
        if (BMSUtil::getReply(buff, 8) > 4)
        {
            if (buff[0] == (x << 1) && buff[1] == 0 && buff[2] == 1 && buff[4] > 0)
            {
                modules[x].setExists(true);
                numFoundModules++;
                Logger::debug("Found module with address: %X", x);
            }
        }
        delay(5);
    }
}

/*
 * Force all modules to reset back to address 0 then set them all up in order so that the first module
 * in line from the master board is 1, the second one 2, and so on.
*/
void BMSModuleManager::renumberBoardIDs()
{
    Logger::debug("renumberBoardIDs starting");
    pinMode(INBMBFAULT, INPUT);

    uint8_t payload[3];
    uint8_t buff[8];
    int attempts = 1;

    for (int y = 1; y < 63; y++)
    {
        modules[y].setExists(false);
        numFoundModules = 0;
    }

    while (attempts < 3)
    {
        payload[0] = 0x3F << 1; //broadcast the reset command
        payload[1] = 0x3C;      //reset
        payload[2] = 0xA5;      //data to cause a reset
        BMSUtil::sendData(payload, 3, true);
        delay(100);
        BMSUtil::getReply(buff, 8);
        if (buff[0] == 0x7F && buff[1] == 0x3C && buff[2] == 0xA5 && buff[3] == 0x57)
            break;
        attempts++;
    }

    setupBoards();
}

/*
After a RESET boards have their faults written due to the hard restart or first time power up, this clears thier faults
*/
void BMSModuleManager::clearFaults()
{
    uint8_t payload[3];
    uint8_t buff[8];
    payload[0] = 0x7F;             //broadcast
    payload[1] = REG_ALERT_STATUS; //Alert Status
    payload[2] = 0xFF;             //data to cause a reset
    BMSUtil::sendDataWithReply(payload, 3, true, buff, 4);

    payload[0] = 0x7F; //broadcast
    payload[2] = 0x00; //data to clear
    BMSUtil::sendDataWithReply(payload, 3, true, buff, 4);

    payload[0] = 0x7F;             //broadcast
    payload[1] = REG_FAULT_STATUS; //Fault Status
    payload[2] = 0xFF;             //data to cause a reset
    BMSUtil::sendDataWithReply(payload, 3, true, buff, 4);

    payload[0] = 0x7F; //broadcast
    payload[2] = 0x00; //data to clear
    BMSUtil::sendDataWithReply(payload, 3, true, buff, 4);

    isFaulted = false;
    communicationErrors = 0;
}

/*
Puts all boards on the bus into a Sleep state, very good to use when the vehicle is a rest state. 
Pulling the boards out of sleep only to check voltage decay and temperature when the contactors are open.
*/

void BMSModuleManager::sleepBoards()
{
    uint8_t payload[3];
    uint8_t buff[8];
    payload[0] = 0x7F;        //broadcast
    payload[1] = REG_IO_CTRL; //IO ctrl start
    payload[2] = 0x04;        //write sleep bit
    BMSUtil::sendData(payload, 3, true);
    delay(2);
    BMSUtil::getReply(buff, 8);
}

/*
Wakes all the boards up and clears thier SLEEP state bit in the Alert Status Registery
*/

void BMSModuleManager::wakeBoards()
{
    uint8_t payload[3];
    uint8_t buff[8];
    payload[0] = 0x7F;        //broadcast
    payload[1] = REG_IO_CTRL; //IO ctrl start
    payload[2] = 0x00;        //write sleep bit
    BMSUtil::sendData(payload, 3, true);
    delay(2);
    BMSUtil::getReply(buff, 8);

    payload[0] = 0x7F;             //broadcast
    payload[1] = REG_ALERT_STATUS; //Alert Status
    payload[2] = 0x04;             //data to cause a reset
    BMSUtil::sendData(payload, 3, true);
    delay(2);
    BMSUtil::getReply(buff, 8);
    payload[0] = 0x7F; //broadcast
    payload[2] = 0x00; //data to clear
    BMSUtil::sendData(payload, 3, true);
    delay(2);
    BMSUtil::getReply(buff, 8);
}

void BMSModuleManager::getAllVoltTemp()
{
    for (int x = 1; x <= MAX_MODULE_ADDR; x++)
    {
        if (modules[x].isExisting())
        {
            Logger::debug("");
            Logger::debug("Module %i exists. Reading voltage and temperature values", x);
            modules[x].readModuleValues();
            Logger::debug("Module voltage: %f", modules[x].getModuleVoltage());
            Logger::debug("Lowest Cell V: %f     Highest Cell V: %f", modules[x].getLowCellV(), modules[x].getHighCellV());
            Logger::debug("Temp1: %f       Temp2: %f", modules[x].getTemperature(0), modules[x].getTemperature(1));
        }
    }

    if (digitalRead(INBMBFAULT) == LOW)
    {
        if (!isFaulted)
            Logger::error("One or more BMS modules have entered the fault state!");
        isFaulted = true;
    }
    else
    {
        if (isFaulted)
            Logger::info("All modules have exited a faulted state");
        isFaulted = false;
    }
}

float BMSModuleManager::getLowCellVolt()
{
    float LowCellVolt = 5.0;
    for (int x = 1; x <= MAX_MODULE_ADDR; x++)
    {
        if (modules[x].isExisting())
        {
            if (modules[x].getLowCellV() < LowCellVolt)
                LowCellVolt = modules[x].getLowCellV();
        }
    }
    return LowCellVolt;
}

float BMSModuleManager::getHighCellVolt()
{
    float HighCellVolt = 1.0;
    for (int x = 1; x <= MAX_MODULE_ADDR; x++)
    {
        if (modules[x].isExisting())
        {
            if (modules[x].getHighCellV() > HighCellVolt)
                HighCellVolt = modules[x].getHighCellV();
        }
    }
    return HighCellVolt;
}

float BMSModuleManager::getPackVoltage()
{
    float packVolt = 0.0;
    for (int x = 1; x <= MAX_MODULE_ADDR; x++)
    {
        if (modules[x].isExisting())
        {
            packVolt += modules[x].getModuleVoltage();
                    }
    }
    return packVolt;
}

void BMSModuleManager::setBatteryID(int id)
{
    batteryID = id;
}

float BMSModuleManager::getAvgTemperature()
{
    float avg = 0.0f;
    int y = 0; //counter for modules below -70 (no sensors connected)
    for (int x = 1; x <= MAX_MODULE_ADDR; x++)
    {
        if (modules[x].isExisting())
        {
            if (modules[x].getAvgTemp() > -70)
            {
                avg += modules[x].getAvgTemp();
            }
            else
            {
                y++;
            }
        }
    }
    avg = avg / (float)(numFoundModules - y);

    return avg;
}

float BMSModuleManager::getLowTemperature()
{
    float Low = 100.0f;
    for (int x = 1; x <= MAX_MODULE_ADDR; x++)
    {
        if (modules[x].isExisting())
        {
            if (modules[x].getLowTemp() < Low)
            {
                Low = modules[x].getLowTemp();
            }
        }
    }
    return Low;
}

float BMSModuleManager::getHighTemperature()
{
    float High = 0.0f;
    for (int x = 1; x <= MAX_MODULE_ADDR; x++)
    {
        if (modules[x].isExisting())
        {
            if (modules[x].getHighTemp() > High)
            {
                High = modules[x].getHighTemp();
            }
        }
    }
    return High;
}

float BMSModuleManager::getAvgCellVolt()
{
    float avg = 0.0f;
    for (int x = 1; x <= MAX_MODULE_ADDR; x++)
    {
        if (modules[x].isExisting())
            avg += modules[x].getAverageV();
    }
    avg = avg / (float)numFoundModules;

    return avg;
}

void BMSModuleManager::printPackDetails()
{
    uint8_t faults;
    uint8_t alerts;
    uint8_t COV;
    uint8_t CUV;

    Logger::console("");
    Logger::console("");
    Logger::console("");
    Logger::console("                                     Pack Status:");

    if (isFaulted)
        Logger::console("                                       FAULTED!");
    else
        Logger::console("                                   All systems go!");
    
    Logger::console("Modules: %i    Voltage: %fV   Avg Cell Voltage: %fV     Avg Temp: %fC ", numFoundModules,
                     getPackVoltage(), getAvgCellVolt(), getAvgTemperature());

    Logger::console("");

    for (int y = 1; y < 63; y++)
    {
        if (modules[y].isExisting())
        {
            faults = modules[y].getFaults();
            alerts = modules[y].getAlerts();
            COV = modules[y].getCOVCells();
            CUV = modules[y].getCUVCells();

            Logger::console("Module #%i", y);

            Logger::console("  Voltage: %fV   (%fV-%fV)     Temperatures: (%fC-%fC)", modules[y].getModuleVoltage(),
                            modules[y].getLowCellV(), modules[y].getHighCellV(), modules[y].getLowTemp(), modules[y].getHighTemp());

            if (faults > 0)
            {
                Logger::console("  MODULE IS FAULTED:");
                if (faults & 1)
                {
                    for (int i = 0; i < 6; i++)
                    {
                        if (COV & (1 << i))
                        {
                            Logger::console("    Overvoltage Cell %i ", i+1);
                        }
                    }
                }
             if (faults & 2)
                {
                    for (int i = 0; i < 6; i++)
                    {
                        if (CUV & (1 << i))
                        {
                            Logger::console("    Undervoltage Cell %i ", i+1);
                        }
                    }
                }
                if (faults & 4)
                {
                    Logger::console("    CRC error in received packet");
                    communicationErrors += 1;
                }
                if (faults & 8)
                {
                    Logger::console("    Power on reset has occurred");
                    communicationErrors += 1;
                }
                if (faults & 0x10)
                {
                    Logger::console("    Test fault active");
                    communicationErrors += 1;
                }
                if (faults & 0x20)
                {
                    Logger::console("    Internal registers inconsistent");
                    communicationErrors += 1;
                }
            }
            if (alerts > 0)
            {
                Logger::console("  MODULE HAS ALERTS:");
                if (alerts & 1)
                {
                    Logger::console("    Over temperature on TS1");
                    communicationErrors += 1;
                }
                if (alerts & 2)
                {
                    Logger::console("    Over temperature on TS2");
                    communicationErrors += 1;
                }
                if (alerts & 4)
                {
                    Logger::console("    Sleep mode active");
                }
                if (alerts & 8)
                {
                    Logger::console("    Thermal shutdown active");
                }
                if (alerts & 0x10)
                {
                    Logger::console("    Test Alert");
                }
                if (alerts & 0x20)
                {
                    Logger::console("    OTP EPROM Uncorrectable Error");
                    communicationErrors += 1;
                }
                if (alerts & 0x40)
                {
                    Logger::console("    GROUP3 Regs Invalid");
                    communicationErrors += 1;
                }
                if (alerts & 0x80)
                {
                    Logger::console("    Address not registered");
                    communicationErrors += 1;
                }
            }
        }
    }
}

long BMSModuleManager::getCommunicationErrors()
{
    return communicationErrors;
}

void BMSModuleManager::resetCommunicationErrors()
{
    communicationErrors = 0;
}
