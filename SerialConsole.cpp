/*
 * SerialConsole.cpp
 *
 Copyright (c) 2017 EVTV / Collin Kidder

 Permission is hereby granted, free of charge, to any person obtaining
 a copy of this software and associated documentation files (the
 "Software"), to deal in the Software without restriction, including
 without limitation the rights to use, copy, modify, merge, publish,
 distribute, sublicense, and/or sell copies of the Software, and to
 permit persons to whom the Software is furnished to do so, subject to
 the following conditions:

 The above copyright notice and this permission notice shall be included
 in all copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

 */
#include "SerialConsole.h"
#include "Logger.h"
#include "BMSModuleManager.h"

template <class T>
inline Print &operator<<(Print &obj, T arg)
{
    obj.print(arg);
    return obj;
} //Lets us stream SerialUSB

extern BMSModuleManager bms;

bool printPrettyDisplay;
uint32_t prettyCounter;
int whichDisplay;

SerialConsole::SerialConsole()
{
    init();
}

void SerialConsole::init()
{
    //State variables for serial console
    ptrBuffer = 0;
    state = STATE_ROOT_MENU;
    loopcount = 0;
    cancel = false;
    printPrettyDisplay = false;
    prettyCounter = 0;
    whichDisplay = 0;
}

void SerialConsole::loop()
{
    if (SERIALCONSOLE.available())
    {
        serialEvent();
    }
    if (printPrettyDisplay && (millis() > (prettyCounter + 3000)))
    {
        prettyCounter = millis();
        if (whichDisplay == 0)
            bms.printPackSummary();
        if (whichDisplay == 1)
            bms.printPackDetails();
    }
}

void SerialConsole::printMenu()
{
    Logger::console("\n*************SYSTEM MENU *****************");
    Logger::console("Enable line endings of some sort (LF, CR, CRLF)");
    Logger::console("Most commands case sensitive\n");
    Logger::console("GENERAL SYSTEM CONFIGURATION\n");
    Logger::console("   h = help (displays this message)");
    Logger::console("   S = Sleep all boards");
    Logger::console("   W = Wake up all boards");
    Logger::console("   C = Clear all board faults");
    Logger::console("   F = Find all connected boards");
    Logger::console("   R = Renumber connected boards in sequence");
    Logger::console("   B = Attempt balancing for 5 seconds");
    Logger::console("   p = Toggle output of pack summary every 3 seconds");
    Logger::console("   d = Toggle output of pack details every 3 seconds");

    Logger::console("   LOGLEVEL=%i - set log level (0=debug, 1=info, 2=warn, 3=error, 4=off)", Logger::getLogLevel());
}

/*	There is a help menu (press H or h or ?)

    Commands are submitted by sending line ending (LF, CR, or both)
 */
void SerialConsole::serialEvent()
{
    int incoming;
    incoming = SERIALCONSOLE.read();
    if (incoming == -1)
    { //false alarm....
        return;
    }

    if (incoming == 10 || incoming == 13)
    { //command done. Parse it.
        handleConsoleCmd();
        ptrBuffer = 0; //reset line counter once the line has been processed
    }
    else
    {
        cmdBuffer[ptrBuffer++] = (unsigned char)incoming;
        if (ptrBuffer > 79)
            ptrBuffer = 79;
    }
}

void SerialConsole::handleConsoleCmd()
{

    if (state == STATE_ROOT_MENU)
    {
        if (ptrBuffer == 1)
        { //command is a single ascii character
            handleShortCmd();
        }
        else
        { //if cmd over 1 char then assume (for now) that it is a config line
            //handleConfigCmd();
        }
    }
}

void SerialConsole::handleShortCmd()
{
    uint8_t val;

    switch (cmdBuffer[0])
    {
    case 'h':
    case '?':
    case 'H':
        printMenu();
        break;
    case 'S':
        Logger::console("Sleeping all connected boards");
        bms.sleepBoards();
        break;
    case 'W':
        Logger::console("Waking up all connected boards");
        bms.wakeBoards();
        break;
    case 'C':
        Logger::console("Clearing all faults");
        bms.clearFaults();
        break;
    case 'F':
        bms.findBoards();
        break;
    case 'R':
        Logger::console("Renumbering all boards.");
        bms.renumberBoardIDs();
        break;
    case 'B':
        //bms.balanceCells(settings.);
        break;
    case 'p':
        if (whichDisplay == 1 && printPrettyDisplay)
            whichDisplay = 0;
        else
        {
            printPrettyDisplay = !printPrettyDisplay;
            if (printPrettyDisplay)
            {
                Logger::console("Enabling pack summary display, 5 second interval");
            }
            else
            {
                Logger::console("No longer displaying pack summary.");
            }
        }
        break;
    case 'd':
        if (whichDisplay == 0 && printPrettyDisplay)
            whichDisplay = 1;
        else
        {
            printPrettyDisplay = !printPrettyDisplay;
            whichDisplay = 1;
            if (printPrettyDisplay)
            {
                Logger::console("Enabling pack details display, 5 second interval");
            }
            else
            {
                Logger::console("No longer displaying pack details.");
            }
        }
        break;
    }
    /*

void loop_menu()
{
  if (SERIALCONSOLE.available() == 0)
    return;

  byte incomingByte = Serial.read(); // read the incoming byte:

  if (status.menuload == 2)
  {
    switch (incomingByte)
    {

    case 113: //c for calibrate zero offset

      status.menuload = 0;
      incomingByte = 115;
      break;

    default:
      // if nothing else matches, do the default
      // default is optional
      break;
    }
  }
  if (status.menuload == 3)
  {
    switch (incomingByte)
    {
    case 113: //q to go back to main menu

      status.menuload = 0;
      incomingByte = 115;
      break;

    case 102: //f factory settings
      setup_settings();
      SERIALCONSOLE.println("  ");
      SERIALCONSOLE.println("  ");
      break;

    case 100: //d display settings
      SERIALCONSOLE.println("  ");
      SERIALCONSOLE.println("  ");
      SERIALCONSOLE.print(settings.OverVSetpoint * 1000, 0);
      SERIALCONSOLE.print("mV Over Voltage Setpoint - 1 ");
      SERIALCONSOLE.println("  ");
      SERIALCONSOLE.print(settings.UnderVSetpoint * 1000, 0);
      SERIALCONSOLE.print("mV Under Voltage Setpoint - 2");
      SERIALCONSOLE.println("  ");
      SERIALCONSOLE.print(settings.OverTSetpoint);
      SERIALCONSOLE.print(" Over Temperature Setpoint - 3");
      SERIALCONSOLE.println("  ");
      SERIALCONSOLE.print(settings.UnderTSetpoint);
      SERIALCONSOLE.print(" Under Temperature Setpoint - 4");
      SERIALCONSOLE.println("  ");
      SERIALCONSOLE.print(settings.BalanceV * 1000, 0);
      SERIALCONSOLE.print("mV Balance Voltage Setpoint - 5 ");
      SERIALCONSOLE.println("  ");
      SERIALCONSOLE.print(settings.BalanceVHyst * 1000, 0);
      SERIALCONSOLE.print("mV Balance Voltage Hystersis - 6 ");
      SERIALCONSOLE.println("  ");
      break;

    case 101: //e display settings
      SERIALCONSOLE.println("  ");
      SERIALCONSOLE.println("Enter Variable Number and New value ");
      SERIALCONSOLE.println("  ");
      break;

    case 49: //1 Over Voltage Setpoint
      if (Serial.available() > 0)
      {
        settings.OverVSetpoint = Serial.parseInt();
        settings.OverVSetpoint = settings.OverVSetpoint / 1000;
        SERIALCONSOLE.print(settings.OverVSetpoint * 1000, 0);
        SERIALCONSOLE.print("mV Over Voltage Setpoint");
      }
      break;

    case 50: //2 Under Voltage Setpoint
      if (Serial.available() > 0)
      {
        settings.UnderVSetpoint = Serial.parseInt();
        settings.UnderVSetpoint = settings.UnderVSetpoint / 1000;
        SERIALCONSOLE.print(settings.UnderVSetpoint * 1000, 0);
        SERIALCONSOLE.print("mV Over Voltage Setpoint");
      }
      break;

    case 51: //3 Over Temperature Setpoint
      if (Serial.available() > 0)
      {
        settings.OverTSetpoint = Serial.parseInt();
        SERIALCONSOLE.print(settings.OverTSetpoint);
        SERIALCONSOLE.print(" Over Temperature Setpoint");
      }
      break;

    case 52: //4 Udner Temperature Setpoint
      if (Serial.available() > 0)
      {
        settings.UnderTSetpoint = Serial.parseInt();
        SERIALCONSOLE.print(settings.UnderTSetpoint);
        SERIALCONSOLE.print(" Under Temperature Setpoint");
      }
      break;

    case 53: //5 Balance Voltage Setpoint
      if (Serial.available() > 0)
      {
        settings.BalanceV = Serial.parseInt();
        settings.BalanceV = settings.BalanceV / 1000;
        SERIALCONSOLE.print(settings.BalanceV * 1000, 0);
        SERIALCONSOLE.print("mV Balance Voltage Setpoint");
      }
      break;

    case 54: //6 Balance Voltage Hystersis
      if (Serial.available() > 0)
      {
        settings.BalanceVHyst = Serial.parseInt();
        settings.BalanceVHyst = settings.BalanceVHyst / 1000;
        SERIALCONSOLE.print(settings.BalanceVHyst * 1000, 0);
        SERIALCONSOLE.print("mV Balance Voltage Hystersis");
      }
      break;
    }
  }

  if (status.menuload == 1)
  {
    switch (incomingByte)
    {
    case 113: //q to go back to main menu
      status.menuload = 0;
      break;

    case 99: //c for calibrate zero offset
      SERIALCONSOLE.println();
      SERIALCONSOLE.println();
      SERIALCONSOLE.println();
      SERIALCONSOLE.println();
      SERIALCONSOLE.println();
      SERIALCONSOLE.println("Current Sensor Calibration Menu");
      SERIALCONSOLE.println("c - To calibrate sensor offset");
      SERIALCONSOLE.println("s - To switch between Current Sensors");
      SERIALCONSOLE.println("q - Go back to menu");
      status.menuload = 2;
      break;

    case 98: //c for calibrate zero offset
      SERIALCONSOLE.println();
      SERIALCONSOLE.println();
      SERIALCONSOLE.println();
      SERIALCONSOLE.println();
      SERIALCONSOLE.println();
      SERIALCONSOLE.println("Battery Settings Menu");
      SERIALCONSOLE.println("r - Reset AH counter");
      SERIALCONSOLE.println("d - Display settings");
      SERIALCONSOLE.println("e - Edit settings");
      SERIALCONSOLE.println("q - Go back to menu");
      status.menuload = 3;
      break;

    default:
      // if nothing else matches, do the default
      // default is optional
      break;
    }
  }

  if (incomingByte == 115 & status.menuload == 0)
  {
    SERIALCONSOLE.println();
    SERIALCONSOLE.println("MENU");
    SERIALCONSOLE.println("Debugging Paused");
    SERIALCONSOLE.println("c - Current Sensor Calibration");
    SERIALCONSOLE.println("b - Battery Settings");
    SERIALCONSOLE.println("q - exit menu");
    status.menuload = 1;
  }
}
    */
}
