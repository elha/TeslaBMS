#include "BMSModuleManager.h"
#include <Arduino.h>
#include "config.h"
#include "SerialConsole.h"
#include "Logger.h"
#include <FlexCAN.h>

BMSModuleManager bms;
SerialConsole console;
EEPROMSettings settings;

// status
int debug = 1;

byte bmsstatus = Boot;
uint16_t SOC = 100; 
uint16_t SOH = 100; 
float currentact;
unsigned long looptime;
int menuload = 0;

void setup()
{
  pinMode(INBMBFAULT, INPUT);
  setup_settings();

  delay(4000);  // wait for USB

  setup_bus();
  setup_bms();
}


void loop()
{
 loop_readcan();
 loop_menu();
 loop_bms();
  
  if (millis() - looptime > 500)
  {
    looptime = millis();

    loop_querybms();
    loop_debug();
    loop_calcsoc();
    loop_vecan();
  }
}

// setup ###############

void setup_settings()
{
  Logger::console("Loading defaults");
  settings.version = 0;
  settings.checksum = 0;
  settings.canSpeed = 500000;
  settings.batteryID = 0x01; //in the future should be 0xFF to force it to ask for an address
  settings.OverVSetpoint = 4.1f;
  settings.UnderVSetpoint = 3.0f;
  settings.OverTSetpoint = 65.0f;
  settings.UnderTSetpoint = -10.0f;
  settings.balanceVoltage = 3.9f;
  settings.balanceHyst = 0.04f;
  settings.logLevel = 3;
  settings.ChargeV = 48.0f;
  settings.DischargeV = 40.0f;
  settings.ChargeI = 20.0f;
  settings.DischargeI = 20.0f; 
  settings.SOC10V = 3.1f;
  settings.SOC90V = 4.1f;
}

void setup_bus()
{
  SERIALCONSOLE.begin(115200); // USB serial
  SERIALBMS.begin(612500);     // Tesla serial bus
  Can0.begin(500000);          // VE.Can to CCGX

  Logger::setLoglevel((Logger::LogLevel)settings.logLevel);
  Logger::console("Initializing ...");
}

void setup_bms()
{
  bms.renumberBoardIDs();
  bms.clearFaults();
  bms.findBoards();

}

// loop ###############

void loop_querybms()
{
    bms.getAllVoltTemp();
}

void loop_bms()
{
  if (bms.getLowCellVolt() < settings.UnderVSetpoint)
  {
    bmsstatus = Err;
  }

  switch (bmsstatus)
  {
    case (Boot):
      bmsstatus = Ready;
      break;

    case (Ready):
      if (bms.getHighCellVolt() > settings.balanceVoltage);
      {
        bms.balanceCells();
      }
      if ((settings.balanceVoltage + settings.balanceHyst) > bms.getHighCellVolt()) //detect AC present for charging and check not balancing
      {
        bmsstatus = Charge;
      }
      break;

    case (Precharge):
      break;

    case (Drive):
      break;

    case (Charge):
      if (bms.getHighCellVolt() > settings.balanceVoltage);
      {
        bms.balanceCells();
      }
      if (bms.getHighCellVolt() > settings.OverVSetpoint);
      {
        bmsstatus = Ready;
      }
      break;

    case (Err):
      if (bms.getLowCellVolt() >= settings.UnderVSetpoint);
      {
        bmsstatus = Ready;
      }

      break;
  }

}

void loop_debug()
{
  if (debug == 0) return;
  SERIALCONSOLE.println();
  SERIALCONSOLE.println();
  SERIALCONSOLE.println();
  SERIALCONSOLE.print("BMS Status : ");
  SERIALCONSOLE.print(bmsstatus);
  switch (bmsstatus)
  {
    case (Boot):
      SERIALCONSOLE.print(" Boot ");
      break;

    case (Ready):
      SERIALCONSOLE.print(" Ready ");
      break;

    case (Precharge):
      SERIALCONSOLE.print(" Precharge ");
      break;

    case (Drive):
      SERIALCONSOLE.print(" Drive ");
      break;

    case (Charge):
      SERIALCONSOLE.print(" Charge ");
      break;

    case (Err):
      SERIALCONSOLE.print(" Error ");
      break;
  }
  SERIALCONSOLE.print("  ");
  bms.printPackDetails();
}


void loop_calcsoc()
{
    SOC = map(uint16_t(bms.getAvgCellVolt() * 1000), uint16_t(settings.SOC10V * 1000), uint16_t(settings.SOC90V * 1000), 10, 90);
}


void loop_vecan() //communication with Victron system over CAN
{
  static CAN_message_t msg;

  msg.ext = 0;
  msg.id = 0x351;
  msg.len = 8;
  msg.buf[0] = lowByte(uint16_t(settings.ChargeV * 100));
  msg.buf[1] = highByte(uint16_t(settings.ChargeV * 100));
  msg.buf[2] = lowByte(uint16_t(settings.ChargeI * 100));
  msg.buf[3] = highByte(uint16_t(settings.ChargeI * 100));
  msg.buf[4] = lowByte(uint16_t(settings.DischargeI * 100));
  msg.buf[5] = highByte(uint16_t(settings.DischargeI * 100));
  msg.buf[6] = lowByte(uint16_t(settings.DischargeV * 100));
  msg.buf[7] = highByte(uint16_t(settings.DischargeV * 100));

  Can0.write(msg);
  
  msg.ext = 0;
  msg.id = 0x355;
  msg.len = 8;
  msg.buf[0] = lowByte(SOC);
  msg.buf[1] = highByte(SOC);
  msg.buf[2] = lowByte(SOH);
  msg.buf[3] = highByte(SOH);
  msg.buf[4] = lowByte(SOC * 10);
  msg.buf[5] = highByte(SOC * 10);
  msg.buf[6] = 0;
  msg.buf[7] = 0;

  Can0.write(msg);

  msg.ext = 0;
  msg.id = 0x356;
  msg.len = 8;
  msg.buf[0] = lowByte(uint16_t(bms.getPackVoltage() * 100));
  msg.buf[1] = highByte(uint16_t(bms.getPackVoltage() * 100));
  msg.buf[2] = lowByte(long(currentact / 100));
  msg.buf[3] = highByte(long(currentact / 100));
  msg.buf[4] = lowByte(uint16_t(bms.getAvgTemperature() * 10));
  msg.buf[5] = highByte(uint16_t(bms.getAvgTemperature() * 10));
  msg.buf[6] = 0;
  msg.buf[7] = 0;

  Can0.write(msg);

  msg.ext = 0;
  msg.id = 0x35A;
  msg.len = 8;
  msg.buf[0] = 0;
  msg.buf[1] = 0;
  msg.buf[2] = 0;
  msg.buf[3] = 0;
  msg.buf[4] = 0;
  msg.buf[5] = 0;
  msg.buf[6] = 0;
  msg.buf[7] = 0;

  Can0.write(msg);

  delay(5);

  // bmsname
  msg.ext = 0;
  msg.id = 0x35E;
  msg.len = 8;
  msg.buf[0] = 'S';
  msg.buf[1] = 'I';
  msg.buf[2] = 'M';
  msg.buf[3] = 'P';
  msg.buf[4] = ' ';
  msg.buf[5] = 'B';
  msg.buf[6] = 'M';
  msg.buf[7] = 'S';

  Can0.write(msg);
  
  delay(5);
  
  // bmsmanufacturer
  msg.ext = 0;
  msg.id = 0x370;
  msg.len = 8;
  msg.buf[0] = 'T';
  msg.buf[1] = 'O';
  msg.buf[2] = 'M';
  msg.buf[3] = ' ';
  msg.buf[4] = 'D';
  msg.buf[5] = 'E';
  msg.buf[6] = ' ';
  msg.buf[7] = 'B';

  Can0.write(msg);

}

void loop_menu()
{
  if (SERIALCONSOLE.available() == 0) return;
  
  byte incomingByte = Serial.read(); // read the incoming byte:

  if (menuload == 2)
  {
    switch (incomingByte)
    {


      case 113: //c for calibrate zero offset

        menuload = 0;
        incomingByte = 115;
        break;

      default:
        // if nothing else matches, do the default
        // default is optional
        break;
    }
  }
  if (menuload == 3)
  {
    switch (incomingByte)
    {
      case 113: //q to go back to main menu

        menuload = 0;
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
        SERIALCONSOLE.print(settings.balanceVoltage * 1000, 0);
        SERIALCONSOLE.print("mV Balance Voltage Setpoint - 5 ");
        SERIALCONSOLE.println("  ");
        SERIALCONSOLE.print(settings.balanceHyst * 1000, 0);
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
          settings.UnderVSetpoint =  settings.UnderVSetpoint / 1000;
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
          settings.balanceVoltage = Serial.parseInt();
          settings.balanceVoltage = settings.balanceVoltage / 1000;
          SERIALCONSOLE.print(settings.balanceVoltage * 1000, 0);
          SERIALCONSOLE.print("mV Balance Voltage Setpoint");
        }
        break;

      case 54: //6 Balance Voltage Hystersis
        if (Serial.available() > 0)
        {
          settings.balanceHyst = Serial.parseInt();
          settings.balanceHyst =  settings.balanceHyst / 1000;
          SERIALCONSOLE.print(settings.balanceHyst * 1000, 0);
          SERIALCONSOLE.print("mV Balance Voltage Hystersis");
        }
        break;

    }
  }

  if (menuload == 1)
  {
    switch (incomingByte)
    {
      case 113: //q to go back to main menu

        menuload = 0;
        debug = 1;
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
        menuload = 2;
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
        menuload = 3;
        break;

      default:
        // if nothing else matches, do the default
        // default is optional
        break;
    }
  }

  if (incomingByte == 115 & menuload == 0)
  {
    SERIALCONSOLE.println();
    SERIALCONSOLE.println("MENU");
    SERIALCONSOLE.println("Debugging Paused");
    SERIALCONSOLE.println("c - Current Sensor Calibration");
    SERIALCONSOLE.println("b - Battery Settings");
    SERIALCONSOLE.println("q - exit menu");
    debug = 0;
    menuload = 1;
  }
}

void loop_readcan()
{
  if (!Can0.available()) return;
  static CAN_message_t msgin;
  
  Can0.read(msgin);      // Read data: len = data length, buf = data byte(s)

  switch (msgin.id)
  {
    case 0x3c2:
      uint32_t inbox = 0;
    
      for (int i = 0; i < 4; i++)
      {
        inbox = (inbox << 8) | msgin.buf[i];
      }
      uint32_t CANmilliamps = inbox;
      if (CANmilliamps > 0x80000000)
      {
        CANmilliamps -= 0x80000000;
      }
      else
      {
        CANmilliamps = (0x80000000 - CANmilliamps) * -1;
      }
      currentact = CANmilliamps / 1000;
      break;

  }
}

