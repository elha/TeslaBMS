#include <Arduino.h>
#include <FlexCAN.h>

#include "Logger.h"
#include "config.h"
#include "BMSModuleManager.h"
#include "SerialConsole.h"
BMSSettings settings;
BMSStatus status;
BMSModuleManager bms;
SerialConsole console;


void setup() {
  setup_settings();
  setup_bus();

  delay(4000);

  setup_bms();
  
  delay(4000);
}

void loop() {
  loop_readcan();
  loop_menu();

  static unsigned long looptime;
  if (millis() - looptime > 500) {
    looptime = millis();
    
    // inputs
    loop_querybms();
    
    // calculations
    loop_calcsoc();
    loop_alarm();

    // state machine
    loop_bms();

    // output
    loop_vecan();
    loop_debug();
  }
}

void loop_alarm() {
  status.alarm[0] = 0;
  if (bms.getHighCellVolt() > settings.OverVSetpoint) {
    status.alarm[0] = 0x04;
    Logger::warn("Alarm - high cell voltage %f over %f", bms.getHighCellVolt(),
                 settings.OverVSetpoint);
  }

  if (bms.getLowCellVolt() < settings.UnderVSetpoint) {
    status.alarm[0] |= 0x10;
    Logger::warn("Alarm - low cell voltage %f under %f", bms.getLowCellVolt(),
                 settings.UnderVSetpoint);
  }

  if (bms.getAvgTemperature() > settings.OverTSetpoint) {
    status.alarm[0] |= 0x40;
    Logger::warn("Alarm - high battery temp %f over %f",
                 bms.getAvgTemperature(), settings.OverTSetpoint);
  }

  status.alarm[1] = 0;
  if (bms.getAvgTemperature() < settings.UnderTSetpoint) {
    status.alarm[1] = 0x01;
    Logger::warn("Alarm - low battery temp %f  under %f",
                 bms.getAvgTemperature(), settings.UnderTSetpoint);
  }
  if (false) {
    status.alarm[1] = 0x40;
    Logger::warn("Alarm - high discharge current");
  }

  status.alarm[2] = 0;
  if (false) {
    status.alarm[2] = 0x01;
    Logger::warn("Alarm - high charge current");
  }
  
  if (false) {
    status.alarm[2] = 0x40;
    Logger::warn("Alarm - internal failure");
  }

  status.alarm[3] = 0;
  if (false) {
    status.alarm[3] = 0x01;
    Logger::warn("Alarm - cell imbalance");
  }
}

// setup ###############

void setup_settings() {
  Logger::console("Loading defaults");
  settings.version = 1;
  settings.OverVSetpoint = 4.1f;
  settings.UnderVSetpoint = 3.0f;
  
  settings.OverTSetpoint = 65.0f;
  settings.UnderTSetpoint = 10.0f;
  
  settings.BalanceV = 3.9f;
  settings.BalanceVHyst = 0.04f;

  settings.logLevel = Logger::Info;

  settings.ChargeVMax = 48.0f;
  settings.DischargeVMin = 40.0f;
  settings.ChargeIMax = 2.0f;
  settings.DischargeIMax = 2.0f;

  settings.SOC10V = 3.1f;
  settings.SOC90V = 4.1f;
}

void setup_bus() {
  CANVE.begin(500000);          // VE.Can to CCGX
  SERIALCONSOLE.begin(115200); // USB serial
  SERIALBMS.begin(612500);     // Tesla serial bus

  Logger::setLoglevel((Logger::LogLevel)settings.logLevel);
  Logger::console("Initializing ...");
}

void setup_bms() {
  bms.renumberBoardIDs();
  bms.findBoards();
  bms.clearFaults();
}

// loop ###############

void loop_querybms() { bms.getAllVoltTemp(); }

void loop_bms() {
  status.CurChargeIMax = 0.01; //settings.ChargeIMax;
  status.CurDischargeIMax = 0.01 ;//settings.DischargeIMax;
  
  if (bms.getLowCellVolt() < settings.UnderVSetpoint) {
    status.bmsstatus = Err;
  }

  switch (status.bmsstatus) {
  case (Boot):
    status.bmsstatus = Ready;
    break;

  case (Ready):
    status.bmsstatus = Charge;
    break;

  case (Drive):
    break;

  case (Charge):
    if (bms.getHighCellVolt() > settings.BalanceV)
      if(bms.balanceCells(settings.BalanceVHyst))
        Logger::info("Balancing Pack");

    if (bms.getHighCellVolt() > settings.OverVSetpoint)
      status.bmsstatus = Ready;

    break;

  case (Err):
    if (bms.getLowCellVolt() >= settings.UnderVSetpoint)
      status.bmsstatus = Ready;

    break;
  }
}

void loop_debug() {
  if (settings.logLevel > Logger::Info)
    return;

  SERIALCONSOLE.println();
  SERIALCONSOLE.println();
  SERIALCONSOLE.println();
  SERIALCONSOLE.print("BMS Status : ");
  SERIALCONSOLE.print(status.bmsstatus);
  switch (status.bmsstatus) {
  case (Boot):
    SERIALCONSOLE.print(" Boot ");
    break;

  case (Ready):
    SERIALCONSOLE.print(" Ready ");
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

void loop_calcsoc() {
  status.SOC = map(uint16_t(bms.getAvgCellVolt() * 1000),
                   uint16_t(settings.SOC10V * 1000),
                   uint16_t(settings.SOC90V * 1000), 10, 90);
}

void loop_vecan() // communication with Victron system over CAN
{
  // http://www.rec-bms.com/datasheet/UserManual_REC_Victron_BMS.pdf Page 10
  CAN_message_t msg;

  msg.ext = 0;
  msg.id = 0x351;
  msg.len = 8;
  msg.buf[0] = lowByte(uint16_t(settings.ChargeVMax * 10));
  msg.buf[1] = highByte(uint16_t(settings.ChargeVMax * 10));
  msg.buf[2] = lowByte(uint16_t(status.CurChargeIMax * 10)); 
  msg.buf[3] = highByte(uint16_t(status.CurChargeIMax * 10));
  msg.buf[4] = lowByte(uint16_t(status.CurDischargeIMax * 10));
  msg.buf[5] = highByte(uint16_t(status.CurDischargeIMax * 10));
  msg.buf[6] = lowByte(uint16_t(settings.DischargeVMin * 10));
  msg.buf[7] = highByte(uint16_t(settings.DischargeVMin * 10));
  Logger::debug("VECan %i %i", msg.id, msg.buf[0]);
  CANVE.write(msg);

  msg.ext = 0;
  msg.id = 0x355;
  msg.len = 8;
  msg.buf[0] = lowByte(status.SOC);
  msg.buf[1] = highByte(status.SOC);
  msg.buf[2] = lowByte(status.SOH);
  msg.buf[3] = highByte(status.SOH);
  msg.buf[4] = lowByte(status.SOC * 100);
  msg.buf[5] = highByte(status.SOC * 100);
  msg.buf[6] = 0;
  msg.buf[7] = 0;
  Logger::debug("VECan %i %i", msg.id, msg.buf[0]);
  CANVE.write(msg);

  msg.ext = 0;
  msg.id = 0x356;
  msg.len = 8;
  msg.buf[0] = lowByte(uint16_t(bms.getPackVoltage() * 100));
  msg.buf[1] = highByte(uint16_t(bms.getPackVoltage() * 100));
  msg.buf[2] = lowByte(long(status.CurI / 100));
  msg.buf[3] = highByte(long(status.CurI / 100));
  msg.buf[4] = lowByte(uint16_t(bms.getAvgTemperature() * 10));
  msg.buf[5] = highByte(uint16_t(bms.getAvgTemperature() * 10));
  msg.buf[6] = 0;
  msg.buf[7] = 0;
  Logger::debug("VECan %i %i", msg.id, msg.buf[0]);
  CANVE.write(msg);

  msg.ext = 0;
  msg.id = 0x35A;
  msg.len = 8;
  msg.buf[0] = status.alarm[0]; // High temp  | Low Voltage | High Voltage
  msg.buf[1] = status.alarm[1]; // High Discharge Current | Low Temperature
  msg.buf[2] = status.alarm[2]; // Internal Failure | High Charge current
  msg.buf[3] = status.alarm[3]; // Cell Imbalance
  msg.buf[4] = status.warn[0];  // bits identical to alarm
  msg.buf[5] = status.warn[1];
  msg.buf[6] = status.warn[2];
  msg.buf[7] = status.warn[3];
  Logger::debug("VECan %i %i", msg.id, msg.buf[0]);
  CANVE.write(msg);

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
  Logger::debug("VECan %i %i", msg.id, msg.buf[0]);
  CANVE.write(msg);

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
  Logger::debug("VECan %i %i", msg.id, msg.buf[0]);
  CANVE.write(msg);
}

void loop_menu() { console.loop(); }

void loop_readcan() {
  CAN_message_t msgin;
  if (!CANVE.available())
      CANVE.read(msgin);


  // status.CurI = CANmilliamps / 1000;
}
