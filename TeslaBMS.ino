#include <Arduino.h>
#include <FlexCAN.h>

#include "Logger.h"
#include "config.h"
#include "BMSModuleManager.h"
BMSSettings settings;
BMSStatus status;
BMSModuleManager bms;

void setup()
{
  setup_settings();
  setup_bus();

  delay(4000);

  setup_bms();

  delay(4000);
}

void loop()
{
  loop_readcan();

  static unsigned long looptime;
  if (millis() - looptime > 500)
  {
    looptime = millis();

    // input
    loop_querybatt();

    // logic
    loop_bms();

    // output
    loop_vecan();
    loop_debug();
  }
}

// setup ###############

void setup_settings()
{
  Logger::console("Loading defaults");
  settings.version = 1;

  settings.UCellWarnMin = 3.20f;
  settings.UCellNormMin = 3.30f;
  settings.UCellOptiMin = 3.40f;

  settings.UCellOptiMax = 3.90f;
  settings.UCellNormMax = 4.00f;
  settings.UCellWarnMax = 4.10f;

  settings.UBattNormMin = settings.UCellNormMin * 12.0f; // 12s74p
  settings.UBattNormMax = settings.UCellNormMax * 12.0f;

  settings.UCellNormBalanceDiff = 0.03f;
  settings.UCellWarnBalanceDiff = 0.06f;

  settings.IBattWarnChargeMax = 4.0f;
  settings.IBattWarnDischargeMax = 4.0f;
  settings.IBattOptiChargeMax = 1.5f;
  settings.IBattOptiDischargeMax = 1.5f;

  settings.TBattNormMin = 10.0f;
  settings.TBattNormMax = 45.0f;

  settings.logLevel = Logger::Info;
}

void setup_bus()
{
  SERIALCONSOLE.begin(115200); // USB serial
  SERIALBMS.begin(612500);     // Tesla serial bus
  CANVE.begin(500000);         // VE.Can to CCGX

  Logger::setLoglevel((Logger::LogLevel)settings.logLevel);
  Logger::console("Initializing ...");
}

void setup_bms()
{
  bms.renumberBoardIDs();
  bms.findBoards();
  bms.clearFaults();
}

// loop ###############

void loop_querybatt()
{
  bms.getAllVoltTemp();
  status.UBattCurr = bms.getPackVoltage();
  
  status.UCellCurrMin = bms.getLowCellVolt();
  status.UCellCurrAvg = bms.getAvgCellVolt();
  status.UCellCurrMax = bms.getHighCellVolt();
  status.UCellCurrDelta = status.UCellCurrMax - status.UCellCurrMin;
  
  status.TBattCurrMin = bms.getLowTemperature();
  status.TBattCurrMax = bms.getHighTemperature();

  status.SocBattCurr = (float)map(uint16_t(status.UCellCurrAvg * 1000),
                          uint16_t(settings.UCellNormMin * 1000),
                          uint16_t(settings.UCellNormMax * 1000), 0, 10000) * 0.01f;
  status.SohBattCurr = 100.0f;

  status.IBattCurr = 2.15f; // TODO
}

void loop_bms()
{
  // Charging and Discharging in 4 stages
  // is opti: Full Dis-/Charge
  // or is norm: linear Dis-/Charge from Full to zero
  // or is warn: zero Dis-/Charge and warning
  // otherwise is error: zero Dis-/Charge and error

  status.Alarm = ERROR_NONE;
  status.Error = ERROR_NONE;

  if (status.UCellCurrMax < settings.UCellOptiMax)
  {
    status.IBattPlanChargeMax = settings.IBattOptiChargeMax;
  }
  else if (status.UCellCurrMax < settings.UCellNormMax)
  {
    if (status.IBattCurrCharge > 0.1 * settings.IBattOptiChargeMax)
      if (bms.balanceCells(settings.UCellNormBalanceDiff))
        Logger::info("Balancing Pack");

    status.IBattPlanChargeMax = settings.IBattOptiChargeMax * (settings.UCellNormMax - status.UCellCurrMax) / (settings.UCellNormMax - settings.UCellOptiMax);
  }
  else if (status.UCellCurrMax < settings.UCellWarnMax)
  {
    if (status.IBattCurrCharge > 0.1 * settings.IBattOptiChargeMax)
      if (bms.balanceCells(settings.UCellNormBalanceDiff))
        Logger::info("Balancing Pack");

    status.IBattPlanChargeMax = 0;
    status.Alarm |= ERROR_HIGHCELLVOLTAGE;
  }
  else //if (status.UCellCurrMax < settings.UCellErrorMax) or worse
  {
    status.IBattPlanChargeMax = 0;
    status.Error |= ERROR_HIGHCELLVOLTAGE;
  }

  if (status.UCellCurrMin > settings.UCellOptiMin)
  {
    status.IBattPlanDischargeMax = settings.IBattOptiDischargeMax;
  }
  else if (status.UCellCurrMin > settings.UCellNormMin)
  {
    status.IBattPlanDischargeMax = settings.IBattOptiDischargeMax * (status.UCellCurrMin - settings.UCellNormMin) / (settings.UCellOptiMin - settings.UCellNormMin);
  }
  else if (status.UCellCurrMin > settings.UCellWarnMin)
  {
    status.IBattPlanDischargeMax = 0;
    status.Alarm |= ERROR_LOWCELLVOLTAGE;
  }
  else //if (status.UCellCurrMin > settings.UCellErrorMin) or worse
  {
    status.IBattPlanDischargeMax = 0;
    status.Error |= ERROR_LOWCELLVOLTAGE;
  }

  // check errors could implement warnings
  if (status.TBattCurrMax > settings.TBattNormMax)
  {
    status.Error |= ERROR_HIGHPACKTEMP;
    Logger::error("Alarm - high battery temp %f over %f", status.TBattCurrMax, settings.TBattNormMax);
  }

  if (status.TBattCurrMin < settings.TBattNormMin)
  {
    status.Error |= ERROR_LOWPACKTEMP;
    Logger::error("Alarm - low battery temp %f under %f", status.TBattCurrMin, settings.TBattNormMin);
  }

  if (status.IBattCurrDischarge > settings.IBattWarnDischargeMax)
  {
    status.Error |= ERROR_HIGHBATTDISCHARGECURRENT;
    Logger::error("Alarm - high discharge current %f over %f", status.IBattCurrDischarge, settings.IBattWarnDischargeMax);
  }

  if (status.IBattCurrCharge > settings.IBattWarnChargeMax)
  {
    status.Error |= ERROR_HIGHBATTCHARGECURRENT;
    Logger::error("Alarm - high charge current %f over %f", status.IBattCurrCharge, settings.IBattWarnChargeMax);
  }

  if (bms.isFaulted)
  {
    status.Error |= ERROR_INTERNALFAILURE;
    Logger::error("Alarm - internal failure");
  }

  if (status.UCellCurrDelta > settings.UCellWarnBalanceDiff)
  {
    status.Error |= ERROR_CELLIMBALANCE;
    Logger::error("Alarm - cell imbalance");
  }

  // shut off charging/discharging on Warning or Error
  if (status.Error != 0 || status.Alarm != 0)
  {
    status.IBattPlanChargeMax = 0;
    status.IBattPlanDischargeMax = 0;
  }
}

void loop_debug()
{
  if (settings.logLevel > Logger::Info)
    return;

  bms.printPackDetails();
}

void loop_vecan() // communication with Victron system over CAN
{
  // http://www.rec-bms.com/datasheet/UserManual_REC_Victron_BMS.pdf Page 10
  CAN_message_t msg;

  msg.ext = 0;
  msg.id = 0x351;
  msg.len = 8;
  msg.buf[0] = lowByte(uint16_t(settings.UBattNormMax * 10));
  msg.buf[1] = highByte(uint16_t(settings.UBattNormMax * 10));
  msg.buf[2] = lowByte(uint16_t(status.IBattPlanChargeMax * 10));
  msg.buf[3] = highByte(uint16_t(status.IBattPlanChargeMax * 10));
  msg.buf[4] = lowByte(uint16_t(status.IBattPlanDischargeMax * 10));
  msg.buf[5] = highByte(uint16_t(status.IBattPlanDischargeMax * 10));
  msg.buf[6] = lowByte(uint16_t(settings.UBattNormMin * 10));
  msg.buf[7] = highByte(uint16_t(settings.UBattNormMin * 10));
  Logger::debug("VECan %i %i", msg.id, msg.buf[0]);
  CANVE.write(msg);

  msg.ext = 0;
  msg.id = 0x355;
  msg.len = 8;
  msg.buf[0] = lowByte(uint16_t(status.SocBattCurr));
  msg.buf[1] = highByte(uint16_t(status.SocBattCurr));
  msg.buf[2] = lowByte(uint16_t(status.SohBattCurr));
  msg.buf[3] = highByte(uint16_t(status.SohBattCurr));
  msg.buf[4] = lowByte(uint16_t(status.SocBattCurr * 100));
  msg.buf[5] = highByte(uint16_t(status.SocBattCurr * 100));
  msg.buf[6] = 0;
  msg.buf[7] = 0;
  Logger::debug("VECan %i %i", msg.id, msg.buf[0]);
  CANVE.write(msg);

  msg.ext = 0;
  msg.id = 0x356;
  msg.len = 8;
  msg.buf[0] = lowByte(uint16_t(status.UBattCurr * 100));
  msg.buf[1] = highByte(uint16_t(status.UBattCurr * 100));
  msg.buf[2] = lowByte(uint16_t(status.IBattCurr * 10));
  msg.buf[3] = highByte(uint16_t(status.IBattCurr * 10));
  msg.buf[4] = lowByte(uint16_t(status.TBattCurrMax * 10));
  msg.buf[5] = highByte(uint16_t(status.TBattCurrMax * 10));
  msg.buf[6] = 0;
  msg.buf[7] = 0;
  Logger::debug("VECan %i %i", msg.id, msg.buf[0]);
  CANVE.write(msg);

  msg.ext = 0;
  msg.id = 0x35A;
  msg.len = 8;
  msg.buf[0] = (byte)((status.Error >>  0) & 0xFF); // High temp  | Low Voltage | High Voltage
  msg.buf[1] = (byte)((status.Error >>  8) & 0xFF); // High Discharge Current | Low Temperature
  msg.buf[2] = (byte)((status.Error >> 16) & 0xFF); // Internal Failure | High Charge current
  msg.buf[3] = (byte)((status.Error >> 24) & 0xFF); // Cell Imbalance
  msg.buf[4] = (byte)((status.Alarm >>  0) & 0xFF); // High temp  | Low Voltage | High Voltage
  msg.buf[5] = (byte)((status.Alarm >>  8) & 0xFF); // High Discharge Current | Low Temperature
  msg.buf[6] = (byte)((status.Alarm >> 16) & 0xFF); // Internal Failure | High Charge current
  msg.buf[7] = (byte)((status.Alarm >> 24) & 0xFF); // Cell Imbalance
  Logger::debug("VECan %i %i", msg.id, msg.buf[0]);
  CANVE.write(msg);

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

void loop_readcan()
{
  CAN_message_t msgin;
  if (!CANVE.available())
    CANVE.read(msgin);

  // status.IBattCurr = CANmilliamps / 1000;
  // status.IBattCurrCharge = CANmilliamps / 1000;
  // status.IBattCurrDischarge = CANmilliamps / 1000;
}
