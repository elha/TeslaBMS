#include <Arduino.h>
#include <FlexCAN.h>

#include "Logger.h"
#include "config.h"
#include "BMSModuleManager.h"
BMSSettings settings;
BMSStatus status;
BMSModuleManager bms;

#include <ADC.h>
#include <DMAChannel.h>
#include <array>
ADC adc;
DMAChannel dma;
std::array<volatile uint16_t, 4096> buffer;

void setup()
{
  setup_settings();
  setup_bus();

  delay(4000);

  setup_adc();
  setup_bms();
}

void loop()
{
  loop_readcan();

  static unsigned long loop1;  
  if( checkinterval(loop1, 500) )
  {
    // input
    loop_querycurrent();
    loop_querybatt();

    // logic
    loop_bms();

    loop_console();

    // output
    loop_vecan();
  }

  static unsigned long loop2;  
  if( checkinterval(loop2, 10000) )
  {
    // calc
    loop_calc();

    // contactor
    loop_contactor();

    // console output
    loop_console();
  }

  static unsigned long loop3;  
  if( checkinterval(loop3, 3600000) )
  {
    // clear com errors
    loop_comerror();
  }
}

// setup ###############
void setup_settings()
{
  Logger::console("Loading defaults");
  settings.version = 1;

  settings.ConfigBattParallelCells = 74;
  settings.ConfigBattSerialCells   = 12;
  
  settings.UCellWarnMin = 3.20f;
  settings.UCellNormMin = 3.25f;
  settings.UCellOptiMin = 3.30f;

  settings.UCellOptiMax = 4.00f;
  settings.UCellNormMax = 4.06f;
  settings.UCellWarnMax = 4.10f;

  settings.UBattNormMin = settings.UCellNormMin * (float)settings.ConfigBattSerialCells; 
  settings.UBattNormMax = settings.UCellNormMax * (float)settings.ConfigBattSerialCells;

  settings.UCellNormBalanceDiff = 0.03f;
  settings.UCellWarnBalanceDiff = 0.06f;

  settings.IBattWarnChargeMax    = 31.0f;
  settings.IBattWarnDischargeMax = 31.0f;
  settings.IBattOptiChargeMax    = 28.0f;
  settings.IBattOptiDischargeMax = 28.0f;

  settings.TBattNormMin = 15.0f;
  settings.TBattNormMax = 32.0f;

  settings.QBattNormMin = getQCellSpec(settings.UCellNormMin) * (float)settings.ConfigBattParallelCells;
  settings.QBattNormMax = getQCellSpec(settings.UCellNormMax) * (float)settings.ConfigBattParallelCells;
  settings.QBattNorm    = settings.QBattNormMax - settings.QBattNormMin;
  settings.QBattNormKwh = getQBattNorm(settings.UCellNormMax);

  settings.ConstInCurrentOffset = 0.0f; // will be set on first measurement
  settings.ConstInCurrentSampleFreq = 256;
  
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
  pinMode(PRELOAD, OUTPUT);
  digitalWrite(PRELOAD, LOW);
  pinMode(CONTACTOR, OUTPUT);
  digitalWrite(CONTACTOR, LOW);  
  
  bms.renumberBoardIDs();
  bms.findBoards();
  bms.clearFaults();

  status.SohBattCurr = 1.0f;
}

void setup_adc()
{
  pinMode(INCURRENT, INPUT);
  
  adc.setResolution(14);
  adc.setConversionSpeed(ADC_CONVERSION_SPEED::LOW_SPEED);
  adc.setSamplingSpeed(ADC_SAMPLING_SPEED::LOW_SPEED);
  adc.adc0->analogRead(INCURRENT); // performs various ADC setup stuff

  dma.source(ADC0_RA); // ADC result register
  dma.transferSize(2);
  dma.triggerAtHardwareEvent(DMAMUX_SOURCE_ADC0);
  dma.destinationBuffer(buffer.data(), buffer.size() * sizeof(buffer[0]));
  dma.enable();

  adc.enableDMA(ADC_0);
  
  adc.adc0->stopPDB();
  adc.adc0->startPDB(settings.ConstInCurrentSampleFreq); 
  NVIC_DISABLE_IRQ(IRQ_PDB); // we don't want or need the PDB interrupt
}

void pdb_isr(void) {
  PDB0_SC &=~PDB_SC_PDBIF; // clear interrupt, called once, bad combination with ADC-library
}

// loop ###############
void loop_querycurrent()
{
  if(adc.adc0->fail_flag) {
    Logger::error("ADC FAIL %l", adc.adc0->fail_flag);
  }

  static size_t lastidx = 0;
  size_t idx = ((uint16_t*) dma.destinationAddress()) - buffer.data();

  if(idx<lastidx) idx += buffer.size();
  
  if((idx - lastidx) < settings.ConstInCurrentSampleFreq)
  {
    return;
  }

  // average last second
  double value = 0;
  for(uint16_t i = 0; i < settings.ConstInCurrentSampleFreq; i++) {
    value += (double)buffer[lastidx++];
    if(lastidx==buffer.size()) lastidx = 0;
  }
  
  static const double maxvalue = pow(2,16)-1;
  value /= (double)settings.ConstInCurrentSampleFreq;
  value -= maxvalue * 0.5;
  
  // ACS758xCB 50A bidirectional, 50% = 0A, 40mV/A @ 5V = 40/5000 = 1/125
  status.IBattCurr = (value / maxvalue) * 125.0l + (double)settings.ConstInCurrentOffset;
  if(settings.ConstInCurrentOffset == 0) settings.ConstInCurrentOffset = -(double)status.IBattCurr;
  
  if(status.IBattCurr >= 0)
  {
    status.IBattCurrCharge    =  0.0f;
    status.IBattCurrDischarge = +status.IBattCurr;
  }
  else
  {
    status.IBattCurrCharge    = -status.IBattCurr;
    status.IBattCurrDischarge = 0.0f;
  }

  status.QBattMeasuredKwh += status.IBattCurr / (double)3600.0 /(double)1000.0 * (double)status.UCellCurrAvg * (double)settings.ConfigBattSerialCells;
}

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

  if (bms.getCommunicationErrors() > status.CBmsWarnErrors)
  {
    status.Error |= ERROR_INTERNALFAILURE;
    Logger::error("Alarm - bms communication errors");    
  }

  // shut off charging/discharging on Warning or Error
  if (status.Error != 0 || status.Alarm != 0)
  {
    status.IBattPlanChargeMax = 0;
    status.IBattPlanDischargeMax = 0;
  }
}

void loop_contactor()
{
  
  if (status.State > 0 && status.Error != 0) 
  {
    digitalWrite(PRELOAD, LOW);
    digitalWrite(CONTACTOR, LOW);  
    status.State = 0;
    Logger::error("Contactor - disconnect on alarm");  
  } 
  else if (status.State == 0 && status.Error == 0 && status.Alarm == 0) //  start preloading soon if normal state
  {
    status.State = 1;
    Logger::info("Contactor - preload starts soon");  
  }
  else if (status.State == 1 && status.Error == 0 && status.Alarm == 0) //  start preloading if normal state
  {
    digitalWrite(PRELOAD, HIGH);
    status.State = 2;
    Logger::info("Contactor - preload");  
  }
  else if (status.State == 2 && abs(status.IBattCurr) < 0.30)  // close Contactor when current is minimal
  {    
    digitalWrite(CONTACTOR, HIGH);
    status.State = 3;
    Logger::info("Contactor - closed");  
  }
  else if (status.State == 3)  // open preload
  {    
    digitalWrite(PRELOAD, LOW);
    status.State = 4;
    Logger::info("Contactor - closed, preload released");  
  }
  else if ((status.State == 4 || status.State == 5) && status.IBattCurr > 0.0f)  // charging -> discharging
  {    
    status.State = 6;
    status.QBattStartCycle = status.QBattCurrKwh;
    status.QBattMeasuredKwh = 0.0f;
    Logger::info("Contactor - discharging");  
  }
  else if ((status.State == 4 || status.State == 6) && status.IBattCurr < 0.0f)  // discharging -> charging
  {    
    status.State = 5;
    status.QBattStartCycle = status.QBattCurrKwh;
    status.QBattMeasuredKwh = 0.0f;
    Logger::info("Contactor - charging");  
  }

}

void loop_calc()
{
  // QBattNorm           = 100% Capacity within Norm-range [Ah]
  // QBattCurr           = current Capacity [Ah]
  // OCV Method: Mapping UCellCurrAvg to discharge curve: 0%-100% = Norm-Range not Spec-Range
  status.QBattCurr = getQCellSpec(status.UCellCurrAvg) * (float)settings.ConfigBattParallelCells - settings.QBattNormMin;  
  status.QBattCurrKwh = getQBattNorm(status.UCellCurrAvg);

  status.SocBattCurr = status.QBattCurrKwh / settings.QBattNormKwh;
  if(status.SocBattCurr < 0.0f) status.SocBattCurr = 0.0f;
  if(status.SocBattCurr > 1.0f) status.SocBattCurr = 1.0f;
  
  // SohBattCurr = QBattCurr / QBattNorm
  if(abs(status.QBattMeasuredKwh)>0.2f && abs(status.QBattStartCycle - status.QBattCurrKwh)>0.2f)
    status.SohBattCurr = abs(status.QBattMeasuredKwh) / abs(status.QBattStartCycle - status.QBattCurrKwh);  
}

void loop_console()
{
  Logger::info("Qm=%fkWh Q=%fAh/%fAh Q=%fkWh/%fkWh SOC=%f SOH=%f I=%fA UBatt=%fV UCell=%fV UCellDelta=%fV T=%fC", 
                        status.QBattMeasuredKwh,
                        status.QBattCurr, settings.QBattNorm,
                        status.QBattCurrKwh, settings.QBattNormKwh, 
                        status.SocBattCurr * 100.0f, status.SohBattCurr * 100.0f,
                        status.IBattCurr, status.UBattCurr, status.UCellCurrAvg, status.UCellCurrDelta,
                        status.TBattCurrMax );

  if (settings.logLevel > Logger::Debug) return;

  bms.printPackDetails();
}

void loop_vecan() // communication with Victron system over CAN
{
  // http://www.rec-bms.com/datasheet/UserManual_REC_Victron_BMS.pdf Page 10
  CAN_message_t msg;

  msg.ext = 0;
  msg.id = 0x351;
  msg.len = 8;
  msg.buf[0] = lowByte(uint16_t(settings.UBattNormMax * 10.0f));
  msg.buf[1] = highByte(uint16_t(settings.UBattNormMax * 10.0f));
  msg.buf[2] = lowByte(uint16_t(status.IBattPlanChargeMax * 10.0f));
  msg.buf[3] = highByte(uint16_t(status.IBattPlanChargeMax * 10.0f));
  msg.buf[4] = lowByte(uint16_t(status.IBattPlanDischargeMax * 10.0f));
  msg.buf[5] = highByte(uint16_t(status.IBattPlanDischargeMax * 10.0f));
  msg.buf[6] = lowByte(uint16_t(settings.UBattNormMin * 10.0f));
  msg.buf[7] = highByte(uint16_t(settings.UBattNormMin * 10.0f));
  Logger::debug("VECan %i %i", msg.id, msg.buf[0]);
  CANVE.write(msg);

  msg.ext = 0;
  msg.id = 0x355;
  msg.len = 8;
  msg.buf[0] = lowByte(uint16_t(status.SocBattCurr * 100.0f));
  msg.buf[1] = highByte(uint16_t(status.SocBattCurr * 100.0f));
  msg.buf[2] = lowByte(uint16_t(status.SohBattCurr * 100.0f));
  msg.buf[3] = highByte(uint16_t(status.SohBattCurr * 100.0f));
  msg.buf[4] = lowByte(uint16_t(status.SocBattCurr * 10000.0f));
  msg.buf[5] = highByte(uint16_t(status.SocBattCurr * 10000.0f));
  msg.buf[6] = 0;
  msg.buf[7] = 0;
  Logger::debug("VECan %i %i", msg.id, msg.buf[0]);
  CANVE.write(msg);

  msg.ext = 0;
  msg.id = 0x356;
  msg.len = 8;
  msg.buf[0] = lowByte(int16_t(status.UBattCurr * 100.0f));
  msg.buf[1] = highByte(int16_t(status.UBattCurr * 100.0f));
  msg.buf[2] = lowByte(int16_t(status.IBattCurr * -10.0f));
  msg.buf[3] = highByte(int16_t(status.IBattCurr * -10.0f));
  msg.buf[4] = lowByte(int16_t(status.TBattCurrMax * 10.0f));
  msg.buf[5] = highByte(int16_t(status.TBattCurrMax * 10.0f));
  msg.buf[7] = 0;
  msg.buf[8] = 0;
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
  msg.buf[0] = 'T';
  msg.buf[1] = 'e';
  msg.buf[2] = 's';
  msg.buf[3] = 'l';
  msg.buf[4] = 'a';
  msg.buf[5] = 'B';
  msg.buf[6] = 'm';
  msg.buf[7] = 's';
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
  if (CANVE.available())
  {
    CANVE.read(msgin);
    Logger::debug("VECan read %i %i", msgin.id, msgin.buf[0]);
  }
}

void loop_comerror()
{
  bms.resetCommunicationErrors();
}

// helper functions
float getQCellSpec(float UCellCurr)
{
    for (int x = 1; x < SizeCellSpecCurve0_2C; x++)
    {
      double UCellSpecH = QCellSpecCurve0_2C[x  ][0];

      if(UCellSpecH > UCellCurr)
      {
        double UCellSpecL = QCellSpecCurve0_2C[x-1][0];
        double UCellSpecDiff = UCellSpecH - UCellSpecL;

        double QCellSpecL = QCellSpecCurve0_2C[x-1][1];
        double QCellSpecH = QCellSpecCurve0_2C[x  ][1];
        double QCellSpecDiff = QCellSpecH - QCellSpecL;
        
        double UCellCurrDiff = UCellCurr  - UCellSpecL;

        // linear interpolation
        return (QCellSpecL + (UCellCurrDiff / UCellSpecDiff * QCellSpecDiff)) * 0.001l;        
      }
    }
    return 100.0f;
}

float getQBattNorm(float UCellCurr)
{
    static const double step = 0.002f;
    double out = 0.0l;
    double last = getQCellSpec(settings.UCellNormMin);
    for (float x = settings.UCellNormMin; x <= UCellCurr; x+=step)
    {
      float QCurr = getQCellSpec(x);
      out += x * (QCurr - last);
      last = QCurr;
    }
    return out * 0.001l * (double)settings.ConfigBattSerialCells * (double)settings.ConfigBattParallelCells;
}

byte checkinterval(unsigned long &loop_PreviousMillis, unsigned long loop_Interval) 
{
  if(millis() - loop_PreviousMillis > loop_Interval)
  {
    loop_PreviousMillis = millis();
    return -1;
  }
  else    
  {
    return 0;
  }
}
