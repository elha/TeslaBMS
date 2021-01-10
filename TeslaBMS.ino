#include <Arduino.h>

#include "Logger.h"
#include "config.h"

#include <FlexCAN_T4.h>

FlexCAN_T4<CAN_DEV, RX_SIZE_256, TX_SIZE_16> CANVE;

unsigned char bmsname[8] = {'T', 'e', 's', 'l', 'a', 'B', 'M', 'S'};
unsigned char bmsmanu[8] = {'T', 'O', 'M', ' ', 'D', 'E', ' ', 'B'};
unsigned char bsmFWV[2]  = {1, 0};

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
  setup_led();
  setup_settings();
  setup_bus();

  delay(4000);

  setup_adc();
  setup_bms();
  setup_cooler();
}

void loop()
{
  static unsigned long loop1;  
  if( checkinterval(loop1, 500) )
  {
    // status LED
    loop_led();
    
    // input
    loop_querycurrent();
    loop_querybatt();

    // logic
    loop_bms();
    loop_cooler();

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

  settings.ConfigBattParallelCells   = 74;
  settings.ConfigBattSerialCells     = 12;
  settings.ConfigBattParallelStrings =  2;
  
  settings.UCellWarnMin = 3.30f;
  settings.UCellNormMin = 3.38f;
  settings.UCellOptiMin = 3.40f;

  settings.UCellOptiMax = 4.00f;
  settings.UCellNormMax = 4.06f;
  settings.UCellWarnMax = 4.10f;

  settings.UBattNormMin = settings.UCellNormMin * (float)settings.ConfigBattSerialCells; 
  settings.UBattNormMax = settings.UCellNormMax * (float)settings.ConfigBattSerialCells;

  settings.UCellNormBalanceDiff = 0.03f;
  settings.UCellWarnBalanceDiff = 0.06f;

  settings.IBattOptiChargeMin    = 05.0f;
  settings.IBattOptiChargeMax    = 50.0f;
  settings.IBattWarnChargeMax    = 90.0f;
  
  settings.IBattOptiDischargeMax = 90.0f;
  settings.IBattWarnDischargeMax = 90.0f;
  
  settings.TBattNormMin = 15.0f;
  settings.TBattOptiMax = 24.0f;
  settings.TBattNormMax = 28.0f;
  settings.TBattWarnMax = 30.0f;

  settings.QBattNormMin = getQCellSpec(settings.UCellNormMin) * (float)settings.ConfigBattParallelCells * (float)settings.ConfigBattParallelStrings;
  settings.QBattNormMax = getQCellSpec(settings.UCellNormMax) * (float)settings.ConfigBattParallelCells * (float)settings.ConfigBattParallelStrings;
  settings.QBattNorm    = settings.QBattNormMax - settings.QBattNormMin;
  settings.QBattNormKwh = getQBattNorm(settings.UCellNormMax);

  settings.ConstInCurrentOffset = 0.0f; // will be set on first measurement
  settings.ConstInCurrentSensitivity = -375.939f; // ACS758xCB 150A bidirectional, 50% = 0A, 13.3mV/A @ 5V = 13.3/5000 = 1/375.939
  settings.ConstInCurrentSampleFreq = 256;
  
  settings.logLevel = Logger::Info;
}

void setup_bus()
{
  SERIALCONSOLE.begin(115200); // USB serial
  SERIALBMS.begin(612500);     // Tesla serial bus
  
  CANVE.begin(); // VE.Can to CCGX
  CANVE.setBaudRate(500000);
  CANVE.setTx(CAN_PIN);
  CANVE.setRx(CAN_PIN);
  CANVE.enableFIFO();
  CANVE.enableFIFOInterrupt();
  CANVE.onReceive(loop_readcan);

  Logger::setLoglevel((Logger::LogLevel)settings.logLevel);
  Logger::console("Initializing ...");
}

void setup_cooler()
{
  pinMode(COOLER, OUTPUT);
  digitalWrite(COOLER, LOW);
}

void setup_bms()
{
  // pinMode(PRELOAD, OUTPUT);
  // digitalWrite(PRELOAD, LOW);
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
  
  adc.adc0->setResolution(14);
  adc.adc0->setConversionSpeed(ADC_CONVERSION_SPEED::LOW_SPEED);
  adc.adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::LOW_SPEED);
  adc.adc0->analogRead(INCURRENT); // performs various ADC setup stuff

  dma.source(ADC0_RA); // ADC result register
  dma.transferSize(2);
  dma.triggerAtHardwareEvent(DMAMUX_SOURCE_ADC0);
  dma.destinationBuffer(buffer.data(), buffer.size() * sizeof(buffer[0]));
  dma.enable();

  adc.adc0->enableDMA();
  
  adc.adc0->stopPDB();
  adc.adc0->startPDB(settings.ConstInCurrentSampleFreq); 
  NVIC_DISABLE_IRQ(IRQ_PDB); // we don't want or need the PDB interrupt
}

void setup_led()
{
  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH);
}

void pdb_isr(void) {
  PDB0_SC &=~PDB_SC_PDBIF; // clear interrupt, called once, bad combination with ADC-library
}

// loop ###############
void loop_led()
{
  digitalWrite(LED, ! digitalRead(LED));
}

void loop_querycurrent()
{
  //if(adc.adc0->fail_flag) {
  //  Logger::error("ADC FAIL %l", adc.adc0->fail_flag);
  //}

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
  
  status.IBattCurr = (value / maxvalue) * (double)settings.ConstInCurrentSensitivity + (double)settings.ConstInCurrentOffset;
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

  status.QCycleMeasuredKwh += (double)status.IBattCurr / (double)3600.0 / (double)1000.0 * (double)status.UCellCurrAvg * (double)settings.ConfigBattSerialCells;
}

void loop_querybatt()
{
  bms.getAllVoltTemp();
  status.UBattCurr = (double)bms.getPackVoltage() / (double)settings.ConfigBattParallelStrings;
  
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
      if (bms.balanceCells(settings.UCellNormBalanceDiff)) {
        Logger::info("Balancing Pack");
        status.Alarm |= ERROR_CELLIMBALANCE; // not a real alarm, but wanted to be notified in CCGX
      }

    status.IBattPlanChargeMax = settings.IBattOptiChargeMax * (settings.UCellNormMax - status.UCellCurrMax) / (settings.UCellNormMax - settings.UCellOptiMax);   
    if (status.IBattPlanChargeMax < settings.IBattOptiChargeMin) status.IBattPlanChargeMax = settings.IBattOptiChargeMin; // minimum charge
  }
  else if (status.UCellCurrMax < settings.UCellWarnMax)
  {
    if (status.IBattCurrCharge > 0.1 * settings.IBattOptiChargeMax)
      if (bms.balanceCells(settings.UCellNormBalanceDiff)) {
        Logger::info("Balancing Pack");
        status.Alarm |= ERROR_CELLIMBALANCE; // not a real alarm, but wanted to be notified in CCGX
      }

    status.IBattPlanChargeMax = 0;
    // status.Alarm |= ERROR_HIGHCELLVOLTAGE; // do not fire alarm, happens often
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
    // status.Alarm |= ERROR_LOWCELLVOLTAGE; // do not fire alarm, happens often
  }
  else //if (status.UCellCurrMin > settings.UCellErrorMin) or worse
  {
    status.IBattPlanDischargeMax = 0;
    status.Error |= ERROR_LOWCELLVOLTAGE;
  }

  // check errors could implement warnings
  if (status.TBattCurrMax > settings.TBattWarnMax)
  {
    status.Error |= ERROR_HIGHPACKTEMP;
    Logger::error("Error - high battery temp %f over %f", status.TBattCurrMax, settings.TBattNormMax);
  }
  else if (status.TBattCurrMax > settings.TBattNormMax)
  {
    status.IBattPlanDischargeMax *= 0.2;
    status.IBattPlanChargeMax *= 0.2;
    Logger::info("Overtemp, limiting Current", status.TBattCurrMax, settings.TBattNormMax);
  }
    
  if (status.TBattCurrMin < settings.TBattNormMin)
  {
    status.IBattPlanDischargeMax *= 0.2;
    status.IBattPlanChargeMax *= 0.2;
    Logger::info("Undertemp, limiting Current", status.TBattCurrMin, settings.TBattNormMin);
  }

  if (status.IBattCurrDischarge > settings.IBattWarnDischargeMax)
  {
    status.Error |= ERROR_HIGHBATTDISCHARGECURRENT;
    Logger::error("Error - high discharge current %f over %f", status.IBattCurrDischarge, settings.IBattWarnDischargeMax);
  }

  if (status.IBattCurrCharge > settings.IBattWarnChargeMax)
  {
    status.Error |= ERROR_HIGHBATTCHARGECURRENT;
    Logger::error("Error - high charge current %f over %f", status.IBattCurrCharge, settings.IBattWarnChargeMax);
  }

  if (bms.isFaulted)
  {
    status.Error |= ERROR_INTERNALFAILURE;
    Logger::error("Error - internal failure");
  }

  if (status.UCellCurrDelta > settings.UCellWarnBalanceDiff)
  {
    status.Error |= ERROR_CELLIMBALANCE;
    Logger::error("Error - cell imbalance");
  }

  if (bms.getCommunicationErrors() > status.CBmsWarnErrors)
  {
    status.Error |= ERROR_INTERNALFAILURE;
    Logger::error("Error - bms communication errors");    
  }

  // shut off charging/discharging on Warning or Error
  if (status.Error != 0 || status.Alarm != 0)
  {
    status.IBattPlanChargeMax = 0;
    status.IBattPlanDischargeMax = 0;
  }
}

void loop_cooler()
{
  // turn on cooler if hotter than opt
  digitalWrite(COOLER, status.TBattCurrMax > settings.TBattOptiMax);
}

void loop_contactor()
{
  
  if (status.State > 0 && status.Error != 0) 
  {
    // digitalWrite(PRELOAD, LOW);
    digitalWrite(CONTACTOR, LOW);  
    status.State = 0;
    Logger::error("Contactor - disconnect on error");  
  } 
  else if (status.State == 0 && status.Error == 0 && status.Alarm == 0) //  start preloading soon if normal state
  {
    status.State = 1;
    Logger::info("Contactor - preload starts soon");  
  }
  else if (status.State == 1 && status.Error == 0 && status.Alarm == 0) //  start preloading if normal state
  {
    // digitalWrite(PRELOAD, HIGH);
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
    // digitalWrite(PRELOAD, LOW);
    status.State = 4;
    Logger::info("Contactor - closed, preload released");  
  }
  else if ((status.State == 4 || status.State == 5) && status.IBattCurr > 0.0f)  // charging -> discharging
  {    
    status.State = 6;
    status.QCycleNormStartKwh = status.QBattCurrKwh;
    status.QCycleMeasuredKwh = 0.0f;
    Logger::info("Contactor - closed, started discharging");  
  }
  else if ((status.State == 4 || status.State == 6) && status.IBattCurr < 0.0f)  // discharging -> charging
  {    
    status.State = 5;
    status.QCycleNormStartKwh = status.QBattCurrKwh;
    status.QCycleMeasuredKwh = 0.0f;
    Logger::info("Contactor - closed, started charging");  
  }

}

void loop_calc()
{
  // QBattNorm           = 100% Capacity within Norm-range [Ah]
  // QBattCurr           = current Capacity [Ah]
  // OCV Method: Mapping UCellCurrAvg to discharge curve: 0%-100% = Norm-Range not Spec-Range
  status.QBattCurr = getQCellSpec(status.UCellCurrAvg) * (float)settings.ConfigBattParallelCells * (float)settings.ConfigBattParallelStrings - settings.QBattNormMin;  
  status.QBattCurrKwh = getQBattNorm(status.UCellCurrAvg);
  
  status.SocBattCurr = status.QBattCurrKwh / settings.QBattNormKwh;
  if(status.SocBattCurr < 0.0f) status.SocBattCurr = 0.0f;
  if(status.SocBattCurr > 1.0f) status.SocBattCurr = 1.0f;
  
  // SohBattCurr = QBattCurr / QBattNorm (Discharge) or QBattNorm / QBattCurr (Charge)
  status.QCycleNormKwh = status.QCycleNormStartKwh - status.QBattCurrKwh; 
  if(status.QCycleMeasuredKwh > +1.0f && status.QCycleNormKwh > +1.0f) 
    status.SohBattCurr = (double)status.QCycleMeasuredKwh / (double)status.QCycleNormKwh;  // discharge
  if(status.QCycleMeasuredKwh < -1.0f && status.QCycleNormKwh < -1.0f) 
    status.SohBattCurr = (double)status.QCycleMeasuredKwh / (double)status.QCycleNormKwh;  // charge

  if(status.SohBattCurr > 1.0f) status.SohBattCurr = 1.0f;
}

void loop_console()
{
  Logger::info("Qm=%f/%fkWh Q=%fAh/%fAh Q=%fkWh/%fkWh SOC=%f SOH=%f I=%fA UBatt=%fV UCell=%fV UCellDelta=%fV T=%fC", 
                        status.QCycleMeasuredKwh,
                        status.QCycleNormKwh,
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

  msg.flags.extended = 0;
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

  msg.flags.extended = 0;
  msg.id = 0x355;
  msg.len = 4;
  msg.buf[0] = (byte)(status.SocBattCurr * 100.0f);
  msg.buf[1] = 0;
  msg.buf[2] = (byte)(status.SohBattCurr * 100.0f);
  msg.buf[3] = 0;
  Logger::debug("VECan %i %i", msg.id, msg.buf[0]);
  CANVE.write(msg);

  msg.flags.extended = 0;
  msg.id = 0x356;
  msg.len = 6;
  msg.buf[0] = lowByte(int16_t(status.UBattCurr * 100.0f));
  msg.buf[1] = highByte(int16_t(status.UBattCurr * 100.0f));
  msg.buf[2] = lowByte(int16_t(status.IBattCurr * -10.0f));
  msg.buf[3] = highByte(int16_t(status.IBattCurr * -10.0f));
  msg.buf[4] = lowByte(int16_t(status.TBattCurrMax * 10.0f));
  msg.buf[5] = highByte(int16_t(status.TBattCurrMax * 10.0f));
  Logger::debug("VECan %i %i", msg.id, msg.buf[0]);
  CANVE.write(msg);

  msg.flags.extended = 0;
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

  // ccgx bms detail menu
  msg.flags.extended = 0;
  msg.id  = 0x373;
  msg.len = 8;
  msg.buf[0] = lowByte(int16_t(bms.getLowCellVolt() * 1000));
  msg.buf[1] = highByte(int16_t(bms.getLowCellVolt() * 1000));
  msg.buf[2] = lowByte(int16_t(bms.getHighCellVolt() * 1000));
  msg.buf[3] = highByte(int16_t(bms.getHighCellVolt() * 1000));
  msg.buf[4] = lowByte(uint16_t(bms.getLowTemperature() + 273.15));
  msg.buf[5] = highByte(uint16_t(bms.getLowTemperature() + 273.15));
  msg.buf[6] = lowByte(uint16_t(bms.getHighTemperature() + 273.15));
  msg.buf[7] = highByte(uint16_t(bms.getHighTemperature() + 273.15));
  Logger::debug("VECan %i %i", msg.id, msg.buf[0]);
  CANVE.write(msg);

  // Installed capacity
  msg.flags.extended = 0;
  msg.id  = 0x379; 
  msg.len = 2;
  msg.buf[0] = lowByte(uint16_t(settings.QBattNorm));
  msg.buf[1] = highByte(uint16_t(settings.QBattNorm));
  Logger::debug("VECan %i %i", msg.id, msg.buf[0]);
  CANVE.write(msg);

  // bmsname
  msg.flags.extended = 0;
  msg.id = 0x35E;
  msg.len = 8;
  for (size_t i = 0; i < sizeof(bmsname); i++)
  {
    msg.buf[i] = bmsname[i];
  }
  Logger::debug("VECan %i %i", msg.id, msg.buf[0]);
  CANVE.write(msg);

  // bmsmanufacturer
  msg.flags.extended = 0;
  msg.id = 0x370;
  msg.len = 8;
  for (size_t i = 0; i < sizeof(bmsmanu); i++)
  {
    msg.buf[i] = bmsmanu[i];
  }
  Logger::debug("VECan %i %i", msg.id, msg.buf[0]);
  CANVE.write(msg);

  // bms firmware version and battery current
  msg.flags.extended = 0;
  msg.id = 0x35F;
  msg.len = 6;
  msg.buf[0] = 0x01;
  msg.buf[1] = 0x00;
  msg.buf[2] = lowByte(bsmFWV[0]);
  msg.buf[3] = lowByte(bsmFWV[1]);
  msg.buf[4] = lowByte(uint16_t(status.QBattCurr));
  msg.buf[5] = highByte(uint16_t(status.QBattCurr));
  Logger::debug("VECan %i %i", msg.id, msg.buf[0]);
  CANVE.write(msg);

  msg.id  = 0x372; // modules info
  msg.len = 8;
  msg.buf[0] = lowByte(uint16_t(bms.numFoundModules));                                      // System/NrOfModulesOnline
  msg.buf[1] = highByte(uint16_t(bms.numFoundModules));
  msg.buf[2] = lowByte(uint16_t(0));                                                        // System/NrOfModulesBlockingCharge
  msg.buf[3] = highByte(uint16_t(0));
  msg.buf[4] = lowByte(uint16_t(0));                                                        // System/NrOfModulesBlockingDischarge
  msg.buf[5] = highByte(uint16_t(0));
  msg.buf[6] = lowByte(uint16_t(settings.ConfigBattParallelStrings - bms.numFoundModules)); // System/NrOfModulesOffline
  msg.buf[7] = highByte(uint16_t(settings.ConfigBattParallelStrings - bms.numFoundModules));
  Logger::debug("VECan %i %i", msg.id, msg.buf[0]);
  CANVE.write(msg);
}

void loop_readcan(const CAN_message_t &msg)
{
  char buf[128], *pos = buf;
  for (int i = 0 ; i < msg.len ; i++) {
    if (i) {
      pos += sprintf(pos, ", ");
    }
    pos += sprintf(pos, "0x%.2X", msg.buf[i]);
  }

  Logger::debug("VECan: MB %d OVERRUN: %d BUS %d LEN: %d EXT: %d REMOTE: %d TS: %d ID: %X IDHIT: %d Buffer: %s", 
    msg.mb, msg.flags.overrun, msg.bus, msg.len, msg.flags.extended, msg.flags.remote, msg.timestamp, msg.id, msg.idhit, buf);
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
    return out * 0.001l * (double)settings.ConfigBattSerialCells * (double)settings.ConfigBattParallelCells * (double)settings.ConfigBattParallelStrings;
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

static inline int8_t sgn(float val) {
  if (val < 0) return -1;
  if (val==0) return 0;
  return 1;
}
