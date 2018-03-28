#include "BMSModuleManager.h"
#include <Arduino.h>
#include "config.h"
#include "SerialConsole.h"
#include "Logger.h"
#include <ADC.h>
#include <FlexCAN.h>

BMSModuleManager bms;
SerialConsole console;
EEPROMSettings settings;

byte bmsstatus = Boot;

int Discharge;

//variables for output control
int pulltime = 1000;
int contctrl, contstat = 0; //1 = out 5 high 2 = out 6 high 3 = both high
unsigned long conttimer, Pretimer = 0;
int Pretime = 5000; //precharge timer
int conthold = 50; //holding duty cycle for contactor 0-255
int Precurrent = 1000; //ma before closing main contator

//variables for VE can
uint16_t chargevoltage = 48000; //max charge voltage in mv 48V
uint16_t chargecurrent = 2000; //max charge current in ma 20A
uint16_t disvoltage = 40000; // max discharge voltage in mv 40V
uint16_t discurrent = 2000; // max discharge current in ma 20A
uint16_t SOH = 100; // SOH place holder

static CAN_message_t msg;
static CAN_message_t msgin;
long unsigned int rxId;

char msgString[128];                        // Array to store serial string
uint32_t inbox;
signed long CANmilliamps;

//variables for current calulation
int value;
int invertcur = 0;
uint16_t offset1 = 1735;
uint16_t offset2 = 1733;
int highconv = 285;
float currentact, RawCur;
float ampsecond;
unsigned long lasttime;
unsigned long looptime;
int currentsense = 14;
int sensor = 1;

//running average
const int RunningAverageCount = 16;
float RunningAverageBuffer[RunningAverageCount];
int NextRunningAverage;

//Variables for SOC calc
int SOC = 100; //State of Charge
int SOCset = 0;
uint16_t socvolt[4] = {3100, 10, 4100, 90};
int CAP = 100; //battery size in Ah

//variables
int incomingByte = 0;
int x = 0;
int debug = 1;
int candebug = 0;
int debugCur = 0;
int menuload = 0;


ADC *adc = new ADC(); // adc object

void loadSettings()
{
  Logger::console("Resetting to factory defaults");
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
}

void setup()
{
  pinMode(INACUR1, INPUT);
  pinMode(INACUR2, INPUT);
  pinMode(INKEY, INPUT);
  pinMode(INACPRESENT, INPUT);
  pinMode(INBMBFAULT, INPUT);
  pinMode(OUTCONTACTOR, OUTPUT); // drive contactor
  pinMode(OUTPRECHARGE, OUTPUT); // precharge
  pinMode(OUTCHARGERELEAY, OUTPUT); // charge relay
  pinMode(OUTCONTACTOR, OUTPUT); // Negative contactor
  pinMode(OUTPWMDRIVER, OUTPUT); // pwm driver output
  pinMode(OUTPWMDRIVER2, OUTPUT); // pwm driver output

  loadSettings();

  delay(4000);  //just for easy debugging. It takes a few seconds for USB to come up properly on most OS's

  SERIALCONSOLE.begin(115200); 
  SERIALBMS.begin(612500); //Tesla serial bus
  Can0.begin(500000);

  Logger::setLoglevel((Logger::LogLevel)settings.logLevel); //Debug = 0, Info = 1, Warn = 2, Error = 3, Off = 4
  Logger::console("Initializing ...");
    
  adc->setAveraging(16); // set number of averages
  adc->setResolution(16); // set bits of resolution
  adc->setConversionSpeed(ADC_CONVERSION_SPEED::MED_SPEED);
  adc->setSamplingSpeed(ADC_SAMPLING_SPEED::MED_SPEED);
  adc->startContinuous(INACUR1, ADC_0);

  
  bms.renumberBoardIDs();

  bms.clearFaults();
  
  bms.findBoards();
}

void loop()
{
  if (Can0.available())                        // If rx flag is set
  {
    canread();
  }
  if (SERIALCONSOLE.available() > 0)
  {
    menu();
  }

  contcon();

  switch (bmsstatus)
  {
    case (Boot):
      Discharge = 0;


      bmsstatus = Ready;
      break;

    case (Ready):
      Discharge = 0;
      if (bms.getHighCellVolt() > settings.balanceVoltage);
      {
        bms.balanceCells();
      }
      if (digitalRead(INACPRESENT) == HIGH && (settings.balanceVoltage + settings.balanceHyst) > bms.getHighCellVolt()) //detect AC present for charging and check not balancing
      {
        bmsstatus = Charge;
      }
      if (digitalRead(INKEY) == HIGH) //detect Key ON
      {
        bmsstatus = Precharge;
        Pretimer = millis();
      }

      break;

    case (Precharge):
      Discharge = 0;
      Prechargecon();
      break;


    case (Drive):
      Discharge = 1;
      if (digitalRead(INKEY) == LOW)//Key OFF
      {
        digitalWrite(OUTCONTACTOR, LOW);
        digitalWrite(OUTCONTACTOR, LOW);

        contctrl = 0; //turn off out 5 and 6
        bmsstatus = Ready;
      }

      break;

    case (Charge):
      Discharge = 0;
      digitalWrite(OUTCHARGERELEAY, HIGH);//enable charger
      if (bms.getHighCellVolt() > settings.balanceVoltage);
      {
        bms.balanceCells();
      }
      if (bms.getHighCellVolt() > settings.OverVSetpoint);
      {
        digitalWrite(OUTCHARGERELEAY, LOW);//turn off charger
        bmsstatus = Ready;
      }
      if (digitalRead(INACPRESENT) == LOW)//detect AC not present for charging
      {
        digitalWrite(OUTCHARGERELEAY, LOW);//turn off charger
        bmsstatus = Ready;
      }
      break;

    case (Err):
      Discharge = 0;

      if (digitalRead(INACPRESENT) == HIGH) //detect AC present for charging
      {
        bmsstatus = Charge;
      }
      if (bms.getLowCellVolt() >= settings.UnderVSetpoint);
      {
        bmsstatus = Ready;
      }

      break;
  }
  if (cursens == Analogue)
  {
    getcurrent();
  }
  if (millis() - looptime > 500)
  {

    looptime = millis();
    bms.getAllVoltTemp();

    if (bms.getLowCellVolt() < settings.UnderVSetpoint)
    {
      bmsstatus = Err;
    }

    if (debug != 0)
    {
      printbmsstat();
      bms.printPackDetails();
    }
    updateSOC();
    VEcan();
  }
}

void printbmsstat()
{
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
  if (digitalRead(INKEY) == HIGH)
  {
    SERIALCONSOLE.print("| AC Present |");
  }
  if (digitalRead(INKEY) == HIGH)
  {
    SERIALCONSOLE.print("| Key ON |");
  }
}


void getcurrent()
{
  if (cursens == Analogue)
  {
    if (currentact < 19000 && currentact > -19000)
    {
      sensor = 1;
      adc->startContinuous(INACUR1, ADC_0);
    }
    else
    {
      sensor = 2;
      adc->startContinuous(INACUR2, ADC_0);
    }

    if (sensor == 1)
    {
      if (debugCur != 0)
      {
        SERIALCONSOLE.print("Low Range: ");
        SERIALCONSOLE.print("Value ADC0: ");
      }
      value = (uint16_t)adc->analogReadContinuous(ADC_0); // the unsigned is necessary for 16 bits, otherwise values larger than 3.3/2 V are negative!
      if (debugCur != 0)
      {
        SERIALCONSOLE.print(value * 3.3 / adc->getMaxValue(ADC_0), 5);
        SERIALCONSOLE.print("  ");
      }
      RawCur = (float(value * 3300 / adc->getMaxValue(ADC_0)) - offset1) * 15.7;
      if (value < 100 || value > (adc->getMaxValue(ADC_0) - 100))
      {
        RawCur = 0;
      }
      if (debugCur != 0)
      {
        SERIALCONSOLE.print("  ");
        SERIALCONSOLE.print(RawCur);
        SERIALCONSOLE.print("mA");
        SERIALCONSOLE.print("  ");
      }
    }
    else
    {
      if (debugCur != 0)
      {
        SERIALCONSOLE.print("High Range: ");
        SERIALCONSOLE.print("Value ADC0: ");
      }
      value = (uint16_t)adc->analogReadContinuous(ADC_0); // the unsigned is necessary for 16 bits, otherwise values larger than 3.3/2 V are negative!
      if (debugCur != 0)
      {
        SERIALCONSOLE.print(value * 3.3 / adc->getMaxValue(ADC_0), 5);
        SERIALCONSOLE.print("  ");
      }
      RawCur = (float(value * 3300 / adc->getMaxValue(ADC_0)) - offset2) * highconv;
      if (value < 100 || value > (adc->getMaxValue(ADC_0) - 100))
      {
        RawCur = 0;
      }
      if (debugCur != 0)
      {
        SERIALCONSOLE.print("  ");
        SERIALCONSOLE.print(RawCur);
        SERIALCONSOLE.print("mA");
        SERIALCONSOLE.print("  ");
      }
    }
  }
  if (invertcur == 1)
  {
    RawCur = RawCur * -1;
  }
  RunningAverageBuffer[NextRunningAverage++] = RawCur;
  if (NextRunningAverage >= RunningAverageCount)
  {
    NextRunningAverage = 0;
  }
  float RunningAverageCur = 0;
  for (int i = 0; i < RunningAverageCount; ++i)
  {
    RunningAverageCur += RunningAverageBuffer[i];
  }
  RunningAverageCur /= RunningAverageCount;

  currentact = RunningAverageCur;

  if (cursens == Analogue)
  {
    if (sensor == 1)
    {
      if (currentact > 500 || currentact < -500 )
      {
        ampsecond = ampsecond + ((currentact * (millis() - lasttime) / 1000) / 1000);
        lasttime = millis();
      }
      else
      {
        lasttime = millis();
      }
    }
    if (sensor == 2)
    {
      if (currentact > 180000 || currentact < -18000 )
      {
        ampsecond = ampsecond + ((currentact * (millis() - lasttime) / 1000) / 1000);
        lasttime = millis();
      }
      else
      {
        lasttime = millis();
      }
    }
  }
  else
  {
    if (currentact > 500 || currentact < -500 )
    {
      ampsecond = ampsecond + ((currentact * (millis() - lasttime) / 1000) / 1000);
      lasttime = millis();
    }
    else
    {
      lasttime = millis();
    }
  }
}

void updateSOC()
{
  if (SOCset == 0)
  {
    SOC = map(uint16_t(bms.getAvgCellVolt() * 1000), socvolt[0], socvolt[2], socvolt[1], socvolt[3]);
    SERIALCONSOLE.print("  ");
    SERIALCONSOLE.print(SOC);
    SERIALCONSOLE.print("  ");
    ampsecond = (SOC * CAP * 10) / 0.27777777777778 ;
    SOCset = 1;
  }
  SOC = ((ampsecond * 0.27777777777778) / (CAP * 1000)) * 100;
  if (bms.getAvgCellVolt() > settings.OverVSetpoint)
  {
    ampsecond = (CAP * 1000) / 0.27777777777778 ; //reset to full, dependant on given capacity. Need to improve with auto correction for capcity.
    if (SOC >= 100)
    {
      SOC = 100;
    }
  }


  if (SOC < 0)
  {
    //
  }

  if (debug != 0)
  {
      if (cursens == Analogue)
  {
    if (sensor == 1)
    {
      SERIALCONSOLE.print("Low Range ");
    }
    else
    {
      SERIALCONSOLE.print("High Range");
    }
  }
  else
  {
    SERIALCONSOLE.print("CANbus ");
  }
    SERIALCONSOLE.print("  ");
    SERIALCONSOLE.print(currentact);
    SERIALCONSOLE.print("mA");
    SERIALCONSOLE.print("  ");
    SERIALCONSOLE.print(SOC);
    SERIALCONSOLE.print("% SOC ");
    SERIALCONSOLE.print(ampsecond * 0.27777777777778, 2);
    SERIALCONSOLE.println ("mAh");

  }
}

void Prechargecon()
{
  if (digitalRead(INKEY) == HIGH) //detect Key ON
  {
    digitalWrite(OUTCONTACTOR, HIGH);//Negative Contactor Close
    contctrl = 2;
    if (Pretimer + Pretime > millis() || currentact > Precurrent)
    {
      digitalWrite(OUTPRECHARGE, HIGH);//precharge
    }
    else //close main contactor
    {
      digitalWrite(OUTCONTACTOR, HIGH);//Positive Contactor Close
      contctrl = 3;
      bmsstatus = Drive;
      digitalWrite(OUTPRECHARGE, LOW);
    }
  }
  else
  {
    bmsstatus = Ready;
    contctrl = 0;
  }
}

void contcon()
{
  if (contctrl != contstat) //check for contactor request change
  {
    if ((contctrl & 1) == 0)
    {
      analogWrite(OUTPWMDRIVER, 0);
      contstat = contstat & 254;
    }
    if ((contctrl & 2) == 0)
    {
      analogWrite(OUTPWMDRIVER2, 0);
      contstat = contstat & 253;
    }

    if ((contctrl & 1) == 1)
    {
      if (conttimer == 0)
      {
        analogWrite(OUTPWMDRIVER, 255);
        conttimer = millis() + pulltime ;
      }
      if (conttimer < millis())
      {
        analogWrite(OUTPWMDRIVER, conthold);
        contstat = contstat | 1;
        conttimer = 0;
      }
    }

    if ((contctrl & 2) == 2)
    {
      if (conttimer == 0)
      {
        analogWrite(OUTPWMDRIVER2, 255);
        conttimer = millis() + pulltime ;
      }
      if (conttimer < millis())
      {
        analogWrite(OUTPWMDRIVER2, conthold);
        contstat = contstat | 2;
        conttimer = 0;
      }
    }
    /*
       SERIALCONSOLE.print(conttimer);
       SERIALCONSOLE.print("  ");
       SERIALCONSOLE.print(contctrl);
       SERIALCONSOLE.print("  ");
       SERIALCONSOLE.print(contstat);
       SERIALCONSOLE.println("  ");
    */

  }
}

void calcur()
{
  adc->startContinuous(INACUR1, ADC_0);
  sensor = 1;
  SERIALCONSOLE.print(" Calibrating Current Offset ::::: ");
  while (x < 20)
  {
    offset1 = offset1 + ((uint16_t)adc->analogReadContinuous(ADC_0) * 3300 / adc->getMaxValue(ADC_0));
    SERIALCONSOLE.print(".");
    delay(100);
    x++;
  }
  offset1 = offset1 / 21;
  SERIALCONSOLE.print(offset1);
  SERIALCONSOLE.print(" current offset 1 calibrated ");
  SERIALCONSOLE.println("  ");
  x = 0;
  adc->startContinuous(INACUR2, ADC_0);
  sensor = 2;
  SERIALCONSOLE.print(" Calibrating Current Offset ::::: ");
  while (x < 20)
  {
    offset2 = offset2 + ((uint16_t)adc->analogReadContinuous(ADC_0) * 3300 / adc->getMaxValue(ADC_0));
    SERIALCONSOLE.print(".");
    delay(100);
    x++;
  }
  offset2 = offset2 / 21;
  SERIALCONSOLE.print(offset2);
  SERIALCONSOLE.print(" current offset 2 calibrated ");
  SERIALCONSOLE.println("  ");
}


void VEcan() //communication with Victron system over CAN
{
  msg.ext = 0;
  msg.id = 0x351;
  msg.len = 8;
  msg.buf[0] = lowByte(chargevoltage / 100);
  msg.buf[1] = highByte(chargevoltage / 100);
  msg.buf[2] = lowByte(chargecurrent / 100);
  msg.buf[3] = highByte(chargecurrent / 100);
  msg.buf[4] = lowByte(discurrent / 100);
  msg.buf[5] = highByte(discurrent / 100);
  msg.buf[6] = lowByte(disvoltage / 100);
  msg.buf[7] = highByte(disvoltage / 100);

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

// Settings menu
void menu()
{


  incomingByte = Serial.read(); // read the incoming byte:

  if (menuload == 2)
  {
    switch (incomingByte)
    {


      case 99: //c for calibrate zero offset

        calcur();
        break;



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
        loadSettings();
        SERIALCONSOLE.println("  ");
        SERIALCONSOLE.println("  ");
        break;

      case 114: //r for reset
        ampsecond = 0;
        SERIALCONSOLE.println("  ");
        SERIALCONSOLE.print(" mAh Zeroed ");
        SERIALCONSOLE.println("  ");
        break;

      case 100: //d dispaly settings
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
        SERIALCONSOLE.print(CAP);
        SERIALCONSOLE.print("Ah Battery Capacity - 7 ");
        SERIALCONSOLE.println("  ");
        break;
      case 101: //e dispaly settings
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

      case 55://7 Battery Capacity inAh
        if (Serial.available() > 0)
        {
          CAP = Serial.parseInt();
          SERIALCONSOLE.print(CAP);
          SERIALCONSOLE.print("Ah Battery Capacity");
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

void canread()
{
  Can0.read(msgin);      // Read data: len = data length, buf = data byte(s)

  switch (msgin.id)
  {
    case 0x3c2:
      CAB300(msgin.buf);
      break;

    default:
      break;
  }
  if (candebug == 1)
  {
    Serial.print(millis());
    if ((rxId & 0x80000000) == 0x80000000)    // Determine if ID is standard (11 bits) or extended (29 bits)
      sprintf(msgString, "Extended ID: 0x%.8lX  DLC: %1d  Data:", (rxId & 0x1FFFFFFF), msgin.len);
    else
      sprintf(msgString, ",0x%.3lX,false,%1d", rxId, msgin.len);

    Serial.print(msgString);

    if ((rxId & 0x40000000) == 0x40000000) {  // Determine if message is a remote request frame.
      sprintf(msgString, " REMOTE REQUEST FRAME");
      Serial.print(msgString);
    } else {
      for (byte i = 0; i < msgin.len; i++) {
        sprintf(msgString, ", 0x%.2X", msgin.buf[i]);
        Serial.print(msgString);
      }
    }

    Serial.println();
  }
}

void CAB300(byte data[8])
{
  for (int i = 0; i < 4; i++)
  {
    inbox = (inbox << 8) | data[i];
  }
  CANmilliamps = inbox;
  if (CANmilliamps > 0x80000000)
  {
    CANmilliamps -= 0x80000000;
  }
  else
  {
    CANmilliamps = (0x80000000 - CANmilliamps) * -1;
  }
  if (cursens == Canbus)
  {
    RawCur = CANmilliamps;
    getcurrent();
  }
  if (candebug == 1)
  {
    Serial.println();
    Serial.print(CANmilliamps);
    Serial.print("mA");
  }
}
