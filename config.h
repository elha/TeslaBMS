#pragma once

#include <Arduino.h>

// bms status values
#define Boot 0
#define Ready 1
#define Drive 2
#define Charge 3
#define Err 4

// current sensor values
#define Undefined 0
#define Analogue 1
#define Canbus 2

// USB
#define SERIALCONSOLE Serial

// Tesla BMS Modules
#define SERIALBMS Serial3 // TX 8, RX 7
#define INBMBFAULT 11

// Victron Bus
#define CANVE Can0        // TX 3, RX 4

// Status LED
#define LED  13       

// Currentsensor
#define INCURRENT A1

#define PRELOAD   38
#define CONTACTOR 39

#define REG_DEV_STATUS      1
#define REG_GPAI            1
#define REG_VCELL1          3
#define REG_VCELL2          5
#define REG_VCELL3          7
#define REG_VCELL4          9
#define REG_VCELL5          0xB
#define REG_VCELL6          0xD
#define REG_TEMPERATURE1    0xF
#define REG_TEMPERATURE2    0x11
#define REG_ALERT_STATUS    0x20
#define REG_FAULT_STATUS    0x21
#define REG_COV_FAULT       0x22
#define REG_CUV_FAULT       0x23
#define REG_ADC_CTRL        0x30
#define REG_IO_CTRL         0x31
#define REG_BAL_CTRL        0x32
#define REG_BAL_TIME        0x33
#define REG_ADC_CONV        0x34
#define REG_ADDR_CTRL       0x3B

#define MAX_MODULE_ADDR     0x3E

#define EEPROM_VERSION      0x10    //update any time EEPROM struct below is changed.
#define EEPROM_PAGE         0

#define ERROR_NONE                      0x0
#define ERROR_HIGHCELLVOLTAGE           0x00000004
#define ERROR_LOWCELLVOLTAGE            0x00000010
#define ERROR_HIGHPACKTEMP              0x00000040
#define ERROR_LOWPACKTEMP               0x00000100
#define ERROR_HIGHBATTDISCHARGECURRENT  0x00004000
#define ERROR_HIGHBATTCHARGECURRENT     0x00010000
#define ERROR_INTERNALFAILURE           0x00400000
#define ERROR_CELLIMBALANCE             0x01000000

static const uint8_t SizeCellSpecCurve0_2C = 30;
//         V,        mAh
static const double QCellSpecCurve0_2C[SizeCellSpecCurve0_2C][2] = { 
        {  2.50,      0.0  },
        {  2.60,     15.0  },
        {  2.70,     30.0  },
        {  2.80,     52.0  },
        {  2.85,     64.0  },
        {  2.90,     80.0  },
        {  2.95,    102.0  },
        {  3.00,    121.0  },
        {  3.05,    147.0  },
        {  3.10,    180.0  },
        {  3.15,    221.0  },
        {  3.20,    278.0  },
        {  3.25,    360.0  }, // Norm min 39.00
        {  3.30,    459.0  },
        {  3.35,    577.0  },
        {  3.40,    736.0  },
        {  3.45,    928.0  },
        {  3.50,   1184.0  },
        {  3.55,   1452.0  },
        {  3.60,   1708.0  },
        {  3.70,   2105.0  },
        {  3.80,   2471.0  },
        {  3.90,   2804.0  },
        {  4.00,   3087.0  },
        {  4.02,   3143.0  },
        {  4.06,   3283.0  }, // Norm max 48.72
        {  4.08,   3331.0  },
        {  4.10,   3361.0  },
        {  4.12,   3378.0  },
        {  4.20,   3393.0  }};

typedef struct {
    uint8_t version;
    uint8_t logLevel;
    
    uint8_t ConfigBattParallelCells;
    uint8_t ConfigBattSerialCells;
    uint8_t ConfigBattParallelStrings;
    
    double UBattNormMin;
    double UBattNormMax;

    double UCellWarnMin;
    double UCellNormMin;
    double UCellOptiMin;

    double UCellOptiMax;
    double UCellNormMax;
    double UCellWarnMax;

    double UCellNormBalanceDiff;
    double UCellWarnBalanceDiff;

    double IBattWarnChargeMax;
    double IBattWarnDischargeMax;

    double IBattOptiChargeMax;
    double IBattOptiChargeMin;
    double IBattOptiDischargeMax;

    double TBattNormMin;
    double TBattNormMax;

    double QBattNormMin;
    double QBattNormMax;    
    double QBattNorm;
    double QBattNormKwh;

    double ConstInCurrentOffset;
    uint16_t ConstInCurrentSampleFreq;
} BMSSettings;

typedef struct {
    double UBattCurr          = 44.0;     // all values in Norm-Range, prevents alarms/erros on startup
    
    double UCellCurrMin       = 3.6;
    double UCellCurrAvg       = 3.6;
    double UCellCurrMax       = 3.6;
    double UCellCurrDelta     = 0.0;

    double IBattCurr          = 0.0;      // Charge < 0
    double IBattCurrCharge    = 0.0;
    double IBattCurrDischarge = 0.0;

    double TBattCurrMin       = 30.0;
    double TBattCurrMax       = 30.0;

    double QBattCurr;
    double QBattCurrKwh;
    double QCycleMeasuredKwh  = 0.0;
    double QCycleStartKwh     = 0.0;
    
    double IBattPlanChargeMax    = 0.1;   // there seems to be a check on those values, CCGX does not recognize BMS when value is zero
    double IBattPlanDischargeMax = 0.1;

    double SohBattCurr = 1.000;
    double SocBattCurr = 1.000;

    long CBmsWarnErrors = 2;   // if more then x communication errors on bms serial bus then report error
    
    byte State = 0; // 0 disconnect, 1 preload, 2 connect, 3 release preload, 4 charge, 5 discharge

    unsigned long Alarm = ERROR_NONE;
    unsigned long Error = ERROR_NONE;
} BMSStatus;
