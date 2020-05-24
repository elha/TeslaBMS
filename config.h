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
#define CAN_DEV CAN0
#define CAN_PIN DEF         // TX 3, RX 4

// Status LED
#define LED  13       

// Currentsensor
#define INCURRENT A1

#define COOLER   38
// #define PRELOAD  38
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
static const float QCellSpecCurve0_2C[SizeCellSpecCurve0_2C][2] = { 
        {  2.50f,      0.0f  },
        {  2.60f,     15.0f  },
        {  2.70f,     30.0f  },
        {  2.80f,     52.0f  },
        {  2.85f,     64.0f  },
        {  2.90f,     80.0f  },
        {  2.95f,    102.0f  },
        {  3.00f,    121.0f  },
        {  3.05f,    147.0f  },
        {  3.10f,    180.0f  },
        {  3.15f,    221.0f  },
        {  3.20f,    278.0f  },
        {  3.25f,    360.0f  }, // Norm min 39.00
        {  3.30f,    459.0f  },
        {  3.35f,    577.0f  },
        {  3.40f,    736.0f  },
        {  3.45f,    928.0f  },
        {  3.50f,   1184.0f  },
        {  3.55f,   1452.0f  },
        {  3.60f,   1708.0f  },
        {  3.70f,   2105.0f  },
        {  3.80f,   2471.0f  },
        {  3.90f,   2804.0f  },
        {  4.00f,   3087.0f  },
        {  4.02f,   3143.0f  },
        {  4.06f,   3283.0f  }, // Norm max 48.72
        {  4.08f,   3331.0f  },
        {  4.10f,   3361.0f  },
        {  4.12f,   3378.0f  },
        {  4.20f,   3393.0f  }};

typedef struct {
    uint8_t version;
    uint8_t logLevel;
    
    uint8_t ConfigBattParallelCells;
    uint8_t ConfigBattSerialCells;
    uint8_t ConfigBattParallelStrings;
    
    float UBattNormMin;
    float UBattNormMax;

    float UCellWarnMin;
    float UCellNormMin;
    float UCellOptiMin;

    float UCellOptiMax;
    float UCellNormMax;
    float UCellWarnMax;

    float UCellNormBalanceDiff;
    float UCellWarnBalanceDiff;

    float IBattWarnChargeMax;
    float IBattWarnDischargeMax;

    float IBattOptiChargeMax;
    float IBattOptiChargeMin;
    float IBattOptiDischargeMax;

    float TBattNormMin;
    float TBattOptiMax;
    float TBattNormMax;
    float TBattWarnMax;

    float QBattNormMin;
    float QBattNormMax;    
    float QBattNorm;
    float QBattNormKwh;

    float ConstInCurrentOffset;
    float ConstInCurrentSensitivity;
    uint16_t ConstInCurrentSampleFreq;
} BMSSettings;

typedef struct {
    float UBattCurr          = 44.0f;     // all values in Norm-Range, prevents alarms/erros on startup
    
    float UCellCurrMin       = 3.6f;
    float UCellCurrAvg       = 3.6f;
    float UCellCurrMax       = 3.6f;
    float UCellCurrDelta     = 0.0f;

    float IBattCurr          = 0.0f;      // Charge < 0
    float IBattCurrCharge    = 0.0f;
    float IBattCurrDischarge = 0.0f;

    float TBattCurrMin       = 30.0f;
    float TBattCurrMax       = 30.0f;

    float QBattCurr;
    float QBattCurrKwh;
    float QCycleMeasuredKwh  = 0.0f;
    float QCycleNormStartKwh = 0.0f;
    float QCycleNormKwh      = 0.0f;
    
    float IBattPlanChargeMax    = 0.1f;   // there seems to be a check on those values, CCGX does not recognize BMS when value is zero
    float IBattPlanDischargeMax = 0.1f;

    float SohBattCurr = 1.000f;
    float SocBattCurr = 1.000f;

    long CBmsWarnErrors = 2;   // if more then x communication errors on bms serial bus then report error
    
    byte State = 0; // 0 disconnect, 1 preload, 2 connect, 3 release preload, 4 charge, 5 discharge

    unsigned long Alarm = ERROR_NONE;
    unsigned long Error = ERROR_NONE;
} BMSStatus;
