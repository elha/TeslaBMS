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
#define SERIALBMS Serial3 
#define INBMBFAULT 11

// Victron Bus
#define CANVE Can0 


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



typedef struct {
    uint8_t version;
    uint8_t logLevel;
    
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
    float IBattOptiDischargeMax;

    float TBattNormMin;
    float TBattNormMax;
} BMSSettings;

typedef struct {
    float UBattCurr          = 44.0f;     // all values in Norm-Range, prevents alarms/erros on startup
    
    float UCellCurrMin       = 3.6f;
    float UCellCurrAvg       = 3.6f;
    float UCellCurrMax       = 3.6f;
    float UCellCurrDelta      = 0.0f;

    float SocBattCurr = 0.0f;             // calculated on UCellNorm 0-100%
    float SohBattCurr = 100.0f;

    float IBattCurr          = 0.0f;      // Charge < 0
    float IBattCurrCharge    = 0.0f;
    float IBattCurrDischarge = 0.0f;

    float TBattCurrMin       = 30.0f;
    float TBattCurrMax       = 30.0f;

    float IBattPlanChargeMax    = 0.1f;   // there seems to be a check on those values, CCGX does not recognize BMS when value is zero
    float IBattPlanDischargeMax = 0.1f;

    unsigned long Alarm = ERROR_NONE;
    unsigned long Error = ERROR_NONE;
} BMSStatus;
