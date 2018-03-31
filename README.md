# TeslaBMS

Based off work done by Collin Kidder and Tom de Bree.


# Homestorage

Implementing a bridge between two Tesla Model S battery modules (24V each, serial resulting in 12s74p) and a victron energy setup using a CCGX and a MultiGrid 48/3000/35 inverter.

- Simplified Setup using a Teensy 3.5
- using the built-in CAN-interface 
- using 5V/3.3V level shifter for interfacing with the Tesla bus (serial)
- Current measured by ACS758
- Contactor activated by hobby servo on Error


trying to collect voltages and corresponding SOC-values (no guarantee) 

- Tesla modules:    18,0V,     18,6V,    23,1V,     24,6V,     24,9V
- per Cell:         3,00V,     3,10V,    3,85V,     4,10V,     4,15V
- SOC:                  0,        10,       70,        90,       100


# Parameters

- Curr  -> Measurements
- Plan  -> Planned values
- Opti  -> optimal Operating-Parameters 3,35V - 3,90V, 0,1C / 25A
- Norm  -> Shutoff normal Operation     3,25V - 4,00V, 0,0C / 0A,  39,6V - 48V 
- Warn  -> Warning CCGX                 3,15V - 4,10V   
- Err   -> Alarm CCGX                   under - over 

- naming convention on variables: 
  [I, U, T][Batt, Module, Cell][Curr, Plan, Opti, Norm, Warn, Err][Charge, Discharge][Min, Max, Avg]


# Control Algorithm:

The Victron CCGX is updated twice a second with IBattPlanChargeMax and IBattPlanDischargeMax by CAN.

## Charging + Discharging

Charging and Discharging in 4 stages
* is opti: Full Dis-/Charge
* or is norm: linear Dis-/Charge from Full to zero
* or is warn: zero Dis-/Charge and warning
* or is error (or worse: zero Dis-/Charge, error and switch contactor off 
