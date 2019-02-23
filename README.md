# TeslaBMS

Based off work done by Collin Kidder and Tom de Bree.


# Homestorage

Implementing a bridge between four Tesla Model S battery modules (24V each, serial resulting in 2p2s-6s74p) and a victron energy setup using a CCGX and a MultiGrid 48/3000/35 inverter.

- Simplified Setup using a Teensy 3.5
- using the built-in CAN-interface 
- using 5V/3.3V level shifter for interfacing with the Tesla bus (serial)
- Current measured by ACS758
- Contactor activated by hobby servo on Error


# Parameters

- Curr  -> Measurements
- Plan  -> Planned values
- Opti  -> optimal Operating-Parameters 3,30V - 4,00V, 0,1C / 35A
- Norm  -> Shutoff normal Operation     3,25V - 4,06V, 0,0C / 0A
- Warn  -> Warning (disabled on CAN)    3,20V - 4,10V   
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
