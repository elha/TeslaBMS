# TeslaBMS

Based off work done by Collin Kidder and Tom de Bree.

# Homestorage

Implementing a bridge between two Tesla Model S battery modules (24V each) and a victron energy setup using a CCGX and a MultiGrid 48/3000/35 inverter.

- Simplified Setup using a Teensy 3.5
- using the built-in CAN-interface 
- using 5V/3.3V level shifter for interfacing with the Tesla bus (serial)
- Current measured by ACS758
- Contactor activated by hobby servo on Error

# Parameters

- Op    -> sets Operating-Parameters for CCGX 
- Normal-> Boundaries for Operation
- Warn  -> Warning CCGX
- Error -> Alarm CCGX


# Control Algorithm:

The Victron CCGX is updated every second with operating parameters by CAN.
