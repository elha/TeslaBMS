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

## Charging Algorithm:

The charging current is controlled by the Maximum charging current parameter. 
It’s calculated as Charging Coefficient('C','H','A',’C') x Battery capacity. 
The parameter has an upper limit which is defined as Maximum Charging Current per Device ('M','A','X',’C') x Number of Devices ('S','I','S',’N').
When any cell reaches the voltage interval between Balance Voltages Start and Balance Voltage End, the charging current starts to ramp down to 1.1 A x Number of Devices until the last cell rises to the End of Charge Voltage. At that point the Maximum charging voltage is set to Number of cells x (End of Charge Voltage per cell - end of charge hysteresis per cell) and the charger is disabled also via the BMS I/O interface. End of Charge, SOC hysteresis and End of charging cell voltage hysteresis prevent unwanted switching. Charger turn-off can also be caused by some of the systems errors (See System Errors indication chapter).
SOC is calibrated to 96 % at the 0.502 x value between Balance Voltages Start and Balance Voltage End.


## Discharging Algorithm:

Calculated maximum discharging current is sent to the Color Control GX by CAN communication in every measurement cycle. When the BMS starts/recovers from the error or from Discharging SOC hysteresis, maximum allowed discharging current is set. It is calculated as discharging coefficient ('D','C','H',’C') x Battery capacity. If this value is higher than maximum discharging current per device ('M','A','X',’D') x number of devices ('S','I','S',’N'), maximum discharging current is decreased to this value. When the lowest open circuit voltage cell is discharged bellow the set threshold 'C','L','O',’W', the maximum discharging current starts to decrease down to 0.05 C (5 % of Capacity in A). After decreasing down, maximum allowed discharging current is set to 0 A. SOC is reset to 3 % and Discharging SOC hysteresis is set to 5 %. If the cell discharges bellow Minimum Cell voltage ('C','M','I','N'), BMS signals Error 2 and SOC is reset to 0 %. If the Charger/inverter is connected to the grid maximum allowed discharge current is drawn from the grid. Otherwise 100 % load current is drawn from the battery until maximum allowed discharging current is set to 0 A. 