mjbots power dist reference

# A. Operation #

This documentation refers to the most recent (r4.3b) version of the
board.

The power dist board provides pre-charging, energy monitoring,
over-current protection, a soft power switch, and a power connector
fanout for use in robotic applications.  The input can be directly
connected to a battery or power supply.  The output can be connected
to high capacitance loads.

Additionally, a CAN-FD port allows host software to monitor the state
of the switch, input voltage, power, and energy, and request that
shutdown be delayed to accomplish a "soft power down".

# A. Register command set #

The power_dist uses the same register framing and CAN-FD as the mjbots moteus controllers, with alternate definitions for the registers.  [moteus reference](https://github.com/mjbots/moteus/blob/main/docs/reference.md#a-register-command-set)

## A.2 ##

The following additional mappings are provided.

### A.2.a Energy (measured in W*hr) ###

- int8 => 1 LSB => 1 W*hr
- int16 => 1 LSB => 0.01 W*hr
- int32 => 1 LSB => 0.000001 W*hr

## Registers ##

### 0x000 - State ###

Mode: Read/write

The current operational state of the power_dist.  Only some values may
be written.

- 0 => power off
- 1 => precharging (read only)
- 2 => power on
- 3 => fault (read only)

If power off is commanded over CAN, the device will not enter sleep,
which differs from the behavior when power off is achived through the
physical switch.

### 0x001 - Fault Code ###

Mode: Read only

A 7 bit fault code.

### 0x002 - Switch status ###

Mode: Read only

Zero if the switch is current turned off, non-zero otherwise.

### 0x003 - Lock time ###

Mode: Read/write

Power will be maintained for this long after the switch has been
turned off.

### 0x004 - Boot time ###

Mode: Read

Time since the processor was powered.

### 0x010 - Output Voltage ###

Mode: Read only

The current output voltage

### 0x011 - Output Current ###

Mode: Read only

The current output current.

### 0x012 - Temperature ###

Mode: Read only

The current output FET temperature.

### 0x013 - Energy ###

Mode: Read only

Total energy provided to the downstream port since power was enabled.

# B. diagnostic command set #

All `tel` and `conf` class commands from [moteus
reference](https://github.com/mjbots/moteus/blob/main/docs/reference.md#b-diagnostic-command-set)
are supported.  In addition, the following diagnostic mode commands
are available.

## `d lock` ##

Request that the lock time be set to the given number of seconds.

```
d lock <time_in_seconds>
```


# C. Mechanical / Electrical #

## Mechanical ##

4x M2.5 mounting holes are provided in a 74mm x 44mm rectangular
pattern.

## Pinout ##

### XT-90 Input ###

The XT-90 connector has a `-` and a `+` imprint on the housing.

### XT-30 Output ###

The XT-30 connectors have a `-` and a `+` imprint on the housing.

### JST PH-4 Switch ###

Looking at the pins of the connector from the top with the mjbots logo
right side up the pins are numbered from right to left.

 - 1 - LED+ - LED positive
 - 2 - LED- - LED ground
 - 3 - SWP - Switch positive
 - 4 - SWG - Switch ground

### JST PH-3 CAN ###

Looking at the pins of the connector with the mjbots logo right side
up, the pins are numbered from the bottom to the top.

 - 1 - CAN_H
 - 2 - CAN_L
 - 3 - GND

NOTE 1: CAN connections should be terminated by a 120 ohm resistor at
both ends of a bus.  Some mjbots products have built in termination
resistors, such as the pi3hat.  The fdcanusb has a software
configurable termination resistor that is by default on.  The
power_dist board has no termination resistors.  For very short runs,
the system will work terminated only on one side.  However, when runs
become longer than 0.5m, you will likely need to terminate both ends.
This can be done by crimping a 120 ohm resistor into a JST PH3
connector and connecting it to the open data connector.

NOTE 2: Ground may not be necessary, only one path through ground in a
system should exist to avoid ground loops.  In a typical robot
application with a common ground, that role is filled by the power
ground.  However, in desktop applications, it may be appropriate to
connect the CAN ground if the device power supply is otherwise
isolated.


### JST ZH-6 SWD ###

Looking at the pins of the connector with the mjbots logo right side
up the pins are numbered 1 to 6 from top to bottom.

 - 1 - NC
 - 2 - NRST
 - 3 - SWDIO
 - 4 - GND
 - 5 - SWCLK
 - 6 - 3.3V
