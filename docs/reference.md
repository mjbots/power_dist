mjbots power_dist and mjpower-ss reference

# A. Operation #

The `power_dist` and mjpower-ss boards provides pre-charging, an
external power switch, and a power connector fanout for use in robotic
applications.  Additionally, the `power_dist` provides energy
monitoring and over-current protection, and a soft switch capability.

The input can be directly connected to a battery or power supply.  The
output can be connected to high capacitance loads.

For the `power_dist`, a CAN-FD port allows host software to monitor
the state of the switch, input voltage, power, and energy, and request
that shutdown be delayed to accomplish a "soft power down".  This
documentation refers to the most recent (r4.5b) version of the
`power_dist`.  r4.3 versions are largely compatible with this as well.

# A. Register command set (power_dist only) #

The `power_dist` uses the same register framing and CAN-FD as the mjbots moteus controllers, with alternate definitions for the registers.  [moteus reference](https://github.com/mjbots/moteus/blob/main/docs/reference.md#a-register-command-set)

The default ID for the power_dist is '32', (which notably differs from
the default id of 1 for the moteus controller).

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
turned off.  The maximum value is 3276.7s, all register scalings
report in units of 100ms (thus int8 will not be able to command or
monitor the full possible scale).

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

# B. diagnostic command set (power_dist only) #

All `tel` and `conf` class commands from [moteus
reference](https://github.com/mjbots/moteus/blob/main/docs/reference.md#b-diagnostic-command-set)
are supported.  In addition, the following diagnostic mode commands
are available.

## `p lock` ##

Request that the lock time be set.

```
p lock <time_in_100ms>
```


# C. Mechanical / Electrical #

## Mechanical ##

`power_dist`: 4x M2.5 mounting holes are provided in a 74mm x 44mm rectangular pattern.

`mjpower-ss`: 4x M2.5 mounting holes are provided in a 64mm x 44mm rectangular pattern.

## Pinout ##

### XT-90 Input ###

The XT-90 connector has a `-` and a `+` imprint on the housing.

### XT-30 Output ###

The XT-30 connectors have a `-` and a `+` imprint on the housing.

### JST PH-4 Switch ###

A small white dot is present on the silk screen near pin 1.

 - 1 - LED+ - LED positive
 - 2 - LED- - LED ground
 - 3 - SWG - Switch ground
 - 4 - SWP - Switch positive

### Onboard Switch (mjpower-ss only) ###

The onboard switch is connected in parallel to the external switch.
If either the onboard switch is set to the ON position, or the
external switch pins are tied together, the board will pre-charge and
provide power to the output when the input voltage reaches a small
threshold below the minimum input voltage.

### JST PH-3 CAN (power_dist only) ###

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


### JST ZH-6 SWD (power_dist only) ###

Looking at the pins of the connector with the mjbots logo right side
up the pins are numbered 1 to 6 from top to bottom.

 - 1 - NC
 - 2 - NRST
 - 3 - SWDIO
 - 4 - GND
 - 5 - SWCLK
 - 6 - 3.3V

# D. Maintenance (power_dist only) #

## Building firmware ##

Building is supported on Ubuntu 20.04 with the following command:

```
tools/bazel test //:target
```

## Flashing firmware ##

A firmware image (.elf file), can be flashed from a linux PC using the
6-pin SWD debug connector.

One time:

```
sudo apt install binutils-arm-none-eabi
```

Ensure that power is applied, the switch is turned on, and no loads
are connected, then:

```
fw/flash.py optional/path/to/file.elf
```

## openocd ##

As with moteus, openocd 0.11.0 or newer is required, see:

https://github.com/mjbots/moteus/blob/main/docs/reference.md#openocd
