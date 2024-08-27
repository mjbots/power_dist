# mjbots power_dist and mjpower-ss #

This repository contains hardware and firmware for the mjbots
power_dist and mjpower-ss boards.  Both provide pre-charge,
over-current limiting and connector fanout.  The power_dist
additionally provides energy monitoring and a soft-switch
functionality.

# Specifications #

| Name                     | power_dist r4.5b | mjpower-ss r2 |
| ------------------------ |------------------|---------------|
| Voltage Input            | 10-44V           | 10-54V        |
| Current (continous/peak) | 45A/90A          | 45/90A        |
| Output                   | 6x XT30          | 6x XT30       |
| Max load capacitance     | 4000uF           | 4000uF        |
| Quiescent current        | 300uA            | 200uA         |
| Dimensions               | 80x50mm          | 70x50mm       |
| Mass                     | 36g              | 34g           |
| External switch          | X                | X             |
| Soft switch              | X                |               |
| Onboard switch           |                  | X             |
| CAN communication        | X                |               |

# Documentation #

* [Reference](docs/reference.md)
