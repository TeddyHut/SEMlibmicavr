# SEMlibmicavr
[SEMlibmodule][libmodule] hardware implementation targeting Microchip megaAVR 0-series microcontrollers.

## Overview
This repository provides the hardware side of [libmodule] for megaAVR 0-series microcontrollers.\
It has been designed for an ATmega3208, but should work with similar models with minor modifications.

#### Name Origin
Around the time when Microchip bought Atmel, Atmel released a series of AVRs with a different internal structure to previous models.
This internal structure seems to have taken inspiration from Atmel's ARM microcontrollers.
Considering this a new AVR for a "Microchip-Atmel" (whether true or not), and in need of a name, I named it lib**mic**avr.
The SEM comes from its use in the SEM project.

## Usage
SEMlibmicavr is intended to be used as a submodule in a project containing [libmodule].
- [ ] See SEMbms as an example.

#### Dependencies
SEMlibmodule/src/ must be in the include search path for libmicavr to work.

#### Integration
[SEMlibmodule][libmodule] requires that `timerhardware.h` is in the include search path.\
`timerhardware.cpp` and `generalhardware.cpp` will need to be added as source files to the project.

[libmodule]: https://github.com/TeddyHut/SEMlibmodule
