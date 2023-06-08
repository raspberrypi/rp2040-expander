# rp2040-expander

This repo contains the source code for the RP2040 based port-expander known as
'rp2040-expander'.  Some demonstration modules, which make use of the
rp2040-expander's API, are also included in this repo.

## Introduction

rp2040-expander is a minimal software environment which utilizes a Raspberry
Pi RP2040 microcontroller as a port-expander peripheral for a host processor.

The RP2040 has no attached SPI Flash, may not execute any software and is
unlikely to require a quartz crystal (subject to the intended use, please
see: LINK whitepaper here).

The rp2040-expander is controlled by the host using the RP2040's 'Serial Wire
Debug' interface over which 'Debug Access Port' transactions are used to
control the internal RP2040 peripherals _or_ to download a program to RP2040 RAM.
Please see the LINK whitepaper here for RAM downloaded program details.

## Contents

Generic source code is supplied in the `/host` directory to provide the necessary
SWD, DAP and higher level control primitives and to provide the functional API to
the host system.

The `/port_xxx` directories contain host specific code that provide per-platform
support, i.e. GPIO manipulations to implement the host side SWD interface.

The `/demo` directory contains the modules used to exercise some of the
rp2040-expander's API.

## SWD interface

The SWD interface is 'bit-bashed' and requires a few supporting functions to
perform the GPIO manipulations required to provide the transfers over SWD.

A port of this software requires that the user provide these functions for
their intended host platform.

Two programmable GPIOs are required on the host, one is used only as an output
from the host, and drives the SWCLK signal, the other GPIO must be bidirectional
and is used for the SWDIO signal.

## DAP interface

The Debug Access Ports within the RP2040 provide access to internal debug
resources and using them it is possible read and write to the RP2040's
memory and internal registers such as GPIO control registers.

In this application only a small subset of the RP2040 DAP functionality is
required and the code contained here supports that subset and no more.

## Host platforms supported

Support for _two_ Host platforms are provided within this repo:

- An RP2040 hosted demo; that is running the demo software on an RP2040 which
  in turn controls a second RP2040 which performs the expander function.

Any available GPIO ports can be used on the host RP2040, the source here uses
GPIO2 and GPIO3.

You can connect any suitable RP2040 boards, most obviously two Pico boards.
(INSERT interconnect diagram here)

- A Raspberry Pi computer hosted demo; that is running the demo on a Pi computer
  (e.g. Pi 3, Pi 4 ) controlling an RP2040 which performs the expander function.

 Any available GPIO ports can be used; the source here uses GPIO17 and GPIO27.
(INSERT interconnect diagram here)
