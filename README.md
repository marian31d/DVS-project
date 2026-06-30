# DVS-Project

A SystemVerilog-based hardware design and verification project for a four-port packet switch.

## Overview

DVS-Project implements a complete RTL design, verification environment, and synthesis flow for a simplified four-port packet switch. The switch accepts packets on any of its four ports and forwards them to one or more destination ports according to fields encoded in each packet.

## Technology Stack

- **SystemVerilog** (94.6%): Core hardware design and verification code
- **Makefile** (5.4%): Build automation and compilation scripts

## Project Components

- **RTL Design**: 4-port switch module with packet handling
- **Verification Environment**: Complete UVM-based testbench with:
  - Agents (driver, sequencer, monitor)
  - Packet data models
  - Checkers and assertions
  - Test suites for functional validation
- **Synthesis**: Design flow for hardware implementation
