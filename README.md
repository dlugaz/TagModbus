# Tag Modbus 
###### Modbus RTU and TCP library designed for the ease of usage and low resource footprint

The goal is to abstract all the protocol specific information so that the user needs to know bare minimum about it.

Main concept it revolves around is **TAG** - instead of user manually interacting with modbus registers you create a
definition of memory that is in modbus registers and the library does all the conversions for you.

But also features powerful tools  to interact with raw byte buffers and interpret them as modbus frames - **zero copies**

### Features:
- **lightweight and multilayered** - user can choose at which level he wants to interact and how heavy he wants the library to be.
- **extendable** - define your 
- **Modbus RTU and TCP**
- **System or device-agnostic** - implement your own IStreamDevice interface or choose from one implemented
- **FrameView** - disect and debug frames peeking into raw byte buffers
- **RegisterBuffer** - convert data in modbus registers into C++ custom types 
- **MasterBase** - easily execute simple modbus master operations such as read, write registers
- **MasterTag** - automatically read and convert data from modbus slave based on your tag definitions

### Examples:

#### MasterTag

`#include "ModbusMasterTag.hpp"

TagModbus::Tag myTag {


}`





## Why?

I started it, because I had multiple modbus devices and ended up reimplementing much of the code. Tried multiple libraries, but most are C only. I really liked Mazurel/ModbusC++ library, but then I needed to create a fast embedded RTU to TCP converter and I needed something that is zerocopy.

## What I want to achieve:

* zero cost abstractions
* low resources footprint
* modular:
  * lightweight module for frame parsing
  * lightweight module for embedded basic drivers
  * efficient but not-so-lightweight module for ease of use. Using as much 
* easy to use

## Contents:

* **ModbusFrame.hpp** - a header only parser and builder for modbus frames. It consists of eModbus::FrameView and eModbus::Frame, where View is nonowning, and Frame is owning. Allows for fast and on the spot (zerocopy) edit of all the fields of modbus frame. Allows to build custom modbus drivers.
* **IStreamDevice.hpp** - Interface that needs to be implemented to use more advanced modbus drivers.
* **ModbusMasterBase.hpp** - the simplest modbus master driver. Allows to send and receive modbus frames via IStreamDevice
* **ModbusRegisterBuffer.hpp** - utility that simplify access to data coded in the registers. Allows to convert the registers to custom data such as (u)int8/16/32, ascii, byte buffers or user defined.
* **ModbusMasterTag.hpp** - modbus master driver that's tag based. Define a repository of tags with register types and numbers, and read them efficiently without a thought about modbus internals.

## Current State:
**This is NOT production ready library**
* ModbusFrame - OK. TODOs:
  * needs separation into frame and frameview
  * it would be great to be able to create a constexpr frames.
* ModbusMasterBase - OK. TODOs:
  * has a FreeRTOS only mutex.
* ModbusRegisterBuffer - OK TODOs:
  * it would be great to be able to create a constexpr frames.
* ModbusTag - WIP. TODOs:
  * resolve how to store and retrieve tag information efficiently
  * resolve how to deal with async calls (for Qt driver) 
* ModbusSlaveBase - TODO
* ModbusSlaveTag - TODO

## Use Cases:
TODO
