# Tag Modbus 
###### Modbus RTU and TCP library designed for the ease of usage and low resource footprint

The goal is to abstract all the protocol specific information so that the user needs to know bare minimum about it.

Main concept it revolves around is **TAG** - instead of user manually interacting with modbus registers you create a
definition of memory that is in modbus registers and the library does all the conversions for you.

But also features powerful tools  to interact with raw byte buffers and interpret them as modbus frames - **zero copies**

### Features:

- **lightweight and modular** - user can choose at which level he wants to interact and how heavy he wants the library to be.
- **extendable and customizable** - define your own conversion data conversion functions or use the buildin ones 
- **Modbus RTU and TCP**
- **System or device-agnostic** - implement your own IStreamDevice interface or choose from one implemented
- **FrameView** - lightweight module for frame parsing - disect and debug frames peeking into raw byte buffers
- **RegisterBuffer** - convert data in modbus registers into C++ custom types 
- **MasterBase** - easily execute simple modbus master operations such as read, write registers
- **MasterTag** - automatically read and convert data from modbus slave based on your tag definitions

### Development Status:
This library is being actively developed. There may be breaking changes in all elements.

**FrameView, RegisterBuffer, MasterBase** are quite mature and ready to use.

**MasterTag** is work in progress, not ready to use.

**Tests** is work in progress, many things lack unit testing, or testing is outdated.

Planned development: SlaveBase, SlaveTag

### How To Install

It is designed to be used with CMAKE build system.

Clone this repository as a standalone 
```
git clone https://github.com/dlugaz/TagModbus.git
```

Add this repository as a submodule in your project
```
git submodule add TagModbus https://github.com/dlugaz/TagModbus.git
git submodule update --init --recursive TagModbus 
```

### Examples:

#### MasterTag

```
//Define your TAGs with basic modbus information
enum TagsIDs {
	myTag1ID,
	myTag2ID
};
TagModbus::Tag myTag{
	.register_type =TagModbus::RegisterType::Holding,
	.register_number = 123,
	.register_length = 2,
	.register_value_type = modbus_parameter_type::U32,
	.key = myTag1ID
};
TagModbus::Tag myTag2{
	.register_type =TagModbus::RegisterType::Holding,
	.register_number = 200,
	.register_length = 25,
	.register_value_type = modbus_parameter_type::ASCII,
	.key = myTag2ID
};


void readFromModbus() {
	//Initialize your streaming medium - can be TCP, RTU, or any medium that can pass data actually
	PosixSerialStreamDevice serialStream {};
	serialStream.open_port("TTYusb0");

	auto modbusMaster = TagModbus::MasterTag::RTU(serialStream);
	//Register your tags dictionary - it's optional but highly recommended to improve efficiency of the requests
	modbusMaster.registerTags({myTag,myTag2});

	//Read the value directly to cpp type with automatic conversion
	uint32_t myTag1Value = modbusMaster.read(
		/* slaveID */123,
		/* TagID to read */myTag1ID);
	std::string myTag2Value = modbusMaster.read(
		/* slaveID */123,
		/* TagID to read */myTag2ID);
	//Read multiple values into a TagValueMap - library will automatically generate and optimize requests
	TagModbus::MasterTag::TagValueMap readOut = modbusMaster.read(
		/* slaveID */123,
		/* Multiple TagIDs to read */{myTag1ID,myTag2ID});

	myTag1Value = readOut[myTag1ID];
	//You can also try to force interpretation to some other type with .as
	myTag2Value = readOut[myTag2ID].as<std::string>();
}
}
```




### Inspirations
#### ArduinoJSON https://github.com/bblanchon/ArduinoJson - data type conversion model
#### Mazurel/ModbusCPP https://github.com/Mazurel/Modbus - 


### Licence