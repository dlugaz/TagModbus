#pragma once

#include "../config.hpp"
#include <string>
#include <ModbusUtils.hpp>

#include "ModbusTagValue.hpp"
enum class modbus_parameter_type:uint8_t {
	U8 = 0x00,
	U16 = 0x01,
	U32 = 0x02,
	FLOAT = 0x03,
	ASCII = 0x04,
	U8_LSB = 0x07,
	U8_MSB = 0x08,
	BOOL = 0x09,
	BYTE_ARRAY = 0x0A,
};
namespace TagModbus {
	struct Tag {
		RegisterType register_type;
		uint16_t register_number;
		uint16_t register_length;
		modbus_parameter_type register_value_type;
		TagID key;
	};
	static_assert(std::is_trivially_copyable_v<Tag>);

}
