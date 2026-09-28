//
// Created by kdluzynski on 07.10.2025.
//

#ifndef MODBUSREGISTERBUFFER_HPP
#define MODBUSREGISTERBUFFER_HPP
#include <cstdint>
#include <span>
#include <stdexcept>
#include <vector>

#include "ModbusTag.hpp"
#include "ModbusUtils.hpp"

namespace TagModbus {
	class RegisterBufferView {

    public:
        using RegistersValueType = uint16_t;
        constexpr explicit RegisterBufferView(const uint16_t& startAddress, const RegisterType registerType,const std::span<RegistersValueType> container):
        startAddress_ {startAddress},buffer_{container}, registerType_{registerType}{}

        template<typename T, TagModbus::ByteOrder Order = TagModbus::ByteOrder::MSB>
        constexpr void put(const uint16_t modbus_address, const T& value) const{
            // The conversion function must be constexpr
            convertToRegisters<T, Order>(
                get_buffer_for_address(modbus_address,requiredRegisters<T>(value)),
                value
            );
        }
        template<typename T, TagModbus::ByteOrder Order = TagModbus::ByteOrder::MSB>
        constexpr void put(const Tag& tag, const T& value) const{
            // The conversion function must be constexpr
            convertToRegisters<T, Order>(
                get_buffer_for_address(tag.register_number,tag.register_length),
                value
            );
        }

        template<typename T, TagModbus::ByteOrder Order = TagModbus::ByteOrder::MSB>
        constexpr T get(const uint16_t modbus_address) const {
            // The conversion function must be constexpr
            return convertFromRegisters<T, Order>(
                get_buffer_for_address(modbus_address,requiredRegisters<T>())
            );
        }

    	template<typename T, TagModbus::ByteOrder Order = TagModbus::ByteOrder::MSB>
		constexpr T get(const Tag& tag) const {
        	return convertFromRegisters<T, Order>(get_buffer_for_address(tag.register_number,tag.register_length));
        }

        template<typename T>
        constexpr void get_into(const uint16_t modbus_address, T &destination) const
        {
            convertFromRegistersTo(get_buffer_for_address(modbus_address,requiredRegisters(destination)),destination);
        }
	    template<typename ElementType, std::size_t Extent>
        constexpr void get_into(const uint16_t modbus_address, std::span<ElementType, Extent> destination) const
        {
            convertFromRegistersTo(get_buffer_for_address(modbus_address, requiredRegisters(destination)), destination);
        }

	    template<typename ElementType, std::size_t N>
        constexpr void get_into(const uint16_t modbus_address, ElementType (&destination)[N]) const
        {
            get_into(modbus_address, std::span<ElementType, N>(destination));
        }

        constexpr uint16_t startAddress() const {
            return startAddress_;
        }
        constexpr std::span<RegistersValueType> buffer() const {
            return buffer_;
        }
        constexpr RegisterType registerType() const {
            return registerType_;
        }
        // constexpr std::span<RegistersValueType> get_buffer_for_address(uint16_t modbus_address) const {
        //     const uint16_t offset = calculate_offset(modbus_address);
        //     return buffer_.subspan(offset);
        // }
    	constexpr std::span<RegistersValueType> get_buffer_for_address(const uint16_t modbus_address, const size_t length) const {
        	const uint16_t offset = calculate_offset(modbus_address);

        	const auto longSpan = buffer_.subspan(offset);
        	if (longSpan.size() < length) {
        		throw std::out_of_range("Buffer insufficient for Tag length");
        	}
        	return longSpan.subspan(0, length);
        }
    private:
        const uint16_t startAddress_ = 0;
        const std::span<RegistersValueType> buffer_;
        const RegisterType registerType_;


        constexpr uint16_t calculate_offset(const uint16_t modbus_address) const {
            if (modbus_address < startAddress_) {
                throw std::out_of_range("Modbus address is below buffer start address.");
            }
            uint16_t offset = modbus_address - startAddress_;


            if (offset > buffer_.size() || offset > MAX_MODBUS_REGISTERS) {
                throw std::out_of_range("Modbus address exceeds buffer size");
            }
            return offset;
        }
    };

    class RegisterBuffer {
    public:
        using RegistersValueType = uint16_t;

        RegisterBuffer(const uint16_t startAddress, const RegisterType registerType, const uint16_t numRegisters)
            : startAddress_{startAddress},
                registerType_(registerType),
              registersValue_(numRegisters)
        {
        }

        RegisterBufferView view() {
            return RegisterBufferView(startAddress_,registerType_, registersValue_);
        }

        template<typename T, TagModbus::ByteOrder Order = TagModbus::ByteOrder::MSB>
        constexpr void put(const uint16_t modbus_address, const T& value) {
            view().put<T, Order>(modbus_address, value);
        }

        template<typename T, TagModbus::ByteOrder Order = TagModbus::ByteOrder::MSB>
        constexpr T get(const uint16_t modbus_address){
            return view().get<T, Order>(modbus_address);
        }

        template<typename T, TagModbus::ByteOrder Order = TagModbus::ByteOrder::MSB>
        constexpr void get_into(uint16_t modbus_address, T &destination)
        {
            view().get_into<T, Order>(modbus_address,destination);
        }

        uint16_t startAddress_;
        RegisterType registerType_;
        std::vector<RegistersValueType> registersValue_; // 2. The data is owned by this class.
    };
}

#endif //MODBUSREGISTERBUFFER_HPP