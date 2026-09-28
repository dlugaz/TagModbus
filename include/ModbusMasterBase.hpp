/*
 * ModbusDriver.hpp
 *
 *  Created on: Jul 14, 2025
 *      Author: kdluzynski
 */

#ifndef INC_MODBUSMASTERDRIVER_HPP_
#define INC_MODBUSMASTERDRIVER_HPP_


#include <map>
#include <IStreamDevice.hpp>

#include "ModbusFrame.hpp"

#include "ModbusRegisterBuffer.hpp"
#include "ModbusUtils.hpp"
#include "config.hpp"
namespace TagModbus {
	class MasterBase{
	protected:
		explicit MasterBase(IStreamDevice& serial_device);
		IStreamDevice& _streamDevice;
		bool isTCP = false;
		std::map<uint8_t,uint32_t> devicesBaudratesMap;

	public:

		uint32_t deviceResponseTime_ms = 30;
		const std::map<uint8_t, uint32_t>& devices_baudrates_map() const {
			return devicesBaudratesMap;
		}


		class Exception :public std::runtime_error {
		public:
			explicit Exception(const std::string_view message)
			:std::runtime_error(message.data()){};
		};
		class ModbusException:public Exception{
		public:
			TagModbus::FrameView::ExceptionCode _exception_code;
			explicit ModbusException(const TagModbus::FrameView::ExceptionCode exception_code)
			:Exception("Modbus Exception Code "+ std::to_string(exception_code)),_exception_code(exception_code)
			{};
		};
		class InvalidFrame:public Exception{
		public:
			TagModbus::FrameView::ValidationStatus _validation_status;
			explicit InvalidFrame(const TagModbus::FrameView::ValidationStatus validation_status)
			:Exception("Validation Failed: "+to_string(validation_status)),
			_validation_status(validation_status)
			{};
		};
		class StreamDeviceFailure:public Exception{
		public:
			SerialError _device_error;
			explicit StreamDeviceFailure(const SerialError device_error)
			:Exception("Stream Failure:" + to_string(device_error)),
			_device_error(device_error)
			{};
		};
		class ResponseTimeout:public Exception{

		};
		static TagModbus::MasterBase TCP(IStreamDevice& serial_device);

		static TagModbus::MasterBase RTU(IStreamDevice& serial_device);


		std::vector<uint16_t> read(uint8_t slave_ID, RegisterType register_type,uint16_t start_address,uint8_t quantity);

		void read(uint8_t slave_ID, const TagModbus::RegisterBufferView &outBuffer);

		void write(uint8_t slave_ID, RegisterType register_type, uint16_t start_address, std::span<const uint16_t> values);

		void sendFrame(TagModbus::FrameView &send_frame, uint16_t timeout_ms) const;

		size_t receiveFrame(TagModbus::FrameView &receive_frame, uint16_t timeout_ms) const;

		void sendReceiveFrame(TagModbus::FrameView &send_frame, TagModbus::FrameView &receive_frame);

		uint32_t getResponseTimeout(TagModbus::FrameView send_frame, uint32_t baud) const;

		uint32_t detectBaud(uint8_t slave_ID, std::span<const uint32_t> baudrates);

		std::map<uint8_t, uint32_t> scanForDevices(std::span<const uint32_t> baudrates, uint16_t timeoutMs = 10);

		static FrameView::FunctionCode getFunctionCode(bool isRead,RegisterType register_type);

	};
}


#endif /* INC_MODBUSMASTERDRIVER_HPP_ */
