//
// Created by kdluzynski on 12.08.2025.
//
#include "ModbusMasterBase.hpp"
#include "ModbusRegisterBuffer.hpp"
TagModbus::MasterBase::MasterBase(IStreamDevice &serial_device):_streamDevice(serial_device) {
}


TagModbus::MasterBase TagModbus::MasterBase::TCP(IStreamDevice &serial_device) {
    TagModbus::MasterBase result(serial_device);
    result.isTCP = true;
    return result;
}

TagModbus::MasterBase TagModbus::MasterBase::RTU(IStreamDevice &serial_device) {
    TagModbus::MasterBase result(serial_device);
    result.isTCP = false;
    return result;
}

std::vector<uint16_t> TagModbus::MasterBase::read(const uint8_t slave_ID, const RegisterType register_type,
    const uint16_t start_address, const uint8_t quantity) {
    TagModbus::Frame frame = TagModbus::Frame::build(
        true,
        slave_ID,
        getFunctionCode(true,register_type),
        start_address,
        quantity);
    TagModbus::Frame receiveFrame{false};
    sendReceiveFrame(frame,receiveFrame);

    return receiveFrame.registersValues();
}

void TagModbus::MasterBase::read(const uint8_t slave_ID, const TagModbus::RegisterBufferView &outBuffer) {
    std::ranges::copy(read(
                            slave_ID,
                            outBuffer.registerType(),
                            outBuffer.startAddress(),
                            outBuffer.buffer().size()),
                    outBuffer.buffer().begin());
}


void TagModbus::MasterBase::write(const uint8_t slave_ID,const RegisterType register_type,const uint16_t start_address,
    const std::span<const uint16_t> values) {
    TagModbus::Frame frame = TagModbus::Frame::build(
        true,
        slave_ID,
        getFunctionCode(false,register_type),
        start_address,
        values.size(),values);
    TagModbus::Frame receiveFrame{false};
    sendReceiveFrame(frame,receiveFrame);

}

void TagModbus::MasterBase::sendFrame(TagModbus::FrameView &send_frame, const uint16_t timeout_ms) const {
	_streamDevice.flush();
    const SerialError err = _streamDevice.write(
        isTCP ? send_frame.tcpFrame() : send_frame.rtuFrame(), timeout_ms);
    if (err != SerialError::SUCCESS)
        throw StreamDeviceFailure(err);
}

size_t TagModbus::MasterBase::receiveFrame(TagModbus::FrameView &receive_frame, const uint16_t timeout_ms) const {
    receive_frame.isRequest(false);
	size_t bytes_read = 0;
	size_t total_bytes_read = 0;
	SerialError err = SerialError::SUCCESS;

	do  {
		auto target_buffer = isTCP ? receive_frame.buffer() : receive_frame.rtuBuffer();
		target_buffer = target_buffer.subspan(total_bytes_read);
		err = _streamDevice.read(target_buffer,
									 timeout_ms,&bytes_read);
		total_bytes_read += bytes_read;
		if (err != SerialError::SUCCESS) {
			throw StreamDeviceFailure(err);
		}
	}while (bytes_read && total_bytes_read < receive_frame.RTULength());

	return total_bytes_read;
}

void TagModbus::MasterBase::sendReceiveFrame(TagModbus::FrameView &send_frame, TagModbus::FrameView &receive_frame) {

    uint16_t slave_ID = send_frame.slaveID();
    uint32_t baud = 0;

    if (!devicesBaudratesMap.contains(slave_ID)) {
        baud = detectBaud(slave_ID, defaultBaudrates);
        if (baud == 0)
            throw StreamDeviceFailure(SerialError::TIMEOUT);;
    } else {
        baud = devicesBaudratesMap[slave_ID];
    }
    _streamDevice.baudrate(baud);
    sendFrame(send_frame, send_frame.calculateTransmissionTimeMs(baud) * 2);
    receiveFrame(receive_frame, getResponseTimeout(send_frame, devicesBaudratesMap[slave_ID]));

    TagModbus::Frame::ValidationStatus validation = receive_frame.validateRTU();
    if (validation != TagModbus::Frame::ValidationStatus::OK) {
    	printf("Receive frame invalid %s",receive_frame.toString().c_str());
    	throw InvalidFrame(validation);
    }
	validation = receive_frame.validateResponse(send_frame);
	if (validation != TagModbus::Frame::ValidationStatus::OK) {
		printf("Response Validation failed %s",receive_frame.toString().c_str());
		throw InvalidFrame(validation);
	}
}

uint32_t TagModbus::MasterBase::getResponseTimeout(TagModbus::FrameView send_frame, const uint32_t baud) const {
    return send_frame.calculateResponseTransmissionTimeMs(baud) + deviceResponseTime_ms;
}

uint32_t TagModbus::MasterBase::detectBaud(const uint8_t slave_ID, const std::span<const uint32_t> baudrates) {

    TagModbus::Frame send_frame = TagModbus::Frame::build(true, slave_ID, TagModbus::Frame::FunctionCode::ReadInputRegisters, 0, 1);
    TagModbus::Frame receive_frame(false);
    uint32_t working_baud = 0;
    uint32_t originalBaud = _streamDevice.baudrate();
    if (originalBaud != IStreamDevice::InvalidBaudrate) {
        for (const auto baud : baudrates) {
            _streamDevice.baudrate(baud); // Set the new baud rate for testing

            SerialError err = _streamDevice.write(send_frame.rtuFrame(), send_frame.calculateTransmissionTimeMs(baud)*2);
            if (err != SerialError::SUCCESS) {
                break;
            }

            err = _streamDevice.read(receive_frame.rtuBuffer(), getResponseTimeout(send_frame, baud));
            if (err == SerialError::TIMEOUT) {
                continue;
            }
            if (err != SerialError::SUCCESS) {
                break;
            }

            if (receive_frame.validateRTU() == TagModbus::Frame::ValidationStatus::OK) {
                // Success! We found the working baud rate.
                working_baud = baud; // Return the working baud and leave it set.
                break;
            }
        }
        _streamDevice.baudrate(originalBaud);
    }else {
        originalBaud = 9600;
        if (_streamDevice.write(send_frame.rtuFrame(), send_frame.calculateTransmissionTimeMs(originalBaud)*2) != SerialError::SUCCESS) {
            return IStreamDevice::InvalidBaudrate;
        }

        if (_streamDevice.read(receive_frame.rtuBuffer(), getResponseTimeout(send_frame, originalBaud)) != SerialError::SUCCESS) {
            return IStreamDevice::InvalidBaudrate;
        }

        if (receive_frame.validateRTU() == TagModbus::Frame::ValidationStatus::OK) {
            working_baud = baudrates.empty() ? 1 : baudrates[0];
        }
    }
    if (working_baud) {
        devicesBaudratesMap.emplace(slave_ID,working_baud);
    }else {
        devicesBaudratesMap.erase(slave_ID);
    }

    return working_baud;
}


std::map<uint8_t, uint32_t> TagModbus::MasterBase::scanForDevices(const std::span<const uint32_t> baudrates,const uint16_t timeoutMs) {
    constexpr int MODBUS_MIN_ADDRESS = 1;
    constexpr int MODBUS_MAX_ADDRESS = 247;

    for (int slave_id = MODBUS_MIN_ADDRESS; slave_id <= MODBUS_MAX_ADDRESS; ++slave_id) {
        int baud = detectBaud(slave_id, baudrates);
        if (baud != 0) {
            devicesBaudratesMap[slave_id] = baud;
        }
    }

    return devicesBaudratesMap;
}

TagModbus::Frame::FunctionCode TagModbus::MasterBase::getFunctionCode(const bool isRead, const RegisterType register_type) {
    switch(register_type){
        case RegisterType::Coil:
            return isRead?
                       TagModbus::Frame::FunctionCode::ReadCoils
                       :TagModbus::Frame::FunctionCode::WriteMultipleCoils;
        case RegisterType::DiscreteInput:
            return isRead
                       ?TagModbus::Frame::FunctionCode::ReadDiscreteInputs
                       :throw std::invalid_argument("Unable to write to Discrete Inputs");
        case RegisterType::AnalogInput:
            return isRead?
                       TagModbus::Frame::FunctionCode::ReadInputRegisters
                       :throw std::invalid_argument("Unable to write to Input Registers");;
        case RegisterType::Holding:
            return isRead?
                       TagModbus::Frame::FunctionCode::ReadHoldingRegisters
                       :TagModbus::Frame::FunctionCode::WriteMultipleRegisters;
        default:
            throw std::invalid_argument("Unknown Register Type");
            //				return ModbusFrame::FunctionCode::Invalid;
    }

}
