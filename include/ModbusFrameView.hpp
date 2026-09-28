//
// Created by kdluzynski on 17.11.2025.
//

#ifndef MODBUSFRAMEVIEW_HPP
#define MODBUSFRAMEVIEW_HPP
#include "ModbusUtils.hpp"

namespace TagModbus {
class FrameView {
    protected:
        std::span<uint8_t> _externalBufferSpan;

        bool _isRequest = false;
        bool _dataBufferTCP = false;

        virtual std::span<uint8_t> _dataBuffer() {
            return std::span (_externalBufferSpan);
        }

        virtual std::span<const uint8_t> _dataBuffer() const {
            return std::span<const uint8_t> (_externalBufferSpan);
        }
        enum FRAME_POS_TCP {
            TRANSACTION_ID = 0,
            PROTOCOL_ID = 2,
            LENGTH = 4,
        };
    enum FRAME_POS_RTU {
        UNIT_ID = 0,
        FUNCTION_CODE = 1,
        DATA = 2,
        EXCEPTION_CODE = DATA,
        START_ADDRESS = DATA,
        BYTE_COUNT = DATA,
        REGISTER_DATA = BYTE_COUNT + 1,
        REGISTER_COUNT = START_ADDRESS + 2,
        BYTE_COUNT_MULTIPLE_REGISTERS = REGISTER_COUNT + 2,
        REGISTER_DATA_WRITE_SINGLE = START_ADDRESS + 2,
        REGISTER_DATA_WRITE_MULTIPLE = BYTE_COUNT_MULTIPLE_REGISTERS + 1,
    };

        static uint16_t betole(const uint8_t *bigendiandata) {
            return static_cast<uint16_t>(bigendiandata[0] << 8) | (bigendiandata[1] & 0xFF);
        }

        static void letobe(const uint16_t littleendiandata, uint8_t *data) {
            data[0] = static_cast<uint8_t>((littleendiandata >> 8) & 0xFF);
            data[1] = static_cast<uint8_t>((littleendiandata) & 0xFF);
        }

        uint16_t RTULengthWithoutCRC() const {
            const int result = calculateRTULength() - CRC_SIZE;
            return (result >= 0) ? static_cast<uint16_t>(result) : 0;
        }

    public:
        virtual ~FrameView() = default;

        enum FunctionCode {
            ReadCoils = 0x01,
            ReadDiscreteInputs = 0x02,
            ReadHoldingRegisters = 0x03,
            ReadInputRegisters = 0x04,
            WriteSingleCoil = 0x05,
            WriteSingleRegister = 0x06,
            WriteMultipleCoils = 0x0F,
            WriteMultipleRegisters = 0x10,
            Diagnostics = 0x08,
            MaskWriteRegister = 0x16,
            ReadWriteMultipleRegisters = 0x17,
            ReadDeviceIdentification = 0x0E,
            Invalid = 0
        };

        bool isTCPFrame() const {
            return _dataBufferTCP;
        }


        static uint16_t calculateModbusCRC(const std::span<const uint8_t> data) {
            static constexpr uint16_t table[256] = {
                0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
                0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
                0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
                0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
                0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
                0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
                0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
                0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
                0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
                0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
                0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
                0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
                0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
                0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
                0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
                0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
                0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
                0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
                0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
                0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
                0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
                0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
                0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
                0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
                0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
                0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
                0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
                0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
                0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
                0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
                0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
                0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040
            };

            uint8_t xor_ = 0;
            uint16_t crc = 0xFFFF;
            for (const uint8_t byte: data) {
                xor_ = byte ^ crc;
                crc >>= 8;
                crc ^= table[xor_];
            }
            return crc;
        }

        uint16_t calculateModbusCRC() const {
            return calculateModbusCRC(rtuBuffer().subspan(0, RTULengthWithoutCRC()));
        }

        void appendCRC() {
            crc(calculateModbusCRC());
        }

        uint16_t crcPosition() const {
            return isTCPFrame()?RTU_HEADER_START_POSITION:0 + RTULengthWithoutCRC();
        }

        uint16_t crc() const {
            const uint16_t crcPos = RTULengthWithoutCRC();
            const uint16_t crcVal = rtuBuffer()[crcPos] | (rtuBuffer()[crcPos + 1] << 8);
            return crcVal;
        }

        void crc(const uint16_t value) {
            const uint16_t crcPos = RTULengthWithoutCRC();
            rtuBuffer()[crcPos] = value & 0xFF;
            rtuBuffer()[crcPos + 1] = (value >> 8) & 0xFF;
        }


        std::span<uint8_t> buffer()  {
            return _dataBuffer();
        }


        std::span<const uint8_t> rtuBuffer() const {
	        if (_dataBufferTCP) {
		        return _dataBuffer().subspan(RTU_HEADER_START_POSITION);
	        }
	        return _dataBuffer();
        }

        std::span<uint8_t> rtuBuffer() {
	        auto const_span = static_cast<const FrameView *>(this)->rtuBuffer();
	        return {const_cast<uint8_t *>(const_span.data()), const_span.size()};
        }

        explicit FrameView(const std::span<uint8_t> buffer, const bool request_type, const bool isTCP)
	        : _externalBufferSpan(buffer), _isRequest(request_type), _dataBufferTCP(isTCP) {}


        bool isRequest() const {
            return _isRequest;
        }

        FrameView &isRequest(const bool is_request) {
            _isRequest = is_request;
            return *this;
        }


        void throwIfNotTCP() const {
            if (!_dataBufferTCP)
                throw std::logic_error("Field not available in RTU frame");
        }

        uint16_t transactionID() const {
            throwIfNotTCP();
            return betole(&_dataBuffer()[FRAME_POS_TCP::TRANSACTION_ID]);
        }

        FrameView &transactionID(const uint16_t value) {
            throwIfNotTCP();
            letobe(value, &_dataBuffer()[FRAME_POS_TCP::TRANSACTION_ID]);
            return *this;
        }

        uint16_t protocolID() const {
            throwIfNotTCP();
            return betole(&_dataBuffer()[FRAME_POS_TCP::PROTOCOL_ID]);
        }

        FrameView &protocolID(const uint16_t value) {
            throwIfNotTCP();
            letobe(value, &_dataBuffer()[FRAME_POS_TCP::PROTOCOL_ID]);
            return *this;
        }

        uint16_t MBAPLength() const {
            throwIfNotTCP();
            return betole(&_dataBuffer()[FRAME_POS_TCP::LENGTH]);
        }

        FrameView &MBAPLength(const uint16_t value) {
            throwIfNotTCP();
            letobe(value, &_dataBuffer()[FRAME_POS_TCP::LENGTH]);
            return *this;
        }

        uint16_t RTULength() const {
        	return calculateRTULength();
        }

        uint16_t pduLength() const {
        	return RTULengthWithoutCRC() - UNIT_ID_SIZE;
        }

        FrameView &slaveID(const uint8_t value) {
            rtuBuffer()[FRAME_POS_RTU::UNIT_ID] = value;
            return *this;
        }

        uint8_t slaveID() const{
            return rtuBuffer()[FRAME_POS_RTU::UNIT_ID];
        }

        FunctionCode functionCode() const{
            return static_cast<FunctionCode>(rtuBuffer()[FRAME_POS_RTU::FUNCTION_CODE] & 0x7F);
        }

        FrameView &functionCode(const FunctionCode value) {
            rtuBuffer()[FRAME_POS_RTU::FUNCTION_CODE] = static_cast<uint8_t>(value);
            return *this;
        }

        bool hasStartAddress() const {
            if (isException())
                return false;
            switch (functionCode()) {
                case ReadCoils:
                case ReadDiscreteInputs:
                case ReadHoldingRegisters:
                case ReadInputRegisters:
                    return _isRequest;
                case WriteSingleCoil:
                case WriteSingleRegister:
                case WriteMultipleCoils:
                case WriteMultipleRegisters:
                    return true;
                default:
                    return false;
            }
        }

        uint16_t startAddress() const{
            if (!hasStartAddress())
                return 0;
            return betole(&rtuBuffer()[START_ADDRESS]);
        }

        FrameView &startAddress(const uint16_t value) {
            if (hasStartAddress())
                letobe(value, &rtuBuffer()[START_ADDRESS]);
            return *this;
        }

        uint16_t byteCount() const  {
            if (isException())
                return 0;
            switch (functionCode()) {
                case ReadCoils:
                case ReadDiscreteInputs:
                case ReadHoldingRegisters:
                case ReadInputRegisters:
                    return _isRequest ? 0 : rtuBuffer()[FRAME_POS_RTU::BYTE_COUNT];
                case WriteMultipleCoils:
                case WriteMultipleRegisters:
                    return _isRequest ? rtuBuffer()[FRAME_POS_RTU::BYTE_COUNT_MULTIPLE_REGISTERS] : 0;
                case WriteSingleCoil:
                case WriteSingleRegister:
                    return 2;
                default:
                    return 0;
            }
        }

        FrameView &byteCount(const uint8_t value) {
            if (!isException()) {
                switch (functionCode()) {
                    case ReadCoils:
                    case ReadDiscreteInputs:
                    case ReadHoldingRegisters:
                    case ReadInputRegisters:
                        if (!_isRequest)
                            rtuBuffer()[FRAME_POS_RTU::BYTE_COUNT] = value;
                        break;
                    case WriteMultipleCoils:
                    case WriteMultipleRegisters:
                        if (_isRequest)
                            rtuBuffer()[FRAME_POS_RTU::BYTE_COUNT_MULTIPLE_REGISTERS] = value;
                        break;
                    default:
                    case WriteSingleCoil:
                    case WriteSingleRegister:
                        break;
                }
            }
            return *this;
        }

        uint16_t registerCount() const {
            if (isException())
                return 0;
            switch (functionCode()) {
                case ReadCoils:
                case ReadDiscreteInputs:
                    return _isRequest ? betole(&rtuBuffer()[FRAME_POS_RTU::REGISTER_COUNT]) : (byteCount() * 8);
                case ReadHoldingRegisters:
                case ReadInputRegisters:
                    return _isRequest ? betole(&rtuBuffer()[FRAME_POS_RTU::REGISTER_COUNT]) : (byteCount() / 2);
                case WriteSingleCoil:
                case WriteSingleRegister:
                    return 1;
                case WriteMultipleCoils:
                case WriteMultipleRegisters:
                    return betole(&rtuBuffer()[FRAME_POS_RTU::REGISTER_COUNT]);
                default:
                    return 0;
            }
        }

        FrameView &registerCount(const uint16_t value) {
            if (!isException()) {
                switch (functionCode()) {
                    case ReadCoils:
                    case ReadDiscreteInputs:
                    case ReadHoldingRegisters:
                    case ReadInputRegisters:
                        if (_isRequest)
                            letobe(value, &rtuBuffer()[FRAME_POS_RTU::REGISTER_COUNT]);
                        break;
                    case WriteMultipleCoils:
                    case WriteMultipleRegisters:
                        letobe(value, &rtuBuffer()[FRAME_POS_RTU::REGISTER_COUNT]);
                        break;
                    case WriteSingleCoil:
                    case WriteSingleRegister:
                    default:
                        break;
                }
            }
            return *this;
        }

        enum ExceptionCode {
            IllegalFunction = 0x01,
            IllegalDataAddress = 0x02,
            IllegalDataValue = 0x03,
            SlaveDeviceFailure = 0x04,
            Acknowledge = 0x05,
            SlaveDeviceBusy = 0x06,
            NegativeAcknowledge = 0x07,
            MemoryParityError = 0x08,
        };

        bool isException() const{
            return (rtuBuffer()[FRAME_POS_RTU::FUNCTION_CODE] & 0x80) != 0;
        }

        FrameView &isException(const bool setFlag) {
            if (setFlag) {
                _isRequest = false;
                rtuBuffer()[FUNCTION_CODE] |= 0x80;
            } else
                rtuBuffer()[FUNCTION_CODE] &= ~0x80;

            return *this;
        }

        ExceptionCode exceptionCode() {
            return static_cast<ExceptionCode>(isException() ? rtuBuffer()[EXCEPTION_CODE] : 0);
        }

        FrameView &exceptionCode(const ExceptionCode exception_code) {
            rtuBuffer()[EXCEPTION_CODE] = exception_code;
            return *this;
        }

        enum class ValidationStatus {
            OK,
            ProtocolIdentifier,
            MBAPHeaderLengthInvalid,
            InvalidCRC,
            TransactionID,
            InvalidFunctionCode,
            Unknown,
            RTUBufferTooShort,
            SlaveMismatch,
            StartAddressMismatch,
            RegisterCountMismatch,
            FunctionCodeMismatch,
        };

        ValidationStatus validateTCP() const {
            if (protocolID() != 0)
                return ValidationStatus::ProtocolIdentifier;

            if (MBAPLength() == 0)
                return ValidationStatus::MBAPHeaderLengthInvalid;

            return validateCommon();
        }


        ValidationStatus validateCommon() const{
            if (rtuBuffer().size() < MINIMAL_RTU_SIZE || rtuBuffer().size()<calculateRTULength())
                return ValidationStatus::RTUBufferTooShort;
            const uint8_t function_code = static_cast<uint8_t>(functionCode());
            if (function_code == 0)
                return ValidationStatus::InvalidFunctionCode;
            return ValidationStatus::OK;
        }

        ValidationStatus validateRTU() const{
            const ValidationStatus commonValidation = validateCommon();
            if (commonValidation != ValidationStatus::OK)
                return commonValidation;
            if (crc() != calculateModbusCRC()) {
                return ValidationStatus::InvalidCRC;
            }

            return ValidationStatus::OK;
        }
	ValidationStatus validateResponse(const FrameView& send_frame) const {
        	if (send_frame.slaveID() != slaveID())
        		return ValidationStatus::SlaveMismatch;
        	if (hasStartAddress() && (send_frame.startAddress() != startAddress()))
        		return ValidationStatus::StartAddressMismatch;
        	if (send_frame.registerCount() != registerCount()&&!isException())
        		return ValidationStatus::RegisterCountMismatch;
        	if (send_frame.functionCode() != functionCode())
        		return ValidationStatus::FunctionCodeMismatch;
	        return ValidationStatus::OK;
        }

        FrameView &clear() {
            std::memset(_dataBuffer().data(), 0, _dataBuffer().size());
            _isRequest = false;
            return *this;
        }

        std::span<const uint8_t> rtuFrame() {
            const uint16_t rtuLength = calculateRTULength();
            return rtuBuffer().subspan(0, rtuLength);
        }

        int tcpFrameSize() const {
            return MBAP_HEADER_SIZE + pduLength();
        }

        std::span<const uint8_t> tcpFrame() {
            throwIfNotTCP();
            MBAPLength(RTULengthWithoutCRC());
            return _dataBuffer().subspan(0, tcpFrameSize());
        }

        std::span<const uint8_t> registersData()  const{
            if (!hasRegistersValues())
                return {};
            uint16_t data_pos = 0;
            switch (functionCode()) {
                case ReadCoils:
                case ReadDiscreteInputs:
                case ReadHoldingRegisters:
                case ReadInputRegisters:
                    data_pos = FRAME_POS_RTU::REGISTER_DATA;
                    break;
                case WriteSingleCoil:
                case WriteSingleRegister:
                    data_pos = FRAME_POS_RTU::REGISTER_DATA_WRITE_SINGLE;
                    break;
                case WriteMultipleCoils:
                case WriteMultipleRegisters:
                    data_pos = FRAME_POS_RTU::REGISTER_DATA_WRITE_MULTIPLE;
                    break;
                default:
                    break;
            }
            return rtuBuffer().subspan(data_pos, byteCount());
        }

        std::span<uint8_t> registersData() {
	        auto const_span = static_cast<const FrameView *>(this)->registersData();
	        return {const_cast<uint8_t *>(const_span.data()), const_span.size()};
        }

        static uint16_t swap_bytes(const uint16_t val) {
            return (val << 8) | (val >> 8);
        }

        bool hasRegistersValues() const {
            if (isException())
                return false;
            switch (functionCode()) {
                default:
                    return false;
                case ReadCoils:
                case ReadDiscreteInputs:
                case ReadHoldingRegisters:
                case ReadInputRegisters:
                    return !_isRequest;
                case WriteSingleCoil:
                case WriteSingleRegister:
                    return true;
                case WriteMultipleCoils:
                case WriteMultipleRegisters:
                    return _isRequest;
            }
        }

        std::vector<uint16_t> registersValues() const {
            const std::span<const uint8_t> byte_span = registersData(); // Assuming this is valid for the call's duration
            std::vector<uint16_t> result;
            if (functionCode() == ReadCoils || functionCode() == ReadDiscreteInputs) {
                result.reserve(byte_span.size() * 8); // Reserve for all bits
                for (unsigned int i = 0; i < byte_span.size() * 8; ++i) {
                    // Corrected iota range
                    const size_t bit_index = i % 8;
                    const size_t byte_index = i / 8;
                    const bool bit_value = (byte_span[byte_index] >> bit_index) & 0x1;
                    result.push_back(static_cast<uint16_t>(bit_value ? 0xFF00 : 0));
                }
            } else {
                result.reserve(byte_span.size() / 2);
                for (unsigned int i = 0; i < byte_span.size() / 2; ++i) {
                    const size_t idx1 = i * 2;
                    const size_t idx2 = i * 2 + 1;

                    const uint16_t val = static_cast<uint16_t>(byte_span[idx1]) |
                                   (static_cast<uint16_t>(byte_span[idx2]) << 8); // Assuming little-endian in byte_span

                    result.push_back(swap_bytes(val));
                }
            }
            return result;
        }

        FrameView &registersValues(const std::span<const uint16_t> values) {
            if (hasRegistersValues()) {
                std::span<uint8_t> registers_data = registersData();
                for (uint16_t i = 0; i < registers_data.size(); i += 2) {
                    const size_t pos_values = i / 2;
                    if (pos_values >= values.size())
                        break;
                    registers_data[i] = values[pos_values] >> 8;
                    registers_data[i + 1] = values[pos_values] & 0xFF;
                }
            }
            return *this;
        }
        //
        // FrameView &registersValues(std::span<const uint16_t> newValues, uint16_t registerOffset = 0) {
	       //  if (!hasRegistersValues()) {
		      //   return *this;
	       //  }
        //
	       //  std::span<uint8_t> data = registersData();
	       //  const bool isBitAccess = (functionCode() == ReadCoils || functionCode() == ReadDiscreteInputs);
        //
	       //  if (isBitAccess) {
		      //   // Coil/Discrete Input updates
		      //   for (size_t i = 0; i < newValues.size(); ++i) {
			     //    size_t totalBitIndex = (registerOffset + i);
			     //    size_t byteIndex = totalBitIndex / 8;
			     //    size_t bitIndex = totalBitIndex % 8;
        //
			     //    if (byteIndex >= data.size()) break;
        //
			     //    // Modbus convention: 0xFF00 is ON, 0x0000 is OFF
			     //    if (newValues[i] != 0) {
				    //     data[byteIndex] |= (1 << bitIndex);
			     //    } else {
				    //     data[byteIndex] &= ~(1 << bitIndex);
			     //    }
		      //   }
	       //  } else {
		      //   // Holding/Input Register updates
		      //   const size_t startByte = registerOffset * 2;
		      //   for (size_t i = 0; i < newValues.size(); ++i) {
			     //    const size_t currentByte = startByte + (i * 2);
			     //    if (currentByte + 1 >= data.size()) break;
        //
			     //    uint16_t val = newValues[i];
        //
			     //    // Convert host native to Big-Endian for the wire
			     //    if constexpr (std::endian::native == std::endian::little) {
				    //     val = std::byteswap(val);
			     //    }
        //
			     //    std::memcpy(data.data() + currentByte, &val, sizeof(uint16_t));
		      //   }
	       //  }
	       //  return *this;
        // }
        //
        // auto registersView() {
	       //  std::span<uint8_t> data = registersData();
	       //  bool isBitAccess = (functionCode() == ReadCoils || functionCode() == ReadDiscreteInputs);
        //
	       //  // Determine the number of elements (bits or 16-bit words)
	       //  size_t count = isBitAccess ? (data.size() * 8) : (data.size() / 2);
        //
	       //  return std::views::iota(0ULL, count)
	       //         | std::views::transform([data, isBitAccess](size_t i) -> uint16_t {
		      //          if (isBitAccess) {
			     //           bool bit_value = (data[i / 8] >> (i % 8)) & 0x1;
			     //           return bit_value ? 0xFF00 : 0x0000;
		      //          } else {
			     //           uint16_t val;
			     //           // Offset is i * 2 bytes per register
			     //           std::copy_n(data.data() + (i * 2), 2, reinterpret_cast<uint8_t *>(&val));
        //
			     //           if constexpr (std::endian::native == std::endian::little) {
				    //            return std::byteswap(val);
			     //           }
			     //           return val;
		      //          }
	       //         });
        // }

        uint16_t calculateRTULength() const {
            return calculateRTULength(isException(),_isRequest,functionCode(),byteCount());
        }

        static uint16_t calculateRTULength(const bool isException, const bool isRequest, const FunctionCode functionCode, const uint16_t byteCount) {
            if (isException) {
                return RTU_HEADER_SIZE + EXCEPTION_CODE_SIZE + CRC_SIZE;
            }
            switch (functionCode) {
                case ReadCoils:
                case ReadDiscreteInputs:
                case ReadHoldingRegisters:
                case ReadInputRegisters:
                    if (isRequest)
                        return RTU_HEADER_SIZE + STARTING_ADDRESS_SIZE + REGISTER_COUNT_SIZE + CRC_SIZE;
                    else
                        return RTU_HEADER_SIZE + BYTE_COUNT_SIZE + byteCount + CRC_SIZE;
                case WriteSingleCoil:
                case WriteSingleRegister:
                    return RTU_HEADER_SIZE + STARTING_ADDRESS_SIZE + WRITE_DATA_SIZE + CRC_SIZE;
                case WriteMultipleCoils:
                case WriteMultipleRegisters:
                    if (isRequest)
                        return RTU_HEADER_SIZE + STARTING_ADDRESS_SIZE + REGISTER_COUNT_SIZE + BYTE_COUNT_SIZE + byteCount
                               + CRC_SIZE;
                    else
                        return RTU_HEADER_SIZE + STARTING_ADDRESS_SIZE + REGISTER_COUNT_SIZE + CRC_SIZE;
                default:
                    return 0;
            };
        }

        uint16_t calculateExpectedResponseRTULength() const {
            if (!_isRequest)
                return RTULength();
            return calculateRTULength(false,true,functionCode(),registerCount()*2);
        }
        int calculateResponseTransmissionTimeMs(const int bitsPerSecond) const {
            const int length = calculateExpectedResponseRTULength();
            return calculateTransmissionTimeMs(length,bitsPerSecond);
        }

        static int calculateTransmissionTimeMs(const size_t length, const int bitsPerSecond) {
            constexpr int BITS_PER_BYTE = 10;
            constexpr int INCREASE_PRECISION = 10;
            const int result = ((BITS_PER_BYTE * 1000 * static_cast<ssize_t>(length) * INCREASE_PRECISION / bitsPerSecond) +
                                5) / INCREASE_PRECISION;

            // return (result < 3)?3:result;
            return (result < 0)?0:result;
        }
        int calculateTransmissionTimeMs(const int bitsPerSecond) const {
            return calculateTransmissionTimeMs(calculateRTULength(),bitsPerSecond);
        }
        //TODO assign registersValues split into request and response

        FrameView &rebuild(const bool is_request, const uint8_t slave_ID, const FunctionCode function_code, const uint16_t start_address,
                             const uint16_t register_count, const std::span<const uint16_t> registers_values = {},
                             const uint16_t transaction_ID = 0) {
            isRequest(is_request);
            transactionID(transaction_ID);
            slaveID(slave_ID);
            functionCode(function_code);

            startAddress(start_address);
            registerCount(register_count);
            byteCount(register_count * 2); //merge registerCount and bytecount?
            registersValues(registers_values);

            MBAPLength(RTULengthWithoutCRC());
            appendCRC();
            return *this;
        }

        FrameView &rebuildExceptionResponse(const uint8_t slave_ID, const FunctionCode function_code, const ExceptionCode exception_code,
                                              const uint16_t transaction_ID = 0) {
            transactionID(transaction_ID);
            slaveID(slave_ID);
            functionCode(function_code);
            isException(true);
            exceptionCode(exception_code);

            MBAPLength(RTULengthWithoutCRC());
            appendCRC();
            return *this;
        }

        std::string toString()  {
            return TagModbus::toString(_dataBuffer());
        }

        static constexpr uint8_t MBAP_HEADER_SIZE = 7;
        static constexpr uint8_t RTU_HEADER_START_POSITION = MBAP_HEADER_SIZE - 1;
        static constexpr uint8_t TRANSACTION_ID_SIZE = 2;
        static constexpr uint8_t PROTOCOL_ID_SIZE = 2;
        static constexpr uint8_t TCP_LENGTH_SIZE = 2;
        static constexpr uint8_t UNIT_ID_SIZE = 1;
        static constexpr uint8_t RTU_HEADER_SIZE = 2;
        static constexpr uint8_t BYTE_COUNT_SIZE = 1;
        static constexpr uint8_t STARTING_ADDRESS_SIZE = 2;
        static constexpr uint8_t REGISTER_COUNT_SIZE = 2;
        static constexpr uint8_t WRITE_DATA_SIZE = 2;
        static constexpr uint8_t CRC_SIZE = 2;
        static constexpr uint8_t EXCEPTION_CODE_SIZE = 1;
        static constexpr uint8_t MINIMAL_RTU_SIZE = RTU_HEADER_SIZE + CRC_SIZE;

    };

    static std::string to_string(const FrameView::ValidationStatus status) {
        switch (status) {
            case FrameView::ValidationStatus::OK:return "OK";
            case FrameView::ValidationStatus::InvalidCRC:return "Invalid CRC";
            case FrameView::ValidationStatus::InvalidFunctionCode:return "Invalid Function Code";
            case FrameView::ValidationStatus::ProtocolIdentifier:return "Protocol Identifier";
            case FrameView::ValidationStatus::MBAPHeaderLengthInvalid:return "MBAP Header Length Invalid";
	        case FrameView::ValidationStatus::TransactionID:
        		return "TransactionID Invalid";
	        case FrameView::ValidationStatus::RTUBufferTooShort:
		        return "RTU Buffer Too Short ";
	        case FrameView::ValidationStatus::SlaveMismatch:
		        return "Slave Mismatch ";
	        case FrameView::ValidationStatus::StartAddressMismatch:
		        return "Start Address Mismatch ";
	        case FrameView::ValidationStatus::RegisterCountMismatch:
		        return "Register Count Mismatch ";
	        case FrameView::ValidationStatus::FunctionCodeMismatch:
		        return "Function Code Mismatch ";
        	case FrameView::ValidationStatus::Unknown:
	        default: return "Unknown";
        }

    }
}

#endif //MODBUSFRAMEVIEW_HPP