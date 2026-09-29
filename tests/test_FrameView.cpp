#include <gtest/gtest.h>
#include <array>
#include "ModbusFrameView.hpp"

using namespace TagModbus;

class FrameViewTest : public ::testing::Test {
protected:
    std::array<uint8_t, 256> buffer{};
    
    // Helper to create a basic RTU Request
    void setupBasicRTURequest(uint8_t slave, FrameView::FunctionCode fc, uint16_t addr) {
        buffer.fill(0);
        FrameView view(buffer, true, false);
        view.slaveID(slave);
        view.functionCode(fc);
        view.startAddress(addr);
    }
};

// 1. Test Static CRC Calculation
TEST_F(FrameViewTest, CalculateStaticCRC) {
    // Standard Modbus test vector: [0x01, 0x03, 0x00, 0x00, 0x00, 0x02] -> CRC: 0xC40B
    std::vector<uint8_t> data = {0x01, 0x03, 0x00, 0x00, 0x00, 0x02};
    uint16_t expected = 0x0BC4;
    EXPECT_EQ(FrameView::calculateModbusCRC(data), expected);
}

// 2. Test RTU Field Accessors
TEST_F(FrameViewTest, RTUFieldAccessors) {
    FrameView view(buffer, true, false);
    
    view.slaveID(0x05);
    view.functionCode(FrameView::ReadHoldingRegisters);
    view.startAddress(0x1234);
    view.registerCount(10);

    EXPECT_EQ(view.slaveID(), 0x05);
    EXPECT_EQ(view.functionCode(), FrameView::ReadHoldingRegisters);
    EXPECT_EQ(view.startAddress(), 0x1234);
    EXPECT_EQ(view.registerCount(), 10);
}

// 3. Test TCP specific logic and Exception throwing
TEST_F(FrameViewTest, TCPLogicAndExceptions) {
    // Initialize as RTU
    FrameView rtuView(buffer, true, false);
    
    // Accessing TCP fields on RTU frame should throw logic_error
    EXPECT_THROW(rtuView.transactionID(), std::logic_error);
    
    // Initialize as TCP
    FrameView tcpView(buffer, true, true);
    tcpView.transactionID(0xABCD);
    tcpView.protocolID(0);
    tcpView.MBAPLength(6);

    EXPECT_EQ(tcpView.transactionID(), 0xABCD);
    EXPECT_EQ(tcpView.protocolID(), 0);
    EXPECT_EQ(tcpView.MBAPLength(), 6);
    EXPECT_TRUE(tcpView.isTCPFrame());
}

// 4. Test Modbus Exceptions
TEST_F(FrameViewTest, ModbusExceptionHandling) {
    FrameView view(buffer, false, false);
    view.functionCode(FrameView::ReadCoils);
    
    view.isException(true);
    view.exceptionCode(FrameView::IllegalDataAddress);

    EXPECT_TRUE(view.isException());
    EXPECT_EQ(view.exceptionCode(), FrameView::IllegalDataAddress);
    // Function code should have bit 7 set (0x01 | 0x80 = 0x81)
    EXPECT_EQ(buffer[1], 0x81); 
}

// 5. Test Register Values (Byte Swapping)
TEST_F(FrameViewTest, RegisterValueConversion) {
    // Setup a response for Read Holding Registers (FC 03)
    // RTU Index: 0:Slave, 1:FC, 2:ByteCount, 3-4:Data
    buffer[0] = 0x01;
    buffer[1] = 0x03;
    buffer[2] = 0x02; // 2 bytes
    buffer[3] = 0x12; // High byte
    buffer[4] = 0x34; // Low byte

    FrameView view(buffer, false, false);
    auto vals = view.registersValues();
    
    ASSERT_EQ(vals.size(), 1);
    // Modbus is Big-Endian on wire. 0x1234 in buffer -> 0x1234 uint16
    EXPECT_EQ(vals[0], 0x1234);
}

// 6. Test Validation
TEST_F(FrameViewTest, ValidationRTU) {
    // Valid packet
    buffer = {0x01, 0x03, 0x00, 0x00, 0x00, 0x02, 0xC4, 0x0B};
    FrameView view(buffer, true, false);
    
    EXPECT_EQ(view.validateRTU(), FrameView::ValidationStatus::OK);

    // Corrupt CRC
    buffer[7] = 0x00;
    EXPECT_EQ(view.validateRTU(), FrameView::ValidationStatus::InvalidCRC);
}

// 7. Test Transmission Time Calculation
TEST_F(FrameViewTest, TransmissionTime) {
    // 10 bytes at 9600 bps
    // Result = ((10 bits * 1000ms * 10 bytes * 10) / 9600 + 5) / 10
    // Result approx 10.4ms -> 10ms
    int time = FrameView::calculateTransmissionTimeMs(10, 9600);
    EXPECT_NEAR(time, 10, 1);
}