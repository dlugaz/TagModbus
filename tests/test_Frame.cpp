#include <gtest/gtest.h>
#include "ModbusFrame.hpp"

using namespace TagModbus;

class FrameTest : public ::testing::Test {
protected:
    // Helper to verify byte arrays
    void expectBytesEqual(const std::span<const uint8_t>& actual, const std::vector<uint8_t>& expected) {
        ASSERT_GE(actual.size(), expected.size());
        for (size_t i = 0; i < expected.size(); ++i) {
            EXPECT_EQ(actual[i], expected[i]) << "Mismatch at index " << i;
        }
    }
};

// 1. Test Static Build (Request)
TEST_F(FrameTest, StaticBuildRequest) {
    uint8_t slave = 0x01;
    auto fc = FrameView::ReadHoldingRegisters;
    uint16_t start = 0x006B; // 107
    uint16_t count = 0x0001;
    uint16_t transId = 0x1234;

    Frame frame = Frame::build(true, slave, fc, start, count, {}, transId);

    EXPECT_TRUE(frame.isRequest());
    EXPECT_TRUE(frame.isTCPFrame());
    EXPECT_EQ(frame.transactionID(), transId);
    EXPECT_EQ(frame.slaveID(), slave);
    EXPECT_EQ(frame.functionCode(), fc);
    EXPECT_EQ(frame.startAddress(), start);
    EXPECT_EQ(frame.registerCount(), count);
}

// 2. Test Raw TCP Data Loading
TEST_F(FrameTest, FromRawTcpData) {
    // Typical Read Holding Registers TCP Request
    // [TransID: 2][ProtID: 2][Len: 2][Unit: 1][FC: 1][Addr: 2][Count: 2]
    std::vector<uint8_t> rawData = {
        0x00, 0x01, 0x00, 0x00, 0x00, 0x06, 0x11, 0x03, 0x00, 0x6B, 0x00, 0x01
    };

    Frame frame = Frame::fromRawTcpData(rawData, true);

    EXPECT_EQ(frame.transactionID(), 0x0001);
    EXPECT_EQ(frame.MBAPLength(), 6);
    EXPECT_EQ(frame.slaveID(), 0x11);
    EXPECT_EQ(frame.functionCode(), FrameView::ReadHoldingRegisters);
}

// 3. Test Raw RTU Data Loading
TEST_F(FrameTest, FromRawRtuData) {
    // RTU Data: [Unit: 1][FC: 1][Addr: 2][Count: 2][CRC: 2]
    // Note: Frame constructor sets isTCP=true, so it adds MBAP header space.
    std::vector<uint8_t> rtuData = { 0x01, 0x03, 0x00, 0x00, 0x00, 0x0A, 0xC5, 0xCD };

    Frame frame = Frame::fromRawRtuData(rtuData, true);

    EXPECT_EQ(frame.slaveID(), 0x01);
    EXPECT_EQ(frame.functionCode(), FrameView::ReadHoldingRegisters);
    EXPECT_EQ(frame.registerCount(), 10);
    // Since isTCP=true, MBAP Length should be updated to match RTU payload (minus CRC)
    EXPECT_EQ(frame.MBAPLength(), 6); 
}

// 4. Test Exception Response
TEST_F(FrameTest, BuildExceptionResponse) {
    uint8_t slave = 0x01;
    auto fc = FrameView::ReadHoldingRegisters;
    auto exc = FrameView::IllegalDataAddress;
    uint16_t transId = 0xAAAA;

    Frame frame = Frame::buildExceptionResponse(slave, fc, exc, transId);

    EXPECT_FALSE(frame.isRequest());
    EXPECT_TRUE(frame.isException());
    EXPECT_EQ(frame.exceptionCode(), exc);
    EXPECT_EQ(frame.transactionID(), transId);
}

// 5. Test Memory Isolation
TEST_F(FrameTest, InternalBufferIsolation) {
    std::vector<uint8_t> data = { 0xAA, 0xBB, 0xCC };
    
    Frame* frame1 = new Frame(true);
    frame1->setRawTcpData(data, true);
    
    // Frame holds its own buffer, so deleting source or moving shouldn't corrupt it
    std::vector<uint8_t> bufferCopy(frame1->buffer().begin(), frame1->buffer().begin() + 3);
    EXPECT_EQ(bufferCopy[0], 0xAA);
    
    delete frame1;
}