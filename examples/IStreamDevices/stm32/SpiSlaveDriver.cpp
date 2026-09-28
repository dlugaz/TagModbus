//
// Created by kdluzynski on 12.08.2025.
//
#include <algorithm>
#include <cassert>
#include <cstring>

#include "cmsis_os.h"
#include "SPISlaveDriver.hpp"

void SPISlaveDriver::setIRQPin(bool send) {
    HAL_GPIO_WritePin(_IRQ_Port, _IRQ_Pin, send ? GPIO_PinState::GPIO_PIN_RESET : GPIO_PinState::GPIO_PIN_SET);
}

inline SerialError toSerialError(HAL_StatusTypeDef hal_status) {
    switch (hal_status) {
        case HAL_OK:
            return SerialError::SUCCESS;
        case HAL_ERROR:
            return SerialError::INTERNAL_ERROR;
        case HAL_BUSY:
            return SerialError::BUSY;
        case HAL_TIMEOUT:
            return SerialError::TIMEOUT;
        default:
            // Handle any other unexpected HAL status values
            return SerialError::UNKNOWN_ERROR;
    }
}

int SPISlaveDriver::isSPIBusy() const {
    return HAL_SPI_GetState(&_hspi) & (HAL_SPI_STATE_BUSY | HAL_SPI_STATE_BUSY_RX | HAL_SPI_STATE_BUSY_TX |
                                       HAL_SPI_STATE_BUSY_TX_RX);
}


uint32_t SPISlaveDriver::getRemainingTimeoutTicks(const uint32_t timeout_ms, const uint32_t startTick) {
    auto elapsedTicks = (HAL_GetTick() - startTick);
    auto timeout_ticks = pdMS_TO_TICKS(timeout_ms);
    uint32_t remainingTimeoutTicks = timeout_ticks > elapsedTicks ? timeout_ticks - elapsedTicks : 0;
    return remainingTimeoutTicks;
}

SerialError SPISlaveDriver::write(std::span<const uint8_t> buffer, uint32_t timeout_ms,
                                  size_t *bytes_written_out) {
    const uint32_t startTick = HAL_GetTick();
    if (bytes_written_out) *bytes_written_out = 0;

    if (!spiBusy.take(pdMS_TO_TICKS(timeout_ms))) {
        return SerialError::BUSY; // another op is in progress and didn't free up in time
    }

    SerialError result = SerialError::SUCCESS;
    const auto tx_size = std::min(tx_temporary_buffer.size(),
                                  buffer.size());
    std::copy_n(buffer.begin(),
                tx_size,
                tx_temporary_buffer.begin());
    (void) txDoneSemaphore.take(0);
    //disable data transmission to avoid garbage bytes
    enableDataTransmission(false);
    //prepare dma transmission
    if (isSPIBusy()) {
        HAL_SPI_Abort(&_hspi);
    }
    HAL_StatusTypeDef HAL_Result = HAL_SPI_Transmit_DMA(&_hspi, tx_temporary_buffer.data(), tx_size);
    if (HAL_Result == HAL_OK) {
        tx_ready = true;
    } else {
        tx_ready = false;
        ++hal_error_count;
        (void) spiBusy.give();
        return SerialError::INTERNAL_ERROR;
    }
    assert(isSPIBusy());
    //notify master that tx is ready
    setIRQPin(true);

    //if there is timeout_ms then wait for the end
    if (timeout_ms > 0) {
        //wait for completion
        while (tx_ready) {
            if (!txDoneSemaphore.take(getRemainingTimeoutTicks(timeout_ms,startTick))) {
                result = SerialError::TIMEOUT;
                setIRQPin(false);
                tx_ready = false;
                HAL_SPI_Abort(&_hspi);
            }
        }
    }
    (void) spiBusy.give();
    return result;
}

SerialError SPISlaveDriver::read(std::span<uint8_t> buffer, uint32_t timeout_ms, size_t *bytes_read_out) {
    uint32_t startTick = HAL_GetTick();
    if (bytes_read_out != nullptr) {
        *bytes_read_out = 0;
    }
    if (!spiBusy.take(pdMS_TO_TICKS(timeout_ms))) {
        return SerialError::BUSY;
    }
    if (timeout_ms > 0) {
        //wait for completion and data in buffer
        if (!rxDoneSemaphore.take(getRemainingTimeoutTicks(timeout_ms,startTick))) {
            rx_ready = false;
            HAL_SPI_Abort(&_hspi);
            (void) spiBusy.give();
            return SerialError::TIMEOUT;
        }
    }

    SerialError result = SerialError::SUCCESS;
    //there is data in the buffer. Copy it
    if (rx_bytes_in_buffer > 0) {
        std::span rx_buffer{
            rx_temporary_buffer.begin() ,
            rx_temporary_buffer.begin() + rx_bytes_in_buffer
        };
        if (bytes_read_out != nullptr) {
            *bytes_read_out = rx_bytes_in_buffer;
        }

        std::copy_n(rx_buffer.begin(),
                    std::min(rx_buffer.size(),
                             buffer.size()),
                    buffer.begin());
        rx_bytes_in_buffer = 0;
    }
    (void) spiBusy.give();

    return result;
}

SerialError SPISlaveDriver::flush() {
    return SerialError::SUCCESS;
}


SPISlaveDriver::SPISlaveDriver(SPI_HandleTypeDef &spi_handle, GPIO_TypeDef *CS_Port, uint16_t CS_pin,
                               GPIO_TypeDef *IRQ_Port, uint16_t IRQ_pin) : _hspi(spi_handle),
                                                                           _CS_Port(CS_Port),
                                                                           _IRQ_Port(IRQ_Port),
                                                                           _CS_pin(CS_pin),
                                                                           _IRQ_Pin(IRQ_pin) {
    setIRQPin(false);
    configASSERT(spiBusy.give());
    armRxTransmission();
    chipSelectToInstanceMap[pinToIndex(_CS_pin)] = this;
}


SPISlaveDriver::~SPISlaveDriver() {
    chipSelectToInstanceMap[pinToIndex(_CS_pin)] = nullptr;
    (void) txDoneSemaphore.give();
    (void) rxDoneSemaphore.give();
    (void) spiBusy.give();
}

void SPISlaveDriver::armRxTransmission() {
    const size_t safe_offset = std::min(rx_bytes_in_buffer.load(), std::size(rx_temporary_buffer));
    std::span rx_offset_buffer{rx_temporary_buffer.begin()+safe_offset,rx_temporary_buffer.end()};
    HAL_StatusTypeDef result = HAL_SPI_Receive_DMA(&_hspi, rx_offset_buffer.data(), rx_offset_buffer.size());
    if (result == HAL_OK) {
        rx_ready = true;
    } else {
        rx_ready = false;
        ++hal_error_count;
    }
}

void SPISlaveDriver::EXTI_callback() {
    GPIO_PinState cs_state = HAL_GPIO_ReadPin(_CS_Port, _CS_pin);
    bool higherPriorityTaskWoken = false;
    if (cs_state == GPIO_PIN_RESET) {
        if (tx_ready || rx_ready) {
            rx_bytes_in_buffer = 0;
            enableDataTransmission(true);
            setIRQPin(false);
        } else {
            ++transaction_not_ready_error_cnt;
        }
    } else {
        if (tx_ready) {
            //end tx transmission
            enableDataTransmission(false);
            tx_ready = false;
            if (isSPIBusy()) {
                HAL_SPI_Abort(&_hspi);
            }
            //prepare rx transmission
            armRxTransmission();
            onTxComplete();
            txDoneSemaphore.giveFromISR(higherPriorityTaskWoken);
        } else if (rx_ready) {
            //end rx transmission
            enableDataTransmission(false);
            if (isSPIBusy()) {
                HAL_SPI_Abort(&_hspi);
            }
            rx_ready = false;
            uint16_t remaining = __HAL_DMA_GET_COUNTER(_hspi.hdmarx);
            size_t received_bytes = (remaining <= _hspi.RxXferSize) ? (_hspi.RxXferSize - remaining) : 0;
            rx_bytes_in_buffer += received_bytes;
            onRxComplete(received_bytes);
            armRxTransmission();
            rxDoneSemaphore.giveFromISR(higherPriorityTaskWoken);
        }
    }
    portYIELD_FROM_ISR(higherPriorityTaskWoken);
}

void SPISlaveDriver::EXTI_helper(const uint16_t GPIO_Pin) {
    const auto driver = chipSelectToInstanceMap[pinToIndex(GPIO_Pin)];
    if (driver != nullptr) {
        driver->EXTI_callback();
    }
}


void SPISlaveDriver::onTxComplete() {
    if (txCompleteCallback) {
        txCompleteCallback();
    }
}


void SPISlaveDriver::onRxComplete(uint16_t size) {
    if (rxCompleteCallback) {
        std::span rx_buffer(rx_temporary_buffer.begin(),
                            std::min(static_cast<size_t>(size), rx_temporary_buffer.size()));
        rxCompleteCallback(rx_buffer);
    }
}


bool SPISlaveDriver::timer_is_elapsed(uint32_t start, uint16_t time_ms) {
    return (HAL_GetTick() - start) > pdMS_TO_TICKS(time_ms);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    SPISlaveDriver::EXTI_helper(GPIO_Pin);
}


std::array<SPISlaveDriver *, 16> SPISlaveDriver::chipSelectToInstanceMap;
