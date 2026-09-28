/*
 * SPIDriver.hpp
 *
 *  Created on: Jul 16, 2025
 *      Author: kdluzynski
 */

#ifndef INC_SPISLAVEDRIVER_HPP_
#define INC_SPISLAVEDRIVER_HPP_
#include <IStreamDevice.hpp>
#include "main.h"
#include "FreeRTOS.h"
#include <unordered_map>
#include <vector>
#include <atomic>

#include <FreeRTOS/Semaphore.hpp>

#include "stm32f407xx.h"


class SPISlaveDriver : public IStreamDevice {
    SPI_HandleTypeDef &_hspi;
    GPIO_TypeDef *_CS_Port;
    GPIO_TypeDef *_IRQ_Port;
    uint16_t _CS_pin;
    uint16_t _IRQ_Pin;

    std::array<uint8_t, 1024> rx_temporary_buffer{0};
    std::array<uint8_t, 300> tx_temporary_buffer{0};
    std::atomic<std::size_t> rx_bytes_in_buffer{0};
    int hal_error_count = 0;
    std::atomic_bool tx_ready = false;
    std::atomic_bool rx_ready = false;
    std::atomic_int transaction_not_ready_error_cnt = 0;

    FreeRTOS::BinarySemaphore spiBusy;
    FreeRTOS::BinarySemaphore txDoneSemaphore;
    FreeRTOS::BinarySemaphore rxDoneSemaphore;


    void setIRQPin(bool send);

    static std::array<SPISlaveDriver *, 16> chipSelectToInstanceMap;

    SPISlaveDriver(const SPISlaveDriver &) = delete;

    SPISlaveDriver operator=(const SPISlaveDriver &) = delete;

public:
    int isSPIBusy() const;

    SerialError write(std::span<const uint8_t> buffer, uint32_t timeout_ms,
                      size_t *bytes_written_out = nullptr) override;

    SerialError flush() override;

    SerialError read(std::span<uint8_t> buffer,
                     uint32_t timeout_ms, size_t *bytes_read_out = nullptr) override;

    explicit SPISlaveDriver(SPI_HandleTypeDef &spi_handle, GPIO_TypeDef *CS_Port, uint16_t CS_pin,
                            GPIO_TypeDef *IRQ_Port, uint16_t IRQ_pin);

    ~SPISlaveDriver() override;

    void armRxTransmission();

    void EXTI_callback();

    static void EXTI_helper(uint16_t GPIO_Pin);

private:
    void onTxComplete() override;

    void onRxComplete(uint16_t size) override;

    static bool timer_is_elapsed(uint32_t start, uint16_t time_ms);

    void enableDataTransmission(const bool value) const {
        if (value) {
            _hspi.Instance->CR1 &= ~SPI_CR1_SSI;
        } else {
            _hspi.Instance->CR1 |= SPI_CR1_SSI;
        }
    }

    static int pinToIndex(uint16_t gpio_pin) {
        assert(gpio_pin !=0);
        return std::countr_zero(gpio_pin);
    }
    static uint32_t getRemainingTimeoutTicks(uint32_t timeout_ms, uint32_t startTick);


};


#endif /* INC_SPISLAVEDRIVER_HPP_ */
