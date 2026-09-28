/*
 * UartDriver.hpp
 *
 *  Created on: Jul 16, 2025
 *      Author: kdluzynski
 */

#ifndef INC_UARTDRIVER_HPP_
#define INC_UARTDRIVER_HPP_

#include <IStreamDevice.hpp>
#include "main.h"

#include <span>
#include <unordered_map>

#include "FreeRTOS/Queue.hpp"
#include "FreeRTOS/Semaphore.hpp"


class UartDriver : public IStreamDevice{
	UART_HandleTypeDef& _huart;
	std::span<uint8_t> rx_buffer;
	std::span<const uint8_t> tx_buffer;
	size_t rx_temporary_buffer_index;
	std::array<uint8_t,256> rx_temporary_buffer;
	FreeRTOS::BinarySemaphore device_busy;
	volatile size_t valid_rx_bytes;
	static std::unordered_map<UART_HandleTypeDef*, UartDriver*> uartDriverInstances;
	GPIO_TypeDef* _DE_Port;
	uint16_t _DE_pin;
public:

	SerialError flush() override;

	static bool timer_is_elapsed(uint32_t start, uint16_t time_ms);

	void ensureMinimumTimeout(size_t messageLength, uint32_t &timeoutMs) const;

	SerialError write(std::span<const uint8_t> buffer, uint32_t timeoutMs, size_t* bytes_written_out = nullptr) override;

	bool is_transmitting() const;

	SerialError waitUntilReadyToTransmit(uint32_t timeout_ms, uint32_t startTick);

	void deinit_transmission();

	SerialError waitForTransmissionComplete(uint32_t timeout_ms, uint32_t startTick);

	SerialError read(std::span<uint8_t> buffer, uint32_t timeoutMs,
	                 size_t *bytes_read_out = nullptr) override;

	void setDirectionPinWrite(bool cond);

	explicit UartDriver(UART_HandleTypeDef& uart_handle,GPIO_TypeDef* DE_Port, uint16_t DE_pin);

	~UartDriver();

	void baudrate(uint32_t baudrate) override;

	uint32_t baudrate() const override;

	static void TxCpltCallback(UART_HandleTypeDef *huart);

	static void RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);

	static void ErrorCallback(UART_HandleTypeDef *huart);

private:
	void onTxComplete() override;

	void onRxComplete(uint16_t size) override;

	void onErrorCallback();
};

#endif /* INC_UARTDRIVER_HPP_ */
