//
// Created by kdluzynski on 12.08.2025.
//
#include "UartDriver.hpp"
#include <algorithm>
#include <ctime>

#include "application.hpp"
#include "cmsis_os.h"

SerialError UartDriver::flush() {
	while (__HAL_UART_GET_FLAG(&_huart, UART_FLAG_RXNE) == SET) {
		__HAL_UART_FLUSH_DRREGISTER(&_huart); // Read and discard
	}
	return SerialError::SUCCESS;
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


bool UartDriver::timer_is_elapsed(uint32_t start, uint16_t time_ms) {
 	return (HAL_GetTick() - start) > pdMS_TO_TICKS(time_ms);
 }
static int calculateMinimalTimeoutMs(const size_t length, const int bitsPerSecond) {
	constexpr int BITS_PER_BYTE = 10;
	constexpr int INCREASE_PRECISION = 10;
	const int result = ((BITS_PER_BYTE * 1000 * static_cast<ssize_t>(length) * INCREASE_PRECISION / bitsPerSecond) +
						5) / INCREASE_PRECISION;

	return (result < 3)?3:result;
}

void UartDriver::ensureMinimumTimeout(size_t messageLength, uint32_t &timeoutMs) const {
	const auto minTimeoutMs = calculateMinimalTimeoutMs(messageLength,baudrate())*15/10;
	timeoutMs = timeoutMs > minTimeoutMs? timeoutMs: minTimeoutMs;
}

 SerialError UartDriver::write(std::span<const uint8_t> buffer, uint32_t timeoutMs, size_t *bytes_written_out) {
	//send request via uart
	uint32_t start = HAL_GetTick();
	SerialError result = waitUntilReadyToTransmit(timeoutMs,start);
	if (result != SerialError::SUCCESS) {
		return result;
	}

 	setDirectionPinWrite(true);
	tx_buffer = buffer;
 	const HAL_StatusTypeDef transmit_result = HAL_UART_Transmit_DMA(&_huart, tx_buffer.data(),
                                                     tx_buffer.size());
	result = toSerialError(transmit_result);
 	if (timeoutMs>0) {
 		result = waitForTransmissionComplete(timeoutMs,start);
 	}
	return result;
}
bool UartDriver::is_transmitting() const {
 	return  (HAL_UART_GetState(&_huart)&(HAL_UART_STATE_BUSY|HAL_UART_STATE_BUSY_RX|HAL_UART_STATE_BUSY_TX|HAL_UART_STATE_BUSY_TX_RX));
 }
SerialError UartDriver::waitUntilReadyToTransmit(uint32_t timeout_ms, const uint32_t startTick) {
 	while (!device_busy.take(0)) {
 		if (!is_transmitting()) {
 			deinit_transmission();
 			continue;
 		}
 		if (timer_is_elapsed(startTick, timeout_ms)) {
 			return SerialError::READY_TIMEOUT;
 		}
 	}
 	return SerialError::SUCCESS;
 }
void UartDriver::deinit_transmission() {
 	setDirectionPinWrite(false);
 	tx_buffer = std::span<const uint8_t>{};
 	rx_buffer = std::span<uint8_t>{};

 	device_busy.giveFromISR();	//zastanowic sie czy tutaj czy w poszczegolnych
 }
SerialError UartDriver::waitForTransmissionComplete(uint32_t timeout_ms, uint32_t startTick) {
	auto last_bytes_received = 0;
 	while (device_busy.getCount() != 1) {
 		size_t received_bytes = _huart.RxXferSize - __HAL_DMA_GET_COUNTER(_huart.hdmarx);
 		if (received_bytes > last_bytes_received) {
 			startTick = HAL_GetTick();
 			last_bytes_received = received_bytes;
 		}
 		if (timer_is_elapsed(startTick, timeout_ms)) {
 			deinit_transmission();
 			HAL_UART_Abort_IT(&_huart);
 			return SerialError::TIMEOUT;
 		}
 	}
 	return SerialError::SUCCESS;
 }
 SerialError UartDriver::read(std::span<uint8_t> buffer, uint32_t timeoutMs, size_t *bytes_read_out) {
	measure(TIME_MEAS::UART_READ_START);
 	uint32_t startTick = HAL_GetTick();
 	if (bytes_read_out != nullptr) {
 		*bytes_read_out = 0;
 	}

 	SerialError result = waitUntilReadyToTransmit(timeoutMs, startTick);
	measure(TIME_MEAS::UART_WAIT_READY);
 	if (result != SerialError::SUCCESS) {
 		return result;
 	}
 	// if (rx_temporary_buffer_index > 0) {
 	// 	std::copy_n(rx_temporary_buffer.begin(),
		// 			 std::min(rx_temporary_buffer_index,
		// 					  buffer.size()),
		// 			 buffer.begin());
 	// 	rx_temporary_buffer_index = 0;
 	// 	deinit_transmission();
 	// 	return SerialError::SUCCESS;
 	// }
	valid_rx_bytes = 0;
 	rx_buffer = buffer;
 	setDirectionPinWrite(false);
	flush();
	measure(TIME_MEAS::UART_FLUSH);
 	HAL_StatusTypeDef res =  HAL_UARTEx_ReceiveToIdle_DMA(&_huart, rx_buffer.data(),
													rx_buffer.size());
 	result = toSerialError(res);
	measure(TIME_MEAS::UART_RECEIVE_SETUP_END);
	if (result != SerialError::SUCCESS) {
		return result;
	}
 	if (timeoutMs > 0) {
 		result = waitForTransmissionComplete(timeoutMs, startTick);
 	}
	measure(TIME_MEAS::UART_RECEIVE_COMPLETE_END);
 	if (bytes_read_out != nullptr) {
 		*bytes_read_out = valid_rx_bytes;
 	}
 	return result;

}

void UartDriver::setDirectionPinWrite(bool cond) {
 	HAL_GPIO_WritePin(_DE_Port,_DE_pin, cond ?GPIO_PinState::GPIO_PIN_SET: GPIO_PinState::GPIO_PIN_RESET);
}


UartDriver::UartDriver(UART_HandleTypeDef& uart_handle,GPIO_TypeDef* DE_Port, uint16_t DE_pin)
:_huart(uart_handle),_DE_Port(DE_Port),_DE_pin(DE_pin) {
	uartDriverInstances.emplace(&_huart,this);
	device_busy.give();
}


 UartDriver::~UartDriver() {
	uartDriverInstances.erase(&_huart);
}


 void UartDriver::baudrate(uint32_t baudrate) {
	if (baudrate == _huart.Init.BaudRate)
		return;
	!device_busy.take();
	HAL_UART_DeInit(&_huart);
	_huart.Init.BaudRate = baudrate;
	HAL_UART_Init(&_huart);
	device_busy.give();
}


 uint32_t UartDriver::baudrate() const {
	return _huart.Init.BaudRate;
}


 void UartDriver::TxCpltCallback(UART_HandleTypeDef *huart) {
	auto it = uartDriverInstances.find(huart);
	if (it != uartDriverInstances.end())
	{
		UartDriver* driver = it->second;
		driver->onTxComplete();
	}
}


 void UartDriver::RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
	// Find the UartDriver instance associated with this huart
	auto it = uartDriverInstances.find(huart);
	if (it != uartDriverInstances.end())
	{
		UartDriver* driver = it->second;
		if (huart->RxEventType == HAL_UART_RXEVENT_IDLE) {
			driver->onRxComplete(Size); // Call an instance method
		}
	}
}


 void UartDriver::onTxComplete() {
	deinit_transmission();
	if (txCompleteCallback) {
		txCompleteCallback();
	}
}


void UartDriver::onRxComplete(uint16_t size) {
	valid_rx_bytes += size;
	deinit_transmission();
	if (rxCompleteCallback) {
		rxCompleteCallback(rx_buffer.subspan(0, std::min(static_cast<size_t>(size), rx_buffer.size())));
	}
}

void UartDriver::onErrorCallback() {
 	deinit_transmission();
}

void UartDriver::ErrorCallback(UART_HandleTypeDef *huart) {
 	auto it = uartDriverInstances.find(huart);
 	if (it != uartDriverInstances.end())
 	{
 		UartDriver* driver = it->second;
 		driver->onErrorCallback();
 	}
}

/**
  * @brief  Tx Transfer completed callback
  * @param  huart: UART handle.
  * @note   This example shows a simple way to report end of DMA Tx transfer, and
  *         you can add your own implementation.
  * @retval None
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
 {
 	UartDriver::TxCpltCallback(huart);

 }
// You'll also need to handle the Receive To Idle callback if you are using it
extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
 {
 	UartDriver::RxEventCallback(huart, Size);
 }
 void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
	UartDriver::ErrorCallback(huart);
 }
void HAL_UART_AbortCpltCallback(UART_HandleTypeDef *huart) {
	size_t received_bytes = huart->RxXferSize - __HAL_DMA_GET_COUNTER(huart->hdmarx);
	UartDriver::RxEventCallback(huart,received_bytes);
 }

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	size_t received_bytes = huart->RxXferSize - __HAL_DMA_GET_COUNTER(huart->hdmarx);
	UartDriver::RxEventCallback(huart,received_bytes);
}
std::unordered_map<UART_HandleTypeDef*, UartDriver*> UartDriver::uartDriverInstances;