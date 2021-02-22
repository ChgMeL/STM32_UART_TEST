
#ifndef __LOGIC_UART_HAL_H
#define __LOGIC_UART_HAL_H
//F:\github\STM32_UART_TEST\Drivers\STM32F0xx_HAL_Driver\Inc\stm32f0xx_hal.h
#include "stm32f0xx_hal.h"
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);	///< Function for getting any data from UART
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart);		///< Check for Error
void HAL_UART_ProcessingMessage_Timeout(UART_HandleTypeDef *huart); ///< MessageProcessing
#endif
