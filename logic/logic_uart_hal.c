
#include "logic_uart_hal.h"
#include "usart.h"
#include "tim.h"
#include "string.h"
#include "stdlib.h"

#define RX_BUFFER_SIZE 20					///< Quantity buffer size Array
#define FCLK 48000000UL						///< 48 MHz

uint8_t RX_BUFFER[RX_BUFFER_SIZE] = {0,}; 	///< Local buffer for UART receive
uint8_t rx_buffer_counter = 0;	///< Counter for number of receiving bits

uint8_t StringToInt(uint8_t* data_input_integer)
{
	uint8_t result = 0;
	if ((*data_input_integer >= '0') && (*data_input_integer <= '9'))
		result = *data_input_integer - 0x30;
	else
		//error
	return result;			
}
void MessageProcessing(uint8_t* string_processing)
{
	uint16_t value = 0;	//65535 max uint8_t
	uint8_t buffer_for_integer[10] = {0,};
	uint8_t i = 0;
	if (strncmp("PWM",(uint8_t*)string_processing, 3) == 0)	///< если совпала команда с PWM, переходим к обработке данным 0 -совпало, 1 - не совпало
		{
			string_processing +=4;	///Пропускаем фразу PWM, пробел и ставим указатель на первую возможную цифру и начинаем работать с цифрами
			while(*string_processing != '\r')			//
			{
				//StringToInt((uint8_t*)string_processing);
				
				buffer_for_integer[i] = *string_processing;
				string_processing++;
				i++;
			}
			value = atoi((uint8_t*)buffer_for_integer);
			
			// Обработчик таймера для PWM
			__HAL_TIM_DISABLE(&htim17);
			TIM17->ARR = ((FCLK/value/48)-1);	/// Auto Reload Register
			TIM17->CCR1 = ((TIM17->ARR)/2);
			TIM17->CNT = 0;
			__HAL_TIM_ENABLE(&htim17);
		}
		
}
void HAL_UART_ProcessingMessage_Timeout(UART_HandleTypeDef *huart)
{
	///< ISR-> RTOF
	///< ICR->RTOCF if (RTOCF == 1)
	if (((huart->Instance->ISR & USART_ISR_RTOF) == 0) && rx_buffer_counter != 0)
	{
		HAL_UART_Transmit(huart,(uint8_t*) RX_BUFFER,rx_buffer_counter,1000);
	}
		
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	///< Хочу сделать обработчик надписи "PWM <FREQ>"
	//uint8_t LocalBuff  = 0;	///< Локальный буффер 
		if (huart == &huart1)
		{
			
			if (huart1.Instance->RDR != '\r')				///< "\n" = 0x0A; work according end of line (get Data from uart register)
			{
				RX_BUFFER[rx_buffer_counter] = huart1.Instance->RDR;
				rx_buffer_counter++;
				
				// === Если вдруг пришло количество данных больше , чем буффер === //
				if (rx_buffer_counter > RX_BUFFER_SIZE)
				{
					HAL_UART_Transmit(&huart1,(uint8_t*) RX_BUFFER,rx_buffer_counter,1000);
					rx_buffer_counter = 0;
				}
				
			}
			else // Если пришел символ конца строки '\r'
			{
				
				MessageProcessing((uint8_t*)RX_BUFFER);	///< Обработчик сообщения команды ( в данном случае PWM)
				RX_BUFFER[rx_buffer_counter++] = '\n';
				HAL_UART_Transmit(&huart1,(uint8_t*) RX_BUFFER,rx_buffer_counter,1000);
				//HAL_UART_Receive_IT(&huart1,RX_BUFFER,1);
				for (int i = 0; i < rx_buffer_counter; i++)
				{
					RX_BUFFER[rx_buffer_counter] = 0;
				}
				rx_buffer_counter = 0;
				//HAL_TIM_PWM_Stop_IT(&htim17,TIM_CHANNEL_1);
			}
		HAL_UART_Receive_IT(&huart1,(RX_BUFFER+rx_buffer_counter),1);
		}
		
}

//#define  HAL_UART_ERROR_NONE             (0x00000000U)    /*!< No error                */
//#define  HAL_UART_ERROR_PE               (0x00000001U)    /*!< Parity error            */
//#define  HAL_UART_ERROR_NE               (0x00000002U)    /*!< Noise error             */
//#define  HAL_UART_ERROR_FE               (0x00000004U)    /*!< Frame error             */
//#define  HAL_UART_ERROR_ORE              (0x00000008U)    /*!< Overrun error           */
//#define  HAL_UART_ERROR_DMA              (0x00000010U)    /*!< DMA transfer error      */
//#define  HAL_UART_ERROR_RTO              (0x00000020U)    /*!< Receiver Timeout error  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
		if (huart == &huart1)
			{
			switch(huart1.ErrorCode)
			{
				char *string_error;	///< Text ( string) for output error
				
				case HAL_UART_ERROR_NONE:
					///< add debug print
				string_error = "NoE\n";
					HAL_UART_Transmit(&huart1,(uint8_t*)string_error,strlen(string_error),1000);
				break;
				
				case HAL_UART_ERROR_PE:
					///< add debug print
				string_error = "PE\n";
					HAL_UART_Transmit(&huart1,(uint8_t*)string_error,sizeof(string_error),1000);
				break;
				
				case HAL_UART_ERROR_NE:
					///< add debug print
				string_error = "NE\n";
					HAL_UART_Transmit(&huart1,(uint8_t*)string_error,sizeof(string_error),1000);
				break;
				
				case HAL_UART_ERROR_ORE:
					///< add debug print
				string_error = "ORE\n";
					HAL_UART_Transmit(&huart1,(uint8_t*)string_error,sizeof(string_error),1000);
				break;
				
				case HAL_UART_ERROR_DMA:
					///< add debug print
				string_error = "DMAE\n";
					HAL_UART_Transmit(&huart1,(uint8_t*)string_error,sizeof(string_error),1000);
				break;
				
				case HAL_UART_ERROR_RTO:
					///< add debug print
				string_error = "RTOE\n";
					HAL_UART_Transmit(&huart1,(uint8_t*)string_error,sizeof(string_error),1000);
				break;
				
				default:
					string_error = "NoErr\n";
					HAL_UART_Transmit(&huart1,(uint8_t*)string_error,sizeof(string_error),1000);
				break;
			}
		}
	
}

