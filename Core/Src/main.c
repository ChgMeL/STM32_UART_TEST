/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdint.h"
#include <stdio.h>
#include "logic_uart_hal.h"
//#include "logic_uart_hal.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define BUFSIZE 7
//#define RX_BUFFER_SIZE (20)					///< Quantity buffer size Array
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
char trans_str[64] = {0,};
uint16_t adc = 0;						
uint32_t adc_flag = 0;					///< adc flag for callback tim1
uint8_t RX_Complete = 0;				///< RX_complete callback for uart_receive
uint8_t RX_DATA = 0;
//uint8_t RX_BUFFER[RX_BUFFER_SIZE] = {0,}; 	///< Local buffer for UART receive
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t buff = 0;
uint8_t BUFF_TX[15] = {1,2,3,4,5};	///< Fot test
uint8_t BUFF_RX[15] = {0,};					///< Buff receive
uint32_t prev_freq;
uint8_t prev_led_status;

uint8_t rx_buff[BUFSIZE] = {0,};
uint16_t rx_buff_len;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//	if (huart == &huart1)
//		{
//			HAL_GPIO_TogglePin(GPIOC, LED_BLUE_Pin|LED_GREEN_Pin);
//		}
//}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
		if (htim == &htim6)
		{
			adc_flag = 1;
		}
		// htim1 = htim1
		// htim2 = htim2
		if(htim == &htim1)
		{
						uint32_t current_freq = __HAL_TIM_GET_COUNTER(&htim2); // значение в счётчике таймера №2
						
						//uint16_t count_secondary = __HAL_TIM_GET_COUNTER(&htim3); // значение в счётчике таймера №3
						//uint16_t arr = __HAL_TIM_GET_AUTORELOAD(&htim2); // значение переполнения таймера №2 (65535)
						//uint32_t freq = count_main;// + (count_secondary * arr) + count_secondary; // вычисляем

						///////////////////////// вывод инфы ///////////////////////////////
						char str[96] = {0,};
						snprintf(str, 96, "FREQUENCY: %.3f MHz | %.3f KHz | %lu Hz\n--------------------\n", (float)current_freq / 1000000.0, (float)current_freq / 1000.0, current_freq);
						
						// === Данное условие реализовано, чтобы всегда не выводить частоту, а только при изменении её === //
						if (prev_freq != current_freq)
						{
							HAL_UART_Transmit(&huart1, (uint8_t*)str, strlen(str), 1000);
							prev_freq = current_freq;
						}
						
						
//						if (HAL_GPIO_ReadPin(GPIO_Input_Test_GPIO_Port,GPIO_Input_Test_Pin))
//							HAL_UART_Transmit(&huart1,"LED ON \n", 9, 1000);
//						else
//							HAL_UART_Transmit(&huart1,"LED OFF \n", 10, 1000);
						//////////////// обнуляем счётчики и рестартуем таймер №1 /////////////////
						__HAL_TIM_SET_COUNTER(&htim2, 0x0000);	
						//__HAL_TIM_SET_COUNTER(&htim3, 0x0000);
						
						//HAL_TIM_Base_Start_IT(&htim1); ///< !----Test  на работу без него
						__HAL_TIM_ENABLE(&htim1);
						//TIM1->CR1 |= TIM_CR1_CEN;				// Ручное включение таймера
		}
}
// void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{

//		if (huart == &huart1)
//		{
//			uint8_t *p_RX_BUFFER;						///< Pointer for RX Buffer
//			uint8_t rx_buffer_counter = 0;	///< Counter for number of receiving bits
//			p_RX_BUFFER = RX_BUFFER;				///< Point for 1st RX Buffer byte (Receive)
//			
//			if (*p_RX_BUFFER != '\n')				///< "\n" = 0x0A; work according end of line
//			{
//				*p_RX_BUFFER == RX_BUFFER[rx_buffer_counter++];
//				p_RX_BUFFER++;
//			}
//			else
//			{
//				HAL_UART_Transmit(&huart1,(uint8_t*) RX_BUFFER,rx_buffer_counter,1000);
//				HAL_UART_Receive_IT(&huart1,RX_BUFFER,1);
//				rx_buffer_counter = 0;
//			}
//		}
//		
//}
//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//		if (huart == &huart1)
//		{
//			HAL_UART_Transmit(&huart1,(uint8_t*)BUFF_RX,5,1000);
//			HAL_UART_Receive_IT(&huart1,(uint8_t*)BUFF_RX,5);
//		}
//}

//void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
//{
//	if (huart == &huart1)
//	{
//		HAL_UART_Transmit(&huart1,(uint8_t*)huart->ErrorCode,4,1000);
//	}
//}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
	
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_ADC_Init();
  MX_TIM6_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_TIM17_Init();
  /* USER CODE BEGIN 2 */
	//__HAL_TIM_CLEAR_IT(&htim1, TIM_IT_UPDATE); // очищаем бит
	//HAL_TIM_Base_Start_IT(&htim1);									// Видимо был нужен для предыдущих тестов
	//HAL_TIM_Base_Start_IT(&htim6);									// Таймер с прерыванием в 1 сек.
	HAL_TIM_Base_Start_IT(&htim1);										// Таймер для расчета частоты ( отсчитываем 1 секунду)
	HAL_TIM_Base_Start(&htim2);												// Таймер работает в режиме захвата импульсов и подсчитываем количество положительных импульсов , приходящих на ETR2 сигнал (TIM2_ETR2)
	HAL_TIM_PWM_Start(&htim17,TIM_CHANNEL_1);		// Для генерирования PWM сигнала , базовая (500 Гц)
	
	//HAL_UART_ReceiverTimeout_Config(&huart1,10000);	// Вроде как настройки на timeout после принятия посылки
	//HAL_UART_EnableReceiverTimeout(&huart1);			// Вроде как активация timeout по приему, настроенному сверху 
	
	
	//HAL_ADCEx_Calibration_Start(&hadc);
		//HAL_GPIO_WritePin(GPIOC, LED_BLUE_Pin, GPIO_PIN_SET);
		//HAL_GPIO_WritePin(GPIOC, LED_GREEN_Pin, GPIO_PIN_SET);
		HAL_UART_Receive_IT(&huart1,(uint8_t*)BUFF_RX,1);			///< Активация UART1 работы по приерывания и обработка по 1 байту
	HAL_GPIO_WritePin(GPIO_Output_Test_GPIO_Port, GPIO_Output_Test_Pin, GPIO_PIN_SET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

	  while (1)
  {
	
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	//HAL_UART_ProcessingMessage_Timeout(&huart1);
		//HAL_UART_Transmit(&huart1,(uint8_t*)BUFF_TX,5,1000);
//		if (adc_flag == 1)
//		{
//			HAL_ADC_Start(&hadc); // запускаем преобразование сигнала АЦП
//			HAL_ADC_PollForConversion(&hadc, 100); // ожидаем окончания преобразования
//			snprintf(trans_str, 63, "ADC %d\n", adc);
//			HAL_UART_Transmit(&huart1, (uint8_t*)trans_str, strlen(trans_str), 1000);
//		}
		//HAL_UART_Receive_DMA(&huart1, (uint8_t*)buff, 1);
			//HAL_GPIO_TogglePin(GPIOC, LED_BLUE_Pin);
			//HAL_GPIO_TogglePin(GPIOC, LED_GREEN_Pin);
		//	HAL_Delay(1000);
		/* USER CODE BEGIN 3 */
//		if (adc_flag == 1)
//		{
//			HAL_GPIO_TogglePin(GPIOC, LED_GREEN_Pin);
//			HAL_UART_Transmit(&huart1,(uint8_t*)BUFF_TX,5,1000);
//		}
//		adc_flag = 0;
//		
//		if (RX_Complete == 1) 
//		{
//			//huart1.gState = HAL_UART_STATE_READY;
//			HAL_UART_Transmit(&huart1,(uint8_t*)BUFF_RX,5,1000);
//			HAL_GPIO_TogglePin(GPIOC, LED_BLUE_Pin);
//			HAL_UART_Receive_IT(&huart1, (uint8_t*)&BUFF_RX, 5);
//			RX_Complete = 0;
//		}
		

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI14|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_RCC_MCOConfig(RCC_MCO, RCC_MCO1SOURCE_SYSCLK, RCC_MCODIV_1);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
