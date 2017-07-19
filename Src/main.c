/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */

#include <string.h>
#include <math.h>
#include "modem/mLoRaWAN.h"
#include "SHT2x/SHT2x.h"
#include "vcom.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

extern uint8_t _version;
char Buffer[100];
//char Rx_indx_1, Rx_data_1[2], Rx_Buffer_1[100], Transfer_cplt_1, Tx_Buffer_1[100];
char Rx_indx_3, Rx_data_3[2], Rx_Buffer_3[100], Transfer_cplt_3, Tx_Buffer_3[100];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

void printAnswer(uint8_t ans)
{
	char cret[30];
	cret[0]='\0';

	switch(ans)
	{
		case LORAWAN_ANSWER_OK:
			sprintf(cret,"LORAWAN_ANSWER_OK\r\n");
		break;
		case LORAWAN_ANSWER_ERROR:
			sprintf(cret,"LORAWAN_ANSWER_ERROR\r\n");
		break;
		case LORAWAN_NO_ANSWER:
			sprintf(cret,"LORAWAN_NO_ANSWER\r\n");
		break;
		case LORAWAN_INIT_ERROR:
			sprintf(cret,"LORAWAN_INIT_ERROR\r\n");
		break;
		case LORAWAN_LENGTH_ERROR:
			sprintf(cret,"LORAWAN_LENGTH_ERROR\r\n");
		break;
		case LORAWAN_SENDING_ERROR:
			sprintf(cret,"LORAWAN_SENDING_ERROR\r\n");
		break;
		case LORAWAN_NOT_JOINED:
			sprintf(cret,"LORAWAN_NOT_JOINED\r\n");
		break;
		case LORAWAN_INPUT_ERROR:
			sprintf(cret,"LORAWAN_INPUT_ERROR\r\n");
		break;
		case LORAWAN_VERSION_ERROR:
			sprintf(cret,"LORAWAN_VERSION_ERROR\r\n");
		break;
	}

	PRINTF("%s\n", cret);
}

void del(uint32_t ms)
{
	HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
	HAL_Delay(200);
	HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
	HAL_Delay(ms);
}

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();

  /* USER CODE BEGIN 2 */

  vcom_Init(huart3);

  uint16_t sT;
  float   temperatureC;           //variable for temperature[°C] as float
  uint8_t  error = 0;              //variable for error code. For codes see system.h

  mUtil_InitUart();
  HAL_UART_Receive_IT(&huart3, Rx_data_3, 1);

//  ON();
//  resetHardware();
//  check();
//  setRetries(0);

  uint8_t join = 1, ret = 0;
  uint8_t count_err = 0;
  uint8_t dr = 0;
  uint32_t del = 10000;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//	  PRINTF("J: %s, count %d\n", (join==0)?"SI":"NO", count_err);
//
//	  if(join != LORAWAN_ANSWER_OK)
//	  {
//		  join = joinOTAA();
//		  printAnswer(join);
//	  }
//	  if(join == LORAWAN_ANSWER_OK)
//	  {
//		  char word[20];
//		  error |= SHT2x_MeasureHM(TEMP, &sT);
//		  temperatureC = SHT2x_CalcTemperatureC(sT);
//		  int d1 = temperatureC;
//		  float f2 = temperatureC - d1;
//		  int d2 = trunc(f2 * 10000);
//		  sprintf(word,"%d.%04d", d1, d2);
//		  int i = 0;
//		  for(i = 0; i<strlen(word); i++){
//			  sprintf(Buffer+i*2, "%02X", word[i]);
//		  }
//		  PRINTF("%s\n", Buffer);
//
//		  dr = 5;
//		  dr = (d2 % 3) + 3;
//		  ret = 1;
//		  ret = setDataRate(dr);
//		  printAnswer(ret);
//
//		  ret = LORAWAN_SENDING_ERROR;
//		  ret = sendConfirmed(2, Buffer);
//		  printAnswer(ret);
//
//		  if(ret == LORAWAN_NOT_JOINED)
//			  join = 1;
//
//		  if(ret == LORAWAN_ANSWER_OK)
//		  {
//			  count_err = 0;
//		  }
//		  else
//			  count_err++;
//	  }
//	  if(count_err>5)
//	  {
//		  resetHardware();
//		  check();
//		  setRetries(0);
//		  count_err = 0;
//		  del(10000);
//	  }
//	  else
//		  del(60000);


//	if(ret_join == LORAWAN_ANSWER_OK)
//	{
//		char word[20];
//		error |= SHT2x_MeasureHM(TEMP, &sT);
//		temperatureC = SHT2x_CalcTemperatureC(sT);
//		int d1 = temperatureC;
//		float f2 = temperatureC - d1;
//		int d2 = trunc(f2 * 10000);
//		sprintf(word,"%d.%04d", d1, d2);
//		int i = 0;
//		for(i = 0; i<strlen(word); i++){
//		sprintf(Buffer+i*2, "%02X", word[i]);
//		}
//		PRINTF("%s\n", Buffer);
//
//		dr = 5;
//		dr = (d2 % 3) + 3;
//		ret = 1;
//		ret = setDataRate(dr);
//		printAnswer(ret);
//
//		ret = LORAWAN_SENDING_ERROR;
//		ret = sendConfirmed(2, Buffer);
//		printAnswer(ret);
//
//		if(ret == 0)
//			count_err = 0;
//		else
//			count_err++;
//	}
//	if(ret == LORAWAN_NOT_JOINED)
//	{
//		  ret_join = LORAWAN_NOT_JOINED;
//		  ret_join = joinOTAA();
//		  printAnswer(ret_join);
//	}
//	if(count_err>=10)
//	{
//		resetHardware();
//		ret_join = LORAWAN_NOT_JOINED;
//		count_err = 0;
//		ret = 1;
//		ret = setRetries(0);
//		printAnswer(ret);
//	}

	  //----------- cineca ok -----------
		ON();
		resetHardware();
		check();
		setRetries(0);
		setDataRate(5);

		if (join != LORAWAN_ANSWER_OK) {
			join = joinOTAA();
			printAnswer(join);
		}
		if (join == LORAWAN_ANSWER_OK) {
			char word[20];
			error |= SHT2x_MeasureHM(TEMP, &sT);
			temperatureC = SHT2x_CalcTemperatureC(sT);
			int d1 = temperatureC;
			float f2 = temperatureC - d1;
			int d2 = trunc(f2 * 10000);
			sprintf(word, "%d.%04d", d1, d2);
			int i = 0;
			for (i = 0; i < strlen(word); i++) {
				sprintf(Buffer + i * 2, "%02X", word[i]);
			}
			PRINTF("%s\n", Buffer);

			ret = LORAWAN_SENDING_ERROR;
			ret = sendConfirmed(1, Buffer);
			printAnswer(ret);
		}
		if(ret != LORAWAN_ANSWER_OK)
			del = 20000;
		else
			del = 10000;
		join = LORAWAN_NOT_JOINED;
		OFF();
	  //----------- end cineca ok -----------

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
		HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
		HAL_Delay(del);
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART3
                              |RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_SYSCLK;
  PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

  __HAL_RCC_PWR_CLK_ENABLE();

  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

//Interrupt callback routine
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

	if (huart->Instance == USART1)	//current UART
	{
		mUtil_Uart_Callback();
	}

//	if (huart->Instance == USART1)	//current UART
//	{
//		Rx_Buffer_1[Rx_indx_1++]=Rx_data_1[0];	//add data to Rx_Buffer
//
//		if (Rx_data_1[0]==10)			//if received data = 13
//		{
//			Rx_Buffer_1[Rx_indx_1]='\0';
//			Rx_indx_1=0;
//			Transfer_cplt_1=1;//transfer complete, data is ready to read
//
//			int n = sprintf(Tx_Buffer_1, "%s", Rx_Buffer_1);
//			//HAL_UART_Transmit(&huart1, (uint8_t*) &Tx_Buffer_1, n, 1000);
//			HAL_UART_Transmit(&huart3, (uint8_t*) &Tx_Buffer_1, n, 1000);
//		}
//
//		HAL_UART_Receive_IT(&huart1, Rx_data_1, 1);	//activate UART receive interrupt every time
//	}
//	HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);

	if (huart->Instance == USART3)	//current UART
	{
		Rx_Buffer_3[Rx_indx_3++]=Rx_data_3[0];	//add data to Rx_Buffer

		if (Rx_data_3[0]==10)			//if received data = 13
		{
			Rx_Buffer_3[Rx_indx_3]='\0';
			Rx_indx_3=0;
			Transfer_cplt_3=1;//transfer complete, data is ready to read

			int n = sprintf(Tx_Buffer_3, "%s", Rx_Buffer_3);
			HAL_UART_Transmit(&huart3, (uint8_t*) &Tx_Buffer_3, n, 1000);
			HAL_UART_Transmit(&huart1, (uint8_t*) &Tx_Buffer_3, n, 1000);
		}

		HAL_UART_Receive_IT(&huart3, Rx_data_3, 1);	//activate UART receive interrupt every time
	}
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
