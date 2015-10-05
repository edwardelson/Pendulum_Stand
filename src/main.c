/*
 * main.c
 *
 *  Created on: Sep 24, 2015
 *      Author: Edward Elson
 *
 *	This is the codes to control electronics for thrust measurement device
 *	sensors and actuators:
 *	1. accelerometer
 *	2. distance sensor
 *	3. valve control circuit
 *	4. TFT LCD
 *	5. SD Card
 *
 */

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_rcc.h"
#include "stm32f4xx_hal_i2c.h"
#include "stm32f4xx_hal_uart.h"
#include "stm32f4xx_hal_adc.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_cortex.h"
#include "stm32f4xx_hal_pwr_ex.h"
#include "stm32f4xx_hal_flash_ex.h"
#include "cmsis_os.h"
#include "string.h"

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;
I2C_HandleTypeDef hi2c1;
ADC_HandleTypeDef hadc1;

osThreadId task1Thread; //LED blinking and UART transmission
osThreadId task2Thread; //valve control circuit
osThreadId task3Thread; //accelerometer

uint8_t valve_on = 0;

/* Private function prototypes -----------------------------------------------*/
void Clock_Config();
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);

void task1Function(void const * argument);
void task2Function(void const * argument);
void task3Function(void const * argument);

/* Main ----------------------------------------------------------------------*/
int main(void)
{
	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* Configure the system clock */
	Clock_Config();

	/* Configure GPIO settings */
	MX_GPIO_Init();
	MX_USART2_UART_Init();
	MX_ADC1_Init();
	MX_I2C1_Init();

	/* Configure I2C Devices */

//	TMP006_Status = config_TMP006(&hi2c1);

	/* Threads Creation */
	osThreadDef(task1, task1Function, osPriorityNormal, 0, 128);
	task1Thread = osThreadCreate(osThread(task1), NULL);

	osThreadDef(task2, task2Function, osPriorityNormal, 0, 128);
	task2Thread = osThreadCreate(osThread(task2), NULL);

	osThreadDef(task3, task3Function, osPriorityNormal, 0, 128);
	task3Thread = osThreadCreate(osThread(task3), NULL);

	/* Start freeRTOS kernel */
	osKernelStart();
	while (1);
}

/* System Clock Configuration
 * Taken from STM32F4CubeMX auto-generated files */
void Clock_Config()
{
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  __PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

}

/* GPIO configuration */
void MX_GPIO_Init(void)
{
	  GPIO_InitTypeDef GPIO_InitStruct;

	  /* GPIO Ports Clock Enable */
	  __GPIOC_CLK_ENABLE();
	  __GPIOH_CLK_ENABLE();
	  __GPIOA_CLK_ENABLE();
	  __GPIOB_CLK_ENABLE();

	  /* The following pin configurations depend on application.
	   * As we'd like to turn on LED and use the switch,
	   * we need to set them correspondingly
	   * check datasheet for more information
	   */

	  /*Configure GPIO pin : PC13 */
	  GPIO_InitStruct.Pin = GPIO_PIN_13;
	  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	  /*Configure GPIO pins : PA2 PA3 */
	  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
	  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
	  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
	  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	  /*Configure GPIO pins : PA5 PA6 */
	  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
	  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/* USART2 init function */
void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart2);

}

/* ADC1 init function */
void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION12b;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = EOC_SINGLE_CONV;
  HAL_ADC_Init(&hadc1);

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
    */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

}

/* I2C1 init function */
void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLED;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLED;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLED;
  HAL_I2C_Init(&hi2c1);

}


/* Task 1 : Find Accelerometer Reading*/
void task1Function(void const * argument)
{
//
    while(1)
	{
//		if (osSemaphoreWait(sem1Handle, osWaitForever) == osOK)
//		{
//
			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
//			//HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_6);
//
//			osSemaphoreRelease(sem1Handle);
			osDelay(1000);
		}
//	}
}

/* Task 2: Valve Control */
void task2Function (void const * argument)
{
	while(1)
	{
		if((!HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13)) && (valve_on == 0)) //when switch is pressed
		{
			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_6);
			osDelay(70);
			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_6);

			valve_on = 1;
		}
		else if ((!HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13)))
		{
			valve_on = 0;
			osDelay(3000);
		}
		else
		{
			osDelay(500);
		}

	}
}

/* Task 3 : Read Accelerometer*/
void task3Function(void const * argument)
{
//	int accX, accY, accZ = 0;
//
//
    while(1)
	{
//		//read adc value from PA0
//		HAL_ADC_Start(&hadc1);
//		if (HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY)== 0x00)
//		{
//			acc = HAL_ADC_GetValue(&hadc1);
//		}
//		HAL_ADC_Stop(&hadc1);
//
//		adc = read_TMP006(&hi2c1);
//
//		//UART Transmission
//		char str[15] = {0};
//		char *msg = str;
//		sprintf(str, "%d", adc);
//		strcat(&str, "\n\r"); //add newline
//		HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 0xFFFF);
//
//		osSemaphoreRelease(sem1Handle);
		osDelay(500);
		}
//	}
}

/* additional code from STM32 */
#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t* file, uint32_t line)
#endif
