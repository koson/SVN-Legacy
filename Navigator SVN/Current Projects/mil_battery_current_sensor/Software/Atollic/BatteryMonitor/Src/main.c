/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 ** This notice applies to any and all portions of this file
 * that are not between comment pairs USER CODE BEGIN and
 * USER CODE END. Other portions of this file, whether
 * inserted by the user or by software development tools
 * are owned by their respective copyright owners.
 *
 * COPYRIGHT(c) 2018 STMicroelectronics
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
#include "main.h"
#include "stm32f0xx_hal.h"
#include <math.h>


/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

CAN_HandleTypeDef hcan;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#define CAN_MOTHERBOARD_ID 0x00
#define CAN_TX_ID 0x09
#define FiltIDHigh 0xFFFF
#define FiltIDLow 0xFFFF
#define FiltMaskIdHigh 0x0000 //specifying which bits of the identifier
#define FiltMaskIdLow 0xFFFF  //are handled as �must match� (1) or as �don�t care� (0).
#define RESISTOR1 698000.0f
#define RESISTOR2 100000.0f
#define RESISTOR2_V2 063400.0f
#define AdcMax 4095.0f
#define Vref 3.3f
#define Vref_V2 2.0f
#define DEBUG 1

battery_t BatteryMonitor;
int TimerIntFlag = 0;
CanTxMsgTypeDef TxMessage;
CanRxMsgTypeDef RxMessage;
CAN_FilterConfTypeDef rxFilter;
char *CanReception;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC_Init(void);
static void MX_TIM2_Init(void);
static void MX_CAN_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void GetVoltageCurrent(void);
void InitBattery(void);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 *
 * @retval None
 */
int main(void) {
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration----------------------------------------------------------*/

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
	MX_ADC_Init();
	MX_TIM2_Init();
	MX_CAN_Init();
	/* USER CODE BEGIN 2 */
	InitBattery();

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */
		/* USER CODE BEGIN 3 */
		if (TimerIntFlag == 1) {
			TimerIntFlag = 0;
			BatteryMonitor.GetReadings();
			for (int i = 0; i < 4; i++) {
				hcan.pTxMsg->Data[i] = BatteryMonitor.Current.CanData[i];
				hcan.pTxMsg->Data[i + 4] = BatteryMonitor.Voltage.CanData[i];
			}
			HAL_CAN_Transmit(&hcan, 30);
		}
	}
	/* USER CODE END 3 */

}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {

	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = 16;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Configure the Systick interrupt time
	 */
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

	/**Configure the Systick
	 */
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	/* SysTick_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC init function */
static void MX_ADC_Init(void) {

	ADC_ChannelConfTypeDef sConfig;

	/**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc.Instance = ADC1;
	hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
	hadc.Init.Resolution = ADC_RESOLUTION_12B;
	hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
	hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	hadc.Init.LowPowerAutoWait = DISABLE;
	hadc.Init.LowPowerAutoPowerOff = DISABLE;
	hadc.Init.ContinuousConvMode = DISABLE;
	hadc.Init.DiscontinuousConvMode = ENABLE;
	hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc.Init.DMAContinuousRequests = DISABLE;
	hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
	if (HAL_ADC_Init(&hadc) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Configure for the selected ADC regular channel to be converted.
	 */
	sConfig.Channel = ADC_CHANNEL_0;
	sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
	sConfig.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;
	if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Configure for the selected ADC regular channel to be converted.
	 */
	sConfig.Channel = ADC_CHANNEL_1;
	if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

}

/* CAN init function */
static void MX_CAN_Init(void) {

	hcan.Instance = CAN;
	hcan.Init.Prescaler = 10;
	hcan.Init.Mode = CAN_MODE_NORMAL;
	hcan.Init.SJW = CAN_SJW_1TQ;
	hcan.Init.BS1 = CAN_BS1_6TQ;
	hcan.Init.BS2 = CAN_BS2_1TQ;
	hcan.Init.TTCM = DISABLE;
	hcan.Init.ABOM = DISABLE;
	hcan.Init.AWUM = DISABLE;
	hcan.Init.NART = DISABLE;
	hcan.Init.RFLM = DISABLE;
	hcan.Init.TXFP = DISABLE;
	if (HAL_CAN_Init(&hcan) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

}

/* TIM2 init function */
static void MX_TIM2_Init(void) {

	TIM_ClockConfigTypeDef sClockSourceConfig;
	TIM_SlaveConfigTypeDef sSlaveConfig;
	TIM_MasterConfigTypeDef sMasterConfig;

	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 128;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 65536;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	sSlaveConfig.SlaveMode = TIM_SLAVEMODE_DISABLE;
	sSlaveConfig.InputTrigger = TIM_TS_ITR0;
	if (HAL_TIM_SlaveConfigSynchronization(&htim2, &sSlaveConfig) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig)
			!= HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

}

/** Pinout Configuration
 */
static void MX_GPIO_Init(void) {

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOA_CLK_ENABLE()
	;
	__HAL_RCC_GPIOB_CLK_ENABLE()
	;

}

/* USER CODE BEGIN 4 */

void CanSetup() {
	if (DEBUG) {
		hcan.Instance->MCR = 0x60;
	}

	hcan.pTxMsg = &TxMessage;
	hcan.pRxMsg = &RxMessage;

	/*##-2- Configure the CAN Filter ###########################################*/
	rxFilter.FilterNumber = 0;
	rxFilter.FilterMode = CAN_FILTERMODE_IDMASK;
	rxFilter.FilterScale = CAN_FILTERSCALE_32BIT;
	rxFilter.FilterIdHigh = 0x0000;
	rxFilter.FilterIdLow = CAN_MOTHERBOARD_ID;
	rxFilter.FilterMaskIdHigh = 0x0000;
	rxFilter.FilterMaskIdLow = 0x0000;
	rxFilter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	rxFilter.FilterActivation = DISABLE;
	rxFilter.BankNumber = 14;

	if (HAL_CAN_ConfigFilter(&hcan, &rxFilter) != HAL_OK) {
		/* Filter configuration Error */
		Error_Handler();
	}

	hcan.pTxMsg->StdId = CAN_TX_ID; /*Specifies the standard identifier. This parameter must be a number between Min_Data
	 = 0 and Max_Data = 0x7FF.*/
	hcan.pTxMsg->ExtId = 0x00; /*Specifies the extended identifier. This parameter must be a number between
	 Min_Data = 0 and Max_Data = 0x1FFFFFFF.*/
	hcan.pTxMsg->RTR = CAN_RTR_DATA; /*Specifies the type of identifier for the message that will be transmitted. This parameter
	 can be a value of CAN_identifier_type*/
	hcan.pTxMsg->IDE = CAN_ID_STD; /*Specifies the type of frame for the received message. This parameter can be a value
	 of CAN_remote_transmission_request*/
	hcan.pTxMsg->DLC = 8; /*Specifies the length of the frame that will be transmitted. This parameter must be a number between
	 Min_Data = 0 and Max_Data = 8. */
	hcan.pTxMsg->Data[0] = 0; /*Contains the data to be transmitted. This parameter must be a number between
	 Min_Data = 0 and Max_Data = 0xFF.*/
}

void InitBattery() {
	CanSetup();
	HAL_ADCEx_Calibration_Start(&hadc);
	BatteryMonitor.GetReadings = &GetVoltageCurrent;
	HAL_TIM_Base_Start_IT(&htim2);
}

void GetVoltageCurrent() {
	BatteryMonitor.Current.Diagnostic = 0;
	BatteryMonitor.Voltage.Diagnostic = 0;
	int LpfTaps = 5000;

	for (int i = 0; i < LpfTaps; i++) {
		HAL_ADC_Start(&hadc);
		while (HAL_ADC_PollForConversion(&hadc, 10) != HAL_OK)
			;
		BatteryMonitor.Current.Diagnostic += (float) (HAL_ADC_GetValue(&hadc));

		HAL_ADC_Start(&hadc);
		while (HAL_ADC_PollForConversion(&hadc, 10) != HAL_OK)
			;
		BatteryMonitor.Voltage.Diagnostic += (float) (HAL_ADC_GetValue(&hadc));
		asm(" nop");
	}
	HAL_ADC_Stop(&hadc);

	BatteryMonitor.Current.Diagnostic /= (float) LpfTaps;
	asm(" nop");
	//	BatteryMonitor.Current.Diagnostic = ((BatteryMonitor.Current.Diagnostic) / AdcMax) * Vref;
//BatteryMonitor.Current.Diagnostic = 28.21f*BatteryMonitor.Current.Diagnostic-0.2195f;

	BatteryMonitor.Voltage.Diagnostic /= (float) LpfTaps;
	BatteryMonitor.Voltage.Diagnostic = ((BatteryMonitor.Voltage.Diagnostic) / AdcMax) * Vref; //Vres_ref !=
	BatteryMonitor.Voltage.Diagnostic = ((RESISTOR1 + RESISTOR2_V2) / (RESISTOR2_V2)) * (BatteryMonitor.Voltage.Diagnostic);
//	if (BatteryMonitor.Current.Diagnostic < 0.019f)
//	{
//		BatteryMonitor.Current.Diagnostic = 0;//BatteryMonitor.Current.Diagnostic*15970.0f-315.7f;
//	}
//	else
//	{
//		BatteryMonitor.Current.Diagnostic = BatteryMonitor.Current.Diagnostic*209.2f+0.27762;
//	}
	BatteryMonitor.Current.Diagnostic = 0.1505f*(BatteryMonitor.Current.Diagnostic)- 0.01677f + 0.10f;
	asm(" nop");
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  file: The file name as string.
 *
 *
 * @param  line: The line in file as a number.
 * @retval None
 */
void _Error_Handler(char *file, int line) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	while (1) {
	}
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
void assert_failed(uint8_t* file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	 tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
