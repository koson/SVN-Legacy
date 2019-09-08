/**
 ******************************************************************************
 * File Name          : USART.c
 * Description        : This file provides code for the configuration
 *                      of the USART instances.
 ******************************************************************************
 ** This notice applies to any and all portions of this file
 * that are not between comment pairs USER CODE BEGIN and
 * USER CODE END. Other portions of this file, whether
 * inserted by the user or by software development tools
 * are owned by their respective copyright owners.
 *
 * COPYRIGHT(c) 2019 STMicroelectronics
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
#include "usart.h"

/* USER CODE BEGIN 0 */
bool uartReceived;
extern uint8_t testdata;
extern uint8_t UartRecBuffer[BUFFER_SIZE];
extern USBtoCAN_t USBtoCAN;
uint8_t receiveTimes = 0;
uint8_t RecFIFO[FIFO_SIZE];
uint8_t fifoCounter = 0;
/* USER CODE END 0 */

UART_HandleTypeDef huart3;

/* USART3 init function */

void MX_USART3_UART_Init(void) {

	huart3.Instance = USART3;
	huart3.Init.BaudRate = 115200;
	huart3.Init.WordLength = UART_WORDLENGTH_8B;
	huart3.Init.StopBits = UART_STOPBITS_1;
	huart3.Init.Parity = UART_PARITY_NONE;
	huart3.Init.Mode = UART_MODE_TX_RX;
	huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart3.Init.OverSampling = UART_OVERSAMPLING_16;
	huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart3) != HAL_OK) {
		Error_Handler();
	}
}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle) {

	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	if (uartHandle->Instance == USART3) {
		/* USER CODE BEGIN USART3_MspInit 0 */

		/* USER CODE END USART3_MspInit 0 */
		/* USART3 clock enable */
		__HAL_RCC_USART3_CLK_ENABLE()
		;

		__HAL_RCC_GPIOE_CLK_ENABLE()
		;
		__HAL_RCC_GPIOB_CLK_ENABLE()
		;
		/**USART3 GPIO Configuration
		 PE15     ------> USART3_RX
		 PB10     ------> USART3_TX
		 */
		GPIO_InitStruct.Pin = GPIO_PIN_15;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
		HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

		GPIO_InitStruct.Pin = GPIO_PIN_10;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

		/* USART3 interrupt Init */
		HAL_NVIC_SetPriority(USART3_IRQn, 1, 0);
		HAL_NVIC_EnableIRQ(USART3_IRQn);
		/* USER CODE BEGIN USART3_MspInit 1 */

		/* USER CODE END USART3_MspInit 1 */
	}
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle) {

	if (uartHandle->Instance == USART3) {
		/* USER CODE BEGIN USART3_MspDeInit 0 */

		/* USER CODE END USART3_MspDeInit 0 */
		/* Peripheral clock disable */
		__HAL_RCC_USART3_CLK_DISABLE();

		/**USART3 GPIO Configuration
		 PE15     ------> USART3_RX
		 PB10     ------> USART3_TX
		 */
		HAL_GPIO_DeInit(GPIOE, GPIO_PIN_15);

		HAL_GPIO_DeInit(GPIOB, GPIO_PIN_10);

		/* USART3 interrupt Deinit */
		HAL_NVIC_DisableIRQ(USART3_IRQn);
		/* USER CODE BEGIN USART3_MspDeInit 1 */

		/* USER CODE END USART3_MspDeInit 1 */
	}
}

/* USER CODE BEGIN 1 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_RESET);

//	for (uint16_t i = fifoCounter; i < fifoCounter + 4; i++) {
//		RecFIFO[i & (FIFO_SIZE - 1)] = UartRecBuffer[i - fifoCounter];
//	}
//	fifoCounter += 4;
	uartReceived = 1;
	RecFIFO[fifoCounter++ & (FIFO_SIZE - 1)] = UartRecBuffer[0];

//	uint8_t count = 0;
//	uint8_t count2 = 0;
//	while ((UartRecBuffer[count] != STARTFLAG) && (count < BUFFER_SIZE)) {
//		count++;
//	}
//	USBtoCAN.usbData[0] = UartRecBuffer[count];
//	while ((UartRecBuffer[(count + count2) & ((BUFFER_SIZE - 1))] != ENDFLAG)
//			&& (count2 < BUFFER_SIZE)) {
//		USBtoCAN.usbData[count2] = UartRecBuffer[(count + count2)
//				& ((BUFFER_SIZE - 1))];
//		count2++;
//	}
//	USBtoCAN.usbData[count2] = UartRecBuffer[(count + count2)
//			& ((BUFFER_SIZE - 1))];
//	memset(UartRecBuffer, 0, sizeof(UartRecBuffer));
//	uartReceived = 1;

//	volatile uint8_t endIndex = BUFFER_SIZE - 1;
//	volatile uint8_t startIndex = 0;
//	while ((UartRecBuffer[endIndex] != ENDFLAG) && (endIndex >= 0)) {
//		endIndex--;
//	}
//
//	while ((UartRecBuffer[(endIndex - startIndex) & ((BUFFER_SIZE - 1))]
//			!= STARTFLAG) && (startIndex < BUFFER_SIZE)) {
////		USBtoCAN.usbData[BUFFER_SIZE - 1 - count2] = UartRecBuffer[(count - count2)
////				& ((BUFFER_SIZE - 1))];
//		startIndex++;
//	}
//	startIndex = endIndex - startIndex;
//	for (int i = 0; i < abs(startIndex - endIndex) + 1; i++) {
//		USBtoCAN.usbData[i] = UartRecBuffer[startIndex + i];
//	}
////	USBtoCAN.usbData[startIndex] = UartRecBuffer[(count - count2)
////			& ((BUFFER_SIZE - 1))];
//	memset(UartRecBuffer, 0, sizeof(UartRecBuffer));
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_RESET);
	asm(" nop");
}

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
