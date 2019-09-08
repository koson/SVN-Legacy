/*
 * USBtoCAN.c
 *
 *  Created on: Sep 26, 2018
 *      Author: rosem
 */

#include <USBtoCAN.h>

USBtoCAN_t USBtoCAN;

extern UART_HandleTypeDef huart3;
extern CAN_HandleTypeDef hcan;
extern CAN_TxHeaderTypeDef TxHeader;
extern CAN_RxHeaderTypeDef RxHeader;
extern uint8_t TxData[8];
extern uint8_t RxData[8];
extern uint32_t TxMailbox;
extern bool canReceived;
extern bool uartReceived;
extern uint8_t RecFIFO[FIFO_SIZE];
extern TIM_HandleTypeDef htim6;
extern uint8_t fifoCounter;
//uint8_t startFlagIndex = 0;
uint16_t startByteIndex = 0;
uint16_t lastStartIndex = 0xFFFF;
//uint8_t endFlagIndex = 0;
uint16_t endByteIndex = 0;
uint16_t packetDataLength = 0;
uint8_t lastID = 0;
uint8_t count;
uint8_t firstTime = 1;
bool startFound;
bool endFound;
bool packetReady = 0;
volatile uint64_t checksumCheck;
volatile uint64_t checksum;
volatile uint8_t UartRecBuffer[BUFFER_SIZE];

// TODO: Remove these debug variables
uint16_t debug_packetFound_count = 0;
uint16_t debug_packetFoundCheck_count = 0;
uint16_t debug_canTX_count = 0;
uint16_t debug_canTX_good_count = 0;
uint16_t debug_canRX_count = 0;
uint16_t debug_canRXCheck_count = 0;
uint16_t debug_waitLoop_count = 0;
uint8_t emptyTxMailboxAvailable = 0x07;

/*Test Buffers*/
uint8_t moboBuffer[DEFAULT_DATA_LENGTH];
uint8_t sensorsBuffer[DEFAULT_DATA_LENGTH];

void rotateByOne(uint8_t arr[]) {
	uint8_t i, first;

	/* Store first element of array */
	first = arr[0];

	for (i = 0; i < BUFFER_SIZE - 1; i++) {
		/* Move each array element to its left */
		arr[i] = arr[i + 1];
	}

	/* Copies the first element of array to last */
	arr[BUFFER_SIZE - 1] = first;
}

void InitUSBtoCAN() {
	USBtoCAN.UsbSend = &USBsend;
	USBtoCAN.UsbReceive = &USBreceive;
	USBtoCAN.CanSend = &CANsend;
	USBtoCAN.CanReceive = &CANreceive;
	USBtoCAN.CanRecID = 0;
	USBtoCAN.DataLength = 0;
	canSetup(8);
	TxHeader.StdId = 0;
	HAL_TIM_Base_Start_IT(&htim6);

}

void USBtoCAN_RUN() {
	//cin << endl << "/***************Waiting for start flag....***************/" << endl;
//	if (startByteIndex != (fifoCounter & (FIFO_SIZE - 1))) {
//		checkPacket();
//		uartReceived = 0;
//	}

//if (emptyTxMailboxAvailable == 0x07)
//{
	if (uartReceived) {
		checkPacket();
		uartReceived = 0;
	}

	if (packetReady) {
		DecodeHeader();
		packetReady = 0;
	}

	if (USBtoCAN.R_nT == TRANSMIT) {
		SendUsbDataToP1_CAN();
		emptyTxMailboxAvailable = 0;
		USBtoCAN.R_nT = 0;
	}
	//}

	if (canReceived) {
		RecCanDataFromP1();
		SendCanDataToP0_USB();
		canReceived = 0;
	}
}

void checkPacket() {
	startFound = 1;
	if (RecFIFO[fifoCounter - 1] == ENDFLAG) {
		uint8_t endFlagPtr = fifoCounter - 1;
		volatile uint8_t distanceFromStart = 0;
		while (RecFIFO[endFlagPtr] != STARTFLAG) {
			distanceFromStart++;
			if (distanceFromStart > PACKET_MAX_SIZE) {
				startFound = 0;
			}
			endFlagPtr--;
		}
		if (startFound) {
			endFlagPtr = fifoCounter - 1;
			for (int i = 0; i <= distanceFromStart; i++) {
				uint8_t distanceFromEnd = (distanceFromStart - i);
				USBtoCAN.usbData[i] = RecFIFO[((endFlagPtr) - distanceFromEnd)
						& (FIFO_SIZE - 1)];
			}
			packetReady = 1;

			// TODO: Remove this debug variable
			debug_packetFound_count += 1;
		}
	}

//	for (uint16_t i = startFlagIndex; i < FIFO_SIZE + startFlagIndex; i++) {
//		checkIndex = i & (FIFO_SIZE - 1);
//		if ((RecFIFO[checkIndex] == STARTFLAG)) {
//			startFlagIndex = checkIndex;
//			eraseStart = startFlagIndex;
//			startFound = 1;
//			firstTime = 0;
//			break;
//		}
//	}
//
//	uint8_t HeaderControlByte = USBtoCAN.usbData[1];
//	if ((((0x07) & HeaderControlByte) + 1) > 2) {
//		asm(" nop");
//	}
//
//	if (startFound) {
//		for (uint8_t i = 0; i < PACKET_MAX_SIZE; i++) {
//			checkIndex = (i + startFlagIndex) & (FIFO_SIZE - 1);
//			USBtoCAN.usbData[i] = RecFIFO[checkIndex];
//			if (RecFIFO[checkIndex] == ENDFLAG) {
//				eraseEnd = checkIndex;
//				startFlagIndex += i + 1;
//				packetReady = 1;
//				break;
//			}
//		}
//	}
//
//	if (startFound) {
//		for (int i = eraseStart; i <= eraseEnd; i++) {
//			RecFIFO[i] = 0;
//		}
//	}
//	startFound = 0;
}

void DecodeHeader() {
//	USBtoCAN.CanRecID = USBtoCAN.usbData[2];
	uint8_t SendID = USBtoCAN.usbData[2];
	uint8_t HeaderControlByte = USBtoCAN.usbData[1];
	USBtoCAN.DataLength = ((0x07) & HeaderControlByte) + 1;
	USBtoCAN.checksum16 = ((0x78) & HeaderControlByte) >> 3;
	USBtoCAN.usbData[1] = ~(0x78) & USBtoCAN.usbData[1];

	if (USBtoCAN.DataLength > 2) {
		asm(" nop");
	}

	if (HeaderControlByte & BIT7) {
		USBtoCAN.R_nT = RECEIVE;
	} else {
		USBtoCAN.R_nT = TRANSMIT;
	}

	checksumCheck = 0;
	if (USBtoCAN.R_nT == TRANSMIT) {
		for (int i = 0; i < USBtoCAN.DataLength + 4; i++) {
			checksumCheck += USBtoCAN.usbData[i];
		}
	} else {
		for (int i = 0; i < 4; i++) {
			checksumCheck += USBtoCAN.usbData[i];
		}
	}
	checksumCheck = checksumCheck % 16;
	if (checksumCheck == USBtoCAN.checksum16) {
		if ((USBtoCAN.R_nT == TRANSMIT)) {
			TxHeader.StdId = SendID;	//changing send ID
			TxHeader.DLC = USBtoCAN.DataLength;	//changing send length
		}

		// Increment the startByteIndex to after the endByteIndex for the next packet check
		startByteIndex = ((endByteIndex + 1) & (FIFO_SIZE - 1));
		// Since the checksum passed, we are done with this data and can erase it
		for (int i = 0; i <= (packetDataLength + 3); i++) {
			RecFIFO[(startByteIndex + i) & (FIFO_SIZE - 1)] = 0x00;
		}

		// TODO: Remove this debug variable
		debug_packetFoundCheck_count += 1;
//		else {
//			USBtoCAN.RecLength = USBtoCAN.DataLength;
//			if (lastID != USBtoCAN.CanRecID) {
//				canSetup(USBtoCAN.CanRecID, USBtoCAN.RecLength);
//				lastID = USBtoCAN.CanRecID;
//				canReceived = 0;
//			}
//		}
	} else {
		USBtoCAN.R_nT = RESET;

		//If the checksum failed, increment the startByteIndex by one
		RecFIFO[startByteIndex] = 0x00;
		startByteIndex = ((startByteIndex + 1) & (FIFO_SIZE - 1));
	}
}

void SendUsbDataToP1_CAN() {
	for (int i = 0; i < USBtoCAN.DataLength; i++) {
		USBtoCAN.canData[i] = USBtoCAN.usbData[i + 3];
	}
	CANsend();
	//memset(USBtoCAN.usbData, 0, sizeof(USBtoCAN.usbData));
}

void RecCanDataFromP1() {
	CANreceive();	//USBtoCAN.Data <- Can Receive Buffer
}

void SendCanDataToP0_USB() {
	checksum = 0;
	USBtoCAN.canData[0] = STARTFLAG;
	USBtoCAN.canData[1] = USBtoCAN.CanRecID;
	USBtoCAN.canData[USBtoCAN.RecLength + 3] = ENDFLAG;
	for (int i = 0; i < USBtoCAN.RecLength + 4; i++) {
		if (i != USBtoCAN.RecLength + 2) {
			checksum += USBtoCAN.canData[i];
		}
	}
	checksum = checksum % 16;
	USBtoCAN.canData[USBtoCAN.RecLength + 2] = checksum;
	USBtoCAN.UsbSend(USBtoCAN.RecLength + 4);
	//memset(USBtoCAN.usbData, 0, sizeof(USBtoCAN.usbData));

	// TODO: Remove this debug variable
	debug_canRXCheck_count += 1;
}

void USBsend(uint8_t size) {
	HAL_UART_Transmit_IT(&huart3, USBtoCAN.canData, size);
}

void USBreceive(uint8_t size) {
	HAL_UART_Receive_IT(&huart3, UartRecBuffer, size);
	asm(" nop");

}

void CANsend() {
	while (HAL_CAN_AddTxMessage(&hcan, &TxHeader, USBtoCAN.canData, &TxMailbox)
			!= HAL_OK)
		;

	// TODO: Remove this debug variable
	debug_canTX_count += 1;
}

void CANreceive() {
	USBtoCAN.CanRecID = RxHeader.StdId;
	USBtoCAN.RecLength = RxHeader.DLC;
	uint8_t start = 2;
	for (int i = start; i < USBtoCAN.RecLength + start; i++) {
		USBtoCAN.canData[i] = RxData[i - start];
	}
	asm(" nop");

	// TODO: Remove this debug variable
	debug_canRX_count += 1;
}

/****************************************************************************************/

