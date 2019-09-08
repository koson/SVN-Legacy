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

/*Test Buffers*/
uint8_t moboBuffer[DEFAULT_DATA_LENGTH];
uint8_t sensorsBuffer[DEFAULT_DATA_LENGTH];

void InitUSBtoCAN() {
	USBtoCAN.UsbSend = &USBsend;
	USBtoCAN.UsbReceive = &USBreceive;
	USBtoCAN.CanSend = &CANsend;
	USBtoCAN.CanReceive = &CANreceive;
	USBtoCAN.DataLength = 0;
}

/**************************|USBtoCAN State Machine Functions|**************************/

void USBtoCAN_RUN() {
	//cin << endl << "/***************Waiting for start flag....***************/" << endl;
	USBtoCAN.UsbReceive(SETUP_SIZE);

	if ((USBtoCAN.usbData[0] == STARTFLAG)
			&& (USBtoCAN.usbData[SETUP_SIZE - 1] == ENDFLAG)) {
		//cout << "Start Flag Entered!" << endl;
		DecodeHeader();
		asm(" nop");
	}

	if (USBtoCAN.R_nT) {
		RecCanDataFromP1();
		SendCanDataToP0_USB();
	} else {
		RecUsbDataFromP0();
		if (USBtoCAN.Valid) {
			SendUsbDataToP1();
		}
	}
}

void DecodeHeader() {
	//cout << endl << "Decoding Header......." << endl;
	USBtoCAN.CanFilterId = USBtoCAN.usbData[2];
	uint8_t HeaderControlByte = USBtoCAN.usbData[1];
	USBtoCAN.R_nT = HeaderControlByte & BIT7;
	USBtoCAN.DataLength = HeaderControlByte
			& (BIT0 | BIT1 | BIT2 | BIT3 | BIT4 | BIT5 | BIT6);
	asm(" nop");
	canSetup(USBtoCAN.CanFilterId, USBtoCAN.DataLength+2);
	//cout << endl << "Header: " << endl;
	//cout << "Receive_notTransmit: " << (int)USBtoCAN.R_nT << endl;
	//cout << "Data Length: " << (int)USBtoCAN.DataLength << endl;
	//cout << "Desired CAN ID: " << (int)USBtoCAN.CanFilterId << endl;
}

void RecUsbDataFromP0() {
	//cout << endl << "/***************Receiving USB Data from Mobo....***************/" << endl;
	//cout << "Data Length: " << (int)USBtoCAN.DataLength << endl;
	USBtoCAN.Valid = 1;
	for (int i = 0; i < USBtoCAN.DataLength; i++) {
		USBtoCAN.usbData[i] = USBtoCAN.UsbReceive(0);
	}
	uint8_t USBdata = USBtoCAN.UsbReceive(0);
	if (USBdata != ENDFLAG) {
		USBtoCAN.Valid = 0;
		//cout << "ERROR: End Flag not received!" << endl;
	}
}

void SendUsbDataToP1() {
	//cout << endl << "/***************Sending USB Data to Sensors...***************/" << endl;
	for (int i = 0; i < USBtoCAN.DataLength; i++) {
		USBtoCAN.UsbSend(USBtoCAN.DataLength+2);
	}
}

void RecCanDataFromP1() {
	//cout << endl << "/***************Receiving CAN Data From Network...***************/" << endl;
	CANreceive();	//USBtoCAN.Data <- Can Receive Buffer
}

void SendCanDataToP0_USB() {
	//cout << endl << "/***************Sending CAN Data to Mobo...***************/" << endl;
	USBtoCAN.canData[0] = STARTFLAG;
	USBtoCAN.canData[USBtoCAN.DataLength+1] = ENDFLAG;
	USBtoCAN.UsbSend(USBtoCAN.DataLength+2);
}

/**************************************************************************************/

/*********************|USBtoCAN Send/Receive Wrapper Functions|************************/

/* USB Send/Receive Notes:
 * HAL library functions accept data pointers
 * for recption
 *
 *
 */

void USBsend(uint8_t size) {

	// HAL libary receives data pointer as input
	// data size set to 1
	asm(" nop");
	HAL_UART_Transmit(&huart3, USBtoCAN.canData, size, 100);	//timeout set to 100

	/************ Test Function **************/
	//cout << "USB Data Out: " << hex << (int)data << endl;
	/*****************************************/

}

uint8_t USBreceive(uint8_t size) {

	// HAL libary receives data pointer as input
	// data size set to 1
	// timeout set constant to 20

	HAL_UART_Receive(&huart3, USBtoCAN.usbData, size, 100); //timeout set to 100

	/************* Test Function **************/
	//uint16_t num = 0;
	//cout << "USB Data In: ";
	//cin >> hex >> num;
	//return num;
	/******************************************/

}

void CANsend() {
	HAL_CAN_AddTxMessage(&hcan, &TxHeader, USBtoCAN.canData, &TxMailbox);
	/************ Test Function **************/
	//	for (int i = 0; i < USBtoCAN.DataLength; i++) {
	//		sensorsBuffer[i] = USBtoCAN.Data[i];
	//	}
	//	//cout << "CAN Data Out: " << endl;
	//	for (int i = 0; i < USBtoCAN.DataLength; i++) {
	//		//cout << sensorsBuffer[i] << " ";
	//	}
	//cout << endl;
	/*****************************************/
}

void CANreceive() {
	for (int i = 1; i < USBtoCAN.DataLength+1; i++) {
		USBtoCAN.canData[i] = RxData[i-1];
	}
	asm(" nop");
	/************* Test Function **************/
	//uint16_t num = 0;
	//cout << "Data Length: " << (int)USBtoCAN.DataLength << endl;
	//	for (int i = 0; i < USBtoCAN.DataLength; i++) {
	//		//cout << "CAN Data In: " << endl;
	//		//cin >> hex >> num;
	//		moboBuffer[i] = num;
	//		USBtoCAN.Data[i] = moboBuffer[i];
	//	}
	/******************************************/
}

/****************************************************************************************/

