/*
 * USBtoCAN.c
 *
 *  Created on: Sep 26, 2018
 *      Author: rosem
 */

#include <USBtoCAN.h>

USBtoCAN_t USBtoCAN;

//UART_HandleTypeDef huart2;
//UART_HandleTypeDef *huart2_ptr = &huart2;
//
//CAN_HandleTypeDef hcan;
//CanTxMsgTypeDef TxMessage;
//CanRxMsgTypeDef RxMessage;
//CAN_FilterConfTypeDef rxFilter;

/*Test Buffers*/
uint8_t moboBuffer[DEFAULT_DATA_LENGTH];
uint8_t sensorsBuffer[DEFAULT_DATA_LENGTH];

void InitUSBtoCAN() {
    USBtoCAN.UsbSend = &USBsend;
    USBtoCAN.UsbReceive = &USBreceive;
    USBtoCAN.CanSend = &CANsend;
    USBtoCAN.CanReceive = &CANreceive;
}

/**************************|USBtoCAN State Machine Functions|**************************/

void USBtoCAN_RUN() {
    while (1) {
        cout << endl << "/***************Waiting for start flag....***************/" << endl;
        if (USBtoCAN.UsbReceive() == STARTFLAG) {
            cout << "Start Flag Entered!" << endl;
            DecodeHeader();
            if (USBtoCAN.R_nT) {
                RecCanDataFromP1();
                SendCanDataToP0();
            } else {
                RecUsbDataFromP0();
                if (USBtoCAN.Valid) {
                    SendUsbDataToP1();
                }
            }
        }
    }
}

void DecodeHeader() {
    cout << endl << "Decoding Header......." << endl;
    USBtoCAN.CanFilterId = USBtoCAN.UsbReceive();
    uint8_t HeaderControlByte = USBtoCAN.UsbReceive();
    USBtoCAN.R_nT = HeaderControlByte & BIT4;
    USBtoCAN.DataLength = HeaderControlByte & (BIT0 | BIT1 | BIT2 | BIT3);

    cout << endl << "Header: " << endl;
    cout << "Receive_notTransmit: " << (int)USBtoCAN.R_nT << endl;
    cout << "Data Length: " << (int)USBtoCAN.DataLength << endl;
    cout << "Desired CAN ID: " << (int)USBtoCAN.CanFilterId << endl;
}

void RecUsbDataFromP0() {
    cout << endl << "/***************Receiving USB Data from Mobo....***************/" << endl;
    cout << "Data Length: " << (int)USBtoCAN.DataLength << endl;
    USBtoCAN.Valid = 1;
    for (int i = 0; i < USBtoCAN.DataLength; i++) {
        USBtoCAN.Data[i] = USBtoCAN.UsbReceive();
    }
    uint8_t USBdata = USBtoCAN.UsbReceive();
    if (USBdata != ENDFLAG) {
        USBtoCAN.Valid = 0;
        cout << "ERROR: End Flag not received!" << endl;
    }
}

void SendUsbDataToP1() {
    //CanSetup(USBtoCAN.CanFilterId, USBtoCAN.DataLength);
    cout << endl << "/***************Sending USB Data to Sensors...***************/" << endl;
    for (int i = 0; i < USBtoCAN.DataLength; i++) {
        USBtoCAN.UsbSend(USBtoCAN.Data[i]);
    }
}

void RecCanDataFromP1() {
    //CanSetup(USBtoCAN.CanFilterId, USBtoCAN.DataLength);
    cout << endl << "/***************Receiving CAN Data From Network...***************/" << endl;
    CANreceive();	//USBtoCAN.Data <- Can Receive Buffer
}

void SendCanDataToP0() {
    cout << endl << "/***************Sending CAN Data to Mobo...***************/" << endl;
    USBtoCAN.UsbSend(STARTFLAG);
    int index = 0;
    while (index < USBtoCAN.DataLength) {
        USBtoCAN.UsbSend(USBtoCAN.Data[index]);
        index++;
    }
    USBtoCAN.UsbSend(ENDFLAG);
}

/**************************************************************************************/

/*********************|USBtoCAN Send/Receive Wrapper Functions|************************/

/* USB Send/Receive Notes:
 * HAL library functions accept data pointers
 * for recption
 *
 *
 */

void USBsend(uint8_t data) {

    // HAL libary receives data pointer as input
    // data size set to 1
    // timeout set constant to 20

    /*const uint8_t timeout = 20;
     uint8_t *pData = data;
     HAL_UART_Transmit(huart2_ptr, pData, 1, timeout);*/

    /************ Test Function **************/
    cout << "USB Data Out: " << hex << (int)data << endl;
    /*****************************************/

}

uint8_t USBreceive() {

    // HAL libary receives data pointer as input
    // data size set to 1
    // timeout set constant to 20

    /*const uint8_t timeout = 20;
     uint8_t *pData;
     HAL_UART_Receive(huart2_ptr, pData, 1, timeout);
     return *pData;*/

    /************* Test Function **************/
    uint16_t num = 0;
    cout << "USB Data In: ";
    cin >> hex >> num;
    return num;
    /******************************************/

}

void CANsend() {
//	for (int i = 0; i < USBtoCAN.DataLength; i++) {
//		hcan.pTxMsg->Data[i] = USBtoCAN.Data[i];
//	}
//	HAL_CAN_Transmit(&hcan, 30);

    /************ Test Function **************/
    for (int i = 0; i < USBtoCAN.DataLength; i++) {
        sensorsBuffer[i] = USBtoCAN.Data[i];
    }
    cout << "CAN Data Out: " << endl;
    for (int i = 0; i < USBtoCAN.DataLength; i++)
    {
        cout << sensorsBuffer[i] << " ";
    }
    cout << endl;
    /*****************************************/
}

void CANreceive() {
//	HAL_CAN_Receive(&hcan, CAN_FIFO0, 30);
//	for (int i = 0; i < USBtoCAN.DataLength; i++) {
//		USBtoCAN.Data[i] = hcan.pRxMsg->Data[i];
//	}
    uint16_t num = 0;
    /************* Test Function **************/
    cout << "Data Length: " << (int)USBtoCAN.DataLength << endl;
    for (int i = 0; i < USBtoCAN.DataLength; i++) {
        cout << "CAN Data In: " << endl;
        cin >> hex >> num;
        moboBuffer[i] = num;
        USBtoCAN.Data[i] = moboBuffer[i];
    }
    /******************************************/
}

/****************************************************************************************/

