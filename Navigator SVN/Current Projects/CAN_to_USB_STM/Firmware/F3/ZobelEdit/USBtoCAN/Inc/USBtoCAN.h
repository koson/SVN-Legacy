/*
 * p2p.h
 *
 *  Created on: Sep 25, 2018
 *      Author: rosem
 */

#ifndef USBTOCAN_H_
#define USBTOCAN_H_

#include <stdint.h>
#include <stdbool.h>
/******Test Includes****/
//#include <iostream>
//#include <bitset>
//#include <stdlib.h>
//#include <ctype.h>
//using namespace std;
/*******************/
#include "stm32f3xx_hal.h"

#define STARTFLAG 0xC0
#define ENDFLAG 0xC1
#define STARTFLAG_USB_DATA 0xC2
#define ENDFLAG_USB_DATA 0xC3
#define ESCAPEFLAG 0x7D
#define SETUP_SIZE 4
#define BUFFER_SIZE 16
#define RECEIVE 1
#define TRANSMIT 2
#define FIFO_SIZE 0x1000
#define PACKET_MAX_SIZE 12

#define DEFAULT_DATA_LENGTH 8
#define DEFAULT_FILT_ID 0x02

#define BIT0 (0x01 << 0)
#define BIT1 (0x01 << 1)
#define BIT2 (0x01 << 2)
#define BIT3 (0x01 << 3)
#define BIT4 (0x01 << 4)
#define BIT5 (0x01 << 5)
#define BIT6 (0x01 << 6)
#define BIT7 (0x01 << 7)


//Point to point protocol
typedef struct USBtoCAN_t {
	uint8_t usbData[BUFFER_SIZE];
	uint8_t canData[BUFFER_SIZE];
    uint8_t DataLength;
    uint8_t RecLength;
    uint8_t CanRecID;
    uint8_t checksum16;

    uint R_nT;
    bool Valid;

    void (*UsbSend)(uint8_t size);
    void (*UsbReceive)(uint8_t size);

    void (*CanSend)(void);
    void (*CanReceive)(void);
} USBtoCAN_t;

void InitUSBtoCAN();
void USBsend(uint8_t size);
void USBreceive(uint8_t size);
void DecodeHeader();
void CANsend();
void CANreceive();
void USBtoCAN_RUN();
void RecUsbDataFromP0();
void SendUsbDataToP1_CAN();
void RecCanDataFromP1();
void SendCanDataToP0_USB();
void CanSetup(uint8_t CanFiltId, uint8_t DataLength);
#endif /* USBTOCAN_H_ */
