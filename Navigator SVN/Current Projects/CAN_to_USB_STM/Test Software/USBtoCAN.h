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
#include <iostream>
#include <bitset>
#include <stdlib.h>
#include <ctype.h>
using namespace std;
/*******************/
//#include "stm32f0xx_hal.h"

#define STARTFLAG 0xC0
#define ENDFLAG 0xC1
#define ESCAPEFLAG 0x7D

#define DEFAULT_DATA_LENGTH 16
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
    uint8_t Data[8];
    uint8_t DataLength;
    uint8_t CanFilterId;

    bool R_nT;
    bool Valid;

    void (*UsbSend)(uint8_t data);
    uint8_t (*UsbReceive)(void);

    void (*CanSend)(void);
    void (*CanReceive)(void);
} USBtoCAN_t;

void InitUSBtoCAN();
void USBsend(uint8_t data);
uint8_t USBreceive();
void DecodeHeader();
void CANsend();
void CANreceive();
void USBtoCAN_RUN();
void RecUsbDataFromP0();
void SendUsbDataToP1();
void RecCanDataFromP1();
void SendCanDataToP0();
void CanSetup(uint8_t CanFiltId, uint8_t DataLength);
#endif /* USBTOCAN_H_ */
