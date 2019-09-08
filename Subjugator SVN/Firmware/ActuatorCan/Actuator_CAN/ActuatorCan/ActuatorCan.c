/*
 * Name: ActuatorCan.c
 * Author: John Morin
 * Date Created: 4/20/19
 * Desc: Driving C code for the Actuator board using CAN
 */

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "MIL_CLK.h"
#include "MIL_CAN.h"

/*************************************************************************/
//Introduce Parameters

#define MOBO_ID 0x51

#define PIN0  GPIO_PIN_0 // Port D
#define PIN1  GPIO_PIN_1
#define PIN2  GPIO_PIN_2
#define PIN3  GPIO_PIN_3
#define PIN4  GPIO_PIN_4
#define PIN5  GPIO_PIN_5
#define PIN6  GPIO_PIN_6
#define PIN7  GPIO_PIN_7
#define PIN8  GPIO_PIN_0 //Port E
#define PIN9  GPIO_PIN_1
#define PIN10 GPIO_PIN_2
#define PIN11 GPIO_PIN_3

#define PORTA_CLK_ENABLE() SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA)
#define PORTD_CLK_ENABLE() SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD)
#define PORTE_CLK_ENABLE() SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE)

uint8_t VerifyMsg(uint8_t *msg);
void    InitProcess(void);
void    GPIOPinOn(uint8_t *msg);
void    GPIOPinOff(uint8_t *msg);

/*************************************************************************/
//Main method

int main(void) {

    //Initialization Process

    uint8_t msg[4]; //use blank message for actual use
    //uint8_t msg[4] = {0x41, 0x08, 0x00, 0x00}; //use dummy message for debugging
    uint8_t arr[12] = {0,0,0,0,0,0,0,0,0,0,0,0};
    MIL_CAN_MailBox_t MailBox;
    MIL_ClkSetInt_16MHz();
    PORTA_CLK_ENABLE();
    PORTD_CLK_ENABLE();
    PORTE_CLK_ENABLE();
    MIL_InitCAN(MIL_CAN_PORT_A, CAN1_BASE);
    InitProcess();

    //Mailbox defines, refer to MIL_CAN.c for more information

    MailBox.canid = MOBO_ID;
    MailBox.filt_mask = 0xF0;
    MailBox.base = CAN1_BASE;
    MailBox.msg_len = 0x04;
    MailBox.obj_num = 0x02;
    MailBox.buffer = msg;
    MIL_InitMailBox(&MailBox);

    while(1) {
        // Checks Mailbox, if there is a new message, edit the existing message to copy it
        if (MIL_CAN_CheckMail(&MailBox) == MIL_CAN_OK) {
            MIL_CAN_GetMail(&MailBox);
            if (VerifyMsg(msg)) {
                if (msg[2] == 0x01) {                         // Pin Write
                    if (msg[3] == 0x01) {                     // Turns the valve on
                        GPIOPinOn(msg);
                        arr[msg[1]] = 0x01;
                    } else if (msg[3] == 0x00) {              // Turns the valve off
                        GPIOPinOff(msg);
                        arr[msg[1]] = 0x00;
                    }
                } else if (msg[2] == 0x00) {                  // Pin Read
                    uint8_t msg_out[4] = {0x41,msg[1],arr[msg[1]],0x00
                }; // New array created to display the state of a certain pin
                    MIL_CANSimpleTX(0x53, msg_out, MailBox.msg_len, MailBox.base);
                }
            }
        }
    }
}

/*************************************************************************/
//Helper methods

uint8_t VerifyMsg(uint8_t *msg) {
    if (msg[0] == 0x41) { // ASCII 'A' chosen for Actuator Board verification
        return 1;
    } else {
        return 0;
    }
}

void InitProcess(void) {
    // Unlock Pin 7 because the hardware's a bit screwy
    HWREG(GPIO_PORTD_BASE+GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTD_BASE+GPIO_O_CR) |= GPIO_PIN_7;

    // Define all pins to their ports as outputs
    GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, PIN8 | PIN9 | PIN10 | PIN11);
    GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, PIN0 | PIN1 | PIN2  | PIN3 | PIN4 | PIN5 | PIN6 | PIN7);
}

//Writes pin specified by the message
void GPIOPinOn(uint8_t *msg) {
    switch (msg[1]) {
        case 0x00:
            GPIOPinWrite(GPIO_PORTD_BASE, PIN0,  PIN0);
            break;
        case 0x01:
            GPIOPinWrite(GPIO_PORTD_BASE, PIN1,  PIN1);
            break;
        case 0x02:
            GPIOPinWrite(GPIO_PORTD_BASE, PIN2,  PIN2);
            break;
        case 0x03:
            GPIOPinWrite(GPIO_PORTD_BASE, PIN3,  PIN3);
            break;
        case 0x04:
            GPIOPinWrite(GPIO_PORTD_BASE, PIN4,  PIN4);
            break;
        case 0x05:
            GPIOPinWrite(GPIO_PORTD_BASE, PIN5,  PIN5);
            break;
        case 0x06:
            GPIOPinWrite(GPIO_PORTD_BASE, PIN6,  PIN6);
            break;
        case 0x07:
            GPIOPinWrite(GPIO_PORTD_BASE, PIN7,  PIN7);
            break;
        case 0x08:
            GPIOPinWrite(GPIO_PORTE_BASE, PIN8,  PIN8);
            break;
        case 0x09:
            GPIOPinWrite(GPIO_PORTE_BASE, PIN9,  PIN9);
            break;
        case 0x0A:
            GPIOPinWrite(GPIO_PORTE_BASE, PIN10, PIN10);
            break;
        case 0x0B:
            GPIOPinWrite(GPIO_PORTE_BASE, PIN11, PIN11);
            break;
        default:
            break;
    }
}

//Zeros pins specified by the message
void GPIOPinOff(uint8_t *msg) {
    switch (msg[1]) {
        case 0x00:
            GPIOPinWrite(GPIO_PORTD_BASE, PIN0,  0x00);
            break;
        case 0x01:
            GPIOPinWrite(GPIO_PORTD_BASE, PIN1,  0x00);
            break;
        case 0x02:
            GPIOPinWrite(GPIO_PORTD_BASE, PIN2,  0x00);
            break;
        case 0x03:
            GPIOPinWrite(GPIO_PORTD_BASE, PIN3,  0x00);
            break;
        case 0x04:
            GPIOPinWrite(GPIO_PORTD_BASE, PIN4,  0x00);
            break;
        case 0x05:
            GPIOPinWrite(GPIO_PORTD_BASE, PIN5,  0x00);
            break;
        case 0x06:
            GPIOPinWrite(GPIO_PORTD_BASE, PIN6,  0x00);
            break;
        case 0x07:
            GPIOPinWrite(GPIO_PORTD_BASE, PIN7,  0x00);
            break;
        case 0x08:
            GPIOPinWrite(GPIO_PORTE_BASE, PIN8,  0x00);
            break;
        case 0x09:
            GPIOPinWrite(GPIO_PORTE_BASE, PIN9,  0x00);
            break;
        case 0x0A:
            GPIOPinWrite(GPIO_PORTE_BASE, PIN10, 0x00);
            break;
        case 0x0B:
            GPIOPinWrite(GPIO_PORTE_BASE, PIN11, 0x00);
            break;
        default:
            break;
    }
}
