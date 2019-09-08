/*
 * Name: TKB_CAN_Test_Bench
 * Author: Marquez Jones
 * Desc: Node designed to transmit different CAN messages
 *       to TKB processor
 *
 * Hardware Notes:
 *                 CAN:
 *                 Demo uses CAN0 on Port B
 *                 PB4 - CANRX
 *                 PB5 - CANTX
 *                 CAN must have termination resistors(120 Ohms) on each node
 */

//includes
#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_can.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/can.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/timer.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"

//MIL Includes
#include "MIL_CAN.h"
#include "MIL_CLK.h"
//#include "MIL_UART.h"

//the current ID that's considered motherboard
//emu = emulated
#define EMU_MOBOID 0x20

/********************************************STRUCTS******************************/

typedef union{

    uint8_t array[4];  //received data via CAN
    float   speed_float;     //the usable float value

}tkb_speed_data_t;


/********************************************FUNC PROTOTYPES******************************/

/*
 * Desc: Timer configured to trigger a periodic interrupt
 *       every 1 second
 */
void InitTimer0(void);

/********************************************ISR PROTOTYPES******************************/

/*
 * Desc: ISR to be triggerd by timer0 overflow
 *       will toggle a flag to be used in main
 */
void Timer0ISR(void);

/********************************************GLOBAL DATA******************************/

//if 1, send message
uint8_t timer0_txflag = 0x00;

//selects message to be sent
uint8_t timer0_msgsel = 0x00;



/*********************************************MAIN**********************************/

int main(void){

    //set system clock to 16MHZ
    MIL_ClkSetInt_16MHz();

    /************CAN INIT START***************/

    //enable gpio clocks for CAN
    /*
     * Check MIL_CAN.h for when this function
     * should be used
     */
    MIL_CANPortClkEnable(MIL_CAN_PORT_B);

    MIL_InitCAN(MIL_CAN_PORT_B,CAN0_BASE);

    //initalize reception object
    tkb_speed_data_t test;
    test.speed_float = 0.5;

    uint8_t thrust_cmd[6] = {0x54,0,test.array[0],test.array[1],test.array[2],test.array[3]};

    //pointer to address variable in thrust packet
    uint8_t *ptest_addr;
    ptest_addr = &thrust_cmd[1];
    thrust_cmd[1] = 0;

    tCANMsgObject CANMsgObj;
    CANMsgObj.ui32MsgLen = 6;

    char blue[] = "BLUE";
    CANMsgObj.pui8MsgData = blue;

    /*
     * Object used to tranmit messagesr
     */
    CANMsgObj.ui32MsgID = EMU_MOBOID;
    CANMsgObj.ui32MsgIDMask = 0;


    /************CAN INIT E1ND***************/

    InitTimer0();

    IntMasterEnable();

    /*
     * While loop logic:
     * Every time the timer interrupts,
     * a CAN message will be sent
     *
     * The code toggles between blue and red
     * every time the timer overflows
     */

    while(1){

        timer0_txflag = 1;
        if(timer0_txflag){


            //transmit using CAN object 1


            CANMessageSet(CAN0_BASE, 1, &CANMsgObj, MSG_OBJ_TYPE_TX);
            timer0_txflag = 0;
            (*ptest_addr)++;
            if(*ptest_addr >= 7){

                *ptest_addr = 0;

            }


          }




    }

}


/**************************************FUNCTION DEFINITIONS********************************************/

/*
 * Desc: Timer configured to trigger a periodic interrupt
 *       every 1 second
 */
void InitTimer0(void){

        //We will be dividing the system clock by 255
       uint8_t prescalar = 0xFF;

       //enable peripheral clock
       /*
        * A feature of ARM processors in general is that in order to use any
        * given peripheral(UART,Timers,etc.) you must enable their clocks
        */
       SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);

       //wait for peripheral clock to stabalize
       while(!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER0));

       //configure timer for periodic count
       TimerConfigure(TIMER0_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_B_PERIODIC);

       //Set the prescalar to divide system clock
       TimerPrescaleSet(TIMER0_BASE, TIMER_B, prescalar);

       //set timer to interrupt every 1 second
       /*
        * The equation to determine timer period is
        * (Time in seconds) * (System Clock)/(Clock Prescalar)
        */
       TimerLoadSet(TIMER0_BASE, TIMER_B, 1* SysCtlClockGet()/prescalar);

       //enable interrupt triggered on timeout
       TimerIntEnable(TIMER0_BASE, TIMER_TIMB_TIMEOUT);

       //set ISR function
       /*
        * One of the features of the TI library is that the use can
        * pass a function pointer into this function to set their
        * own interrupt handler
        *
        * In this case I passed Timer0IntBlink into this function to
        * set it as my interrupt service routine
        */
       TimerIntRegister(TIMER0_BASE, TIMER_B, &Timer0ISR);

       //enable timer interrupts from CPU perspective
       IntEnable(INT_TIMER0B);

       //enable timer
       TimerEnable(TIMER0_BASE, TIMER_B);


}

/********************************************ISR DEFINITIONS******************************/

/*
 * Desc: ISR to be triggerd by timer0 overflow
 *       a toggle a flag to toggle which message is sent
 *       a tx flag will transmit a CAN message
 */
void Timer0ISR(void){

    //Clear the interrupt flag
    TimerIntClear(TIMER0_BASE, TIMER_TIMB_TIMEOUT);

    timer0_txflag = 0xFF;
    timer0_msgsel ^= 0xFF;

}






