/*
 * Name: Thruster_Kill_Board.h
 * Author: Marquez Jones
 * Date Updated: 3/22/19
 * Desc: functions used for thruster kill board
 * NOTE: WHEN READING THIS CODE THE MEANING OF KILL
 *       CHANGED WHILE WRITING CODE. PLEASE REFER
 *       TO CORRESPONDING COMMENTS FOR CLARITY
 * Terminology:
 * TKB - Thruster Kill Board (functions written for this project found in
 *                            Thurster_Kill_Board.h and .c)
 *
 * SCHEMATIC CONVENTIONS FROM FRANK
 * F - FRONT
 * B - BACK
 * R - RIGHT
 * L - LEFT
 * H - HORIZONTAL
 * V - VERTICAL
 *
 * THRUSTER PIN MAPPING:
 *  ALL PWM MODULE 0
 *  FH   Gen1   L-PWM3(B5)   R-PWM2(B4)
 *  FV   Gen2   L-PWM4(E4)   R-PWM5(E5)
 *  BH   Gen0   L-PWM0(B6)   R-PWM1(B7)
 *  BV   Gen3   L-PWM7(C5)   R-PWM6(C4)
 *
 *  CAN NOTES: This ECU will filter for two separate CAN Task Groups
 *             it will look for either thruster updates from motherboard
 *             or KILL statuses from the KILL task group
 *
 * INTERRUPTS NOTE: the only interrupts currently are
 *                  the the Hall IO interrupts.
 *
 *                  CAN will currently be implemented by
 *                  polling
 *
 * CAN NOTES:
 * TX - PA1
 * RX - PA0
 *
 * DO WHILES IN DEFINES EXPLAINED:
 * it's just a syntax thing to make multi line defines works
 * I tried two different methods and the do while(0) worked.
 * I know it's kind of esoteric but it works.
 *
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
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "driverlib/pwm.h"


#include "MIL_BR_ESC.h" //ESC header
#include "MIL_CLK.h"
#include "MIL_CAN.h"

#ifndef THRUSTER_KILL_BOARD_H_
#define THRUSTER_KILL_BOARD_H_

//Delay macros
#define SEC1 16000000
#define MS_100_DELAY() SysCtlDelay((SEC1/30))
#define SEC_1_DELAY()  SysCtlDelay((SEC1/3))
#define SEC_15_DELAY() SysCtlDelay(15*(SEC1/3))
#define SEC_30_DELAY()  SysCtlDelay(30*(SEC1/3))

/**********UART START*****************/

//Uart init
#define TKB_InitUART (UART0_BASE,MIL_DEFAULT_BAUD_115K)

/**********UART END*****************/


/***********CAN START******************/
/*
 * ID NOTES: MIL CAN IDs will be restricted to
 *           8 bits.
 *
 *           ID FILT:
 *           With regards to the FILTID_bm, a
 *           bit being 1 means that bit in the
 *           ID matters. Likewise, if there's a
 *           0, that bit is a don't care and won't
 *           be looked at in filtering.
 */

//CAN init
#define TKB_CANPortEn()    MIL_CANPortClkEnable(MIL_CAN_PORT_A)
#define TKB_CANInit()      MIL_InitCAN(MIL_CAN_PORT_A,CAN1_BASE)
#define TKB_CAN_TX(pMsg)   MIL_CANSimpleTX(TKB_CANID,pMsg,8,CAN1_BASE)

//CAN defines
//task group 5 ,ECU 2
#define TKB_CANID 0x52 //device ID
#define TKB_CAN_BASE CAN1_BASE

//task group 0, ECU 0
#define TKB_KILLID 0x00    //from any source
#define TKB_UNKILLID TKE_MOBOID  //unkill signal from motherboard
#define TKB_KILL_FILTID_bm 0xF0
#define TKB_UNKILL_FILTID_bm 0xFF //in case of waiting for an unkill, we'll care about all bits
#define TKB_CAN_KILL_LEN 2 //1 byte for confirm kill, 1 byte for source code
#define TKB_CAN_KILL_OBJ 1

//MOBO Thrust messages will be considered
//task group 2, ECU 0
/*
 * KILLS CAN ALSO COME STRAIGHT FROM MOTHERBOARD
 * SEE TKB_KILL_ADDR which defines what ID
 * motherboard needs to write to in order to
 * kill the thruster
 */
#define TKB_MOBOID 0x20
#define TKB_MOBO_FILTID_bm 0xFF
#define TKB_CAN_MOBO_LEN 8 // r/w , address ,float
#define TKB_CAN_MOBO_OBJ 2

/***********CAN END********************/


/**************THRUSTER START***************************/

#define NUM_THRUSTERS 8

/*
 * THRUSTER PIN MAPPING(FROM FRANK'S SCHEMATIC:
 *  ALL PWM MODULE 0
 *  FH   Gen1   L-PWM3(B5)   R-PWM2(B4)
 *  FV   Gen2   L-PWM4(E4)   R-PWM5(E5)
 *  BH   Gen0   L-PWM0(B6)   R-PWM1(B7)
 *  BV   Gen3   L-PWM7(C5)   R-PWM6(C4)
 */

#define TKB_PWM_BASE   PWM0_BASE  //module zero
#define TKB_FH_PWM_GEN PWM_GEN_1
#define TKB_FV_PWM_GEN PWM_GEN_2
#define TKB_BH_PWM_GEN PWM_GEN_0
#define TKB_BV_PWM_GEN PWM_GEN_3

//from schematic cross referenced with signals by signal name table
//in TM4C123GH6PM manual
#define TKB_PWM_FHL_PIN PWM_OUT_3
#define TKB_PWM_FHR_PIN PWM_OUT_2
#define TKB_PWM_FVL_PIN PWM_OUT_4
#define TKB_PWM_FVR_PIN PWM_OUT_5
#define TKB_PWM_BHL_PIN PWM_OUT_0
#define TKB_PWM_BHR_PIN PWM_OUT_1
#define TKB_PWM_BVL_PIN PWM_OUT_7
#define TKB_PWM_BVR_PIN PWM_OUT_6

//Write addresses for all mobo data
//this is to avoid confusion with
//CAN IDs
#define TKB_PWM_FHL_ADDR 0
#define TKB_PWM_FHR_ADDR 1
#define TKB_PWM_FVL_ADDR 2
#define TKB_PWM_FVR_ADDR 3
#define TKB_PWM_BHL_ADDR 4
#define TKB_PWM_BHR_ADDR 5
#define TKB_PWM_BVL_ADDR 6
#define TKB_PWM_BVR_ADDR 7
#define TKB_KILL_ADDR  0x4B //ascii 'K'

#define TKB_PWM_OUT_EN()  PWMOutputState(TKB_PWM_BASE, 0xFF,true)
#define TKB_PWM_OUT_DIS() PWMOutputState(TKB_PWM_BASE, 0xFF,false)

//Struct used for converting received thrust data into
//a float
typedef union{

    uint8_t array[4];  //received data via CAN
    float   speed_float;     //the usable float value

}tkb_speed_data_t;

//Data extracted from the CAN message
//
typedef struct{

    uint8_t thrust_addr;        //we have 8 thrusters
    uint32_t pwm_gen;
    uint32_t pwm_out;
    tkb_speed_data_t speed;

}tkb_thrust_data_t;

/*
 * Desc: Initializes PWM for ESC communications
 *       in this I:
 *       -Init PWM0
 *       -Init All generators(3 to 0)
 *       -Configure all associated pins
 *        or alt function PWM
 *       -Configure PWM up_down mode
 *       -Set PWM to ESC protocol period of 2000us
 *       -enable all generators
 *
 *       -DOES NOT SET PWM OUTPUT TO TRUE
 */
void TKB_PWM0_Init(void);


/*
 * Desc: Initializes communication with ESC by sending
 *       the stop pulse
 *
 *       ALSO SETS PWM OUTPUT TO TRUE
 */
void TKB_Init_ESC(void);

/*
 * Desc: Executes thruster kill sequence
 *
 *       SEQUENCE:
 *       SEND STOP TO THRUSTERS
 *       KILL POWER TO THRUSTERS
 *       TRANSMIT HARD KILL TO MOBO
 *       KILL POWER TO MAIN
 */
void TKB_HardKill(void);

/*
 * Desc: unkill sequence from hard kill
 *       POWER MAIN
 *       POWER THRUSTERS
 *       INIT ESC
 */
void TKB_UnKill(void);

/*
 * Desc: Executes thruster kill sequence
 *
 *       SEQUENCE:
 *       SEND STOP TO THRUSTERS
 *       KILL POWER TO THRUSTERS
 */
void TKB_SoftKill(void);

/*
 * Desc: Executes idle
 *
 *       SEQUENCE:
 *       SEND STOP TO THRUSTERS
 */
void TKB_IdleThrusters(void);

/*
 * Desc: sends stop commands to all thrusters
 *
 * Assumes: PWM is initialized
 */
void TKB_StopAllThrust(void);

/*
 * Desc: will read in struct data and set thruster to the speed
 *       specified in speed variable
 *
 * Parameters:
 * thruster - your struct containg thruster data
 *
 * Assumes: Assumes PWM and ESCs are intialized
 */
void TKB_PWM_SetSpeed(tkb_thrust_data_t thruster);

/**************THRUSTER END***************************/

/**************PROTOCOL END***************************/

//INDEXES
#define MSG_TYPE_IDX 0 //thrust or kill
#define MSG_CR_IDX 1   //if it's command or response
#define KILL_TYPE_IDX 2 //if it's soft or hard kill
#define MSG_HS_IDX 2    //if it's hard or soft kill(redundant define) same as KILL_TYPE_IDX
#define THRUST_FLOAT_START 2
#define THRUST_FLOAT_END 5
#define MSG_UA_IDX 3   //if it's assertion or unassertion
#define THRUST_ID_IDX 1
#define THRUST_FLOAT_START 2
#define THRUST_FLOAT_END 5

//BYTE DEFINES
/*CMD/Response(CR) Byte*/
#define CMD_BYTE  0x43
#define RESP_BYTE 0x52
/*Hard/Soft(HS) Kill Byte*/
#define SOFT_BYTE 0x53
#define HARD_BYTE 0x48
/*Unassert/Assert(UA) Byte */
#define U_BYTE 0x55 //ASCII: 'U' UNASSERT
#define A_BYTE 0x41 //ASCII: 'A' ASSERT
/*Message Type Byte*/
#define KILL_START_BYTE 0x4B //ASCII: 'K'
#define THRUST_START_BYTE 0x54 //ASCII: 'T'

/*
 * Desc: checks if the message is a kill message
 *       will return 1 if kill, 0 otherwise
 */
uint8_t TKB_Check_KillMsg(uint8_t *pMsg);

/*
 * Desc: checks if the message is a thruster message
 *       will return 1 if thruster, 0 otherwise
 */
uint8_t TKB_Check_ThrustMsg(uint8_t *pMsg);

/*
 * Desc: Returns CR byte
 * Parameter: pointer to CAN message
 */
uint8_t TKB_Get_CRByte(uint8_t *pMsg);

/*
 * Desc: Returns HS byte
 * Parameter: Pointer to CAN message
 */
uint8_t TKB_Get_HSByte(uint8_t *pMsg);

/*
 * Desc: Returns UA byte
 * Parameter: pointer to CAN message
 */
uint8_t TKB_Get_UAByte(uint8_t *pMsg);



/**************PROTOCOL START**************************/

/**************IO START*******************************/

//This board uses literally every port on the MCU
#define INIT_ALL_PORT_CLKS() do{ \
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA); \
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB); \
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC); \
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD); \
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE); \
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF); \
        }while(0)

#define KILL_PIN GPIO_PIN_3 //ON PORT F
#define DBG_LED_PIN GPIO_PIN_0 //ON PORT D
/*
 * USES PIN ORIGINALLY DESIGNED
 * AS UART TX
 */
#define MAIN_PWR_PIN GPIO_PIN_5//ON PORT D

//initialize Kill pin
//default pin to 0 to kill power to thrusters
#define INIT_THRUST_PWR_IO() do{ \
        GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE,KILL_PIN);\
        GPIOPinWrite(GPIO_PORTF_BASE, KILL_PIN, 0x00);\
        }while(0)

/*
 * ATTACHED TO UART PINS
 * THIS WAS IMPLEMENTED TO
 * SEPARATE THRUSTER AND MAIN POWER
 * IF A HARD KILL OCCURS
 * THIS POWER WILL BE KILLED
 */
#define INIT_MAIN_PWR_IO() do{ \
        GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE,MAIN_PWR_PIN);\
        GPIOPinWrite(GPIO_PORTD_BASE, MAIN_PWR_PIN, 0xFF);\
        }while(0);

#define KILL_THRUSTERS()  GPIOPinWrite(GPIO_PORTF_BASE, KILL_PIN, 0x00)
#define POWER_THRUSTERS() GPIOPinWrite(GPIO_PORTF_BASE, KILL_PIN, 0xFF)

#define KILL_MAIN() GPIOPinWrite(GPIO_PORTD_BASE, MAIN_PWR_PIN, 0x00)
#define POWER_MAIN() GPIOPinWrite(GPIO_PORTD_BASE, MAIN_PWR_PIN, 0xFF)

//initialize Debug LED
//defaulted to on
#define INIT_DBG_IO()  do{\
        GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE,DBG_LED_PIN);\
        GPIOPinWrite(GPIO_PORTD_BASE, DBG_LED_PIN, 0xFF);\
        }while(0)

#define DBG_LED_ON()   GPIOPinWrite(GPIO_PORTD_BASE, DBG_LED_PIN, 0xFF)
#define DBG_LED_OFF()  GPIOPinWrite(GPIO_PORTD_BASE, DBG_LED_PIN, 0x00)

/**************IO END*********************************/

/*************TIMER START*****************************/

/*
 * Desc: Initialize timer to trigger overflow ISR
 *
 * Purpose: will check the Soft kill hall effect sensor
 */
void Timer0_OVF_Init(void (*pISR)(void), float period_ms);


/*
 * Desc: Initialize timer to trigger overflow ISR
 *
 * Purpose: Will check both go and if thrusters should idle
 */
void Timer1_OVF_Init(void (*pISR)(void), float period_ms);

/*************TIMER END*******************************/

/**************HALL EFFECT START**********************/

/*
 * NOTE: NAMING CONVENTIONS DO NOT FOLLOW
 *       THE SCHEMATIC.
 *
 *       THE PIN NAMES IN THE SOFTWARE
 *       ARE AKIN TO TEAM TERMINOLOGY
 *       AS OPPOSED TO THE HARDWARE TERMS
 *
 *       THIS IS THE SCHEMATIC TO FIRMWARE PIN MAP:
 *       FIRMARE         SCHEMATIC             PIN
 *       HALL_SOFTKILL -  HALL_SOFTKILL        :PB1
 *       HALL_GO       -  HALL_KILL_ENABLE     :PB2
 *       HALL_ON_OFF   -  HALL_HARDKILL        :PB0
 */
//ALL ON PORT B
#define HALL_SOFTKILL_PIN GPIO_PIN_1
#define HALL_GO_PIN       GPIO_PIN_2
#define HALL_ON_OFF_PIN   GPIO_PIN_0

//activation levels
//IN REFERENCE TO THERE BEING A MAGNET
#define HALL_ACT_LO 0
#define HALL_ACT_HI 1

#define HALL_LO 0 //returned if low
#define HALL_HI 1 //returned if high

/*
 * Desc: Initialize All HAll sensor pins to input
 * Parameters: Pointer to ISR(for kill handling)
 *
 * NOTE: Interrupt is only attached to the ON_OFF
 *       signal. All other pins will be checked
 *       via other means
 */
void Init_HALL_IO(uint8_t activation_lvl);

/*
 * Name: Check Hall functions
 * Desc: Each of these functions will
 *       output 0xFF is the intpus are
 *       high and 0x00 if the input
 *       is low
 */
uint8_t HALL_CheckGo(void);
uint8_t HALL_CheckSoftKill(void);
uint8_t HALL_Check_ON_OFF(void);


/**************HALL EFFECT END************************/

#endif /* THRUSTER_KILL_BOARD_H_ */
