/*
 * Name: Thruster_Kill_Board.h
 * Author: Marquez Jones
 * Date Updated: 3/22/19
 * Desc: functions used for thruster kill board
 *
 * NOTE: WHEN READING THIS CODE THE MEANING OF KILL
 *       CHANGED WHILE WRITING CODE. PLEASE REFER
 *       TO CORRESPONDING COMMENTS FOR CLARITY
 *
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
 * INTERRUPTS NOTE: Interrupt sources will be
 *                  The HALL effect sensor
 *                  and a Timer interrupt
 *
 *                  CAN will currently be implemented by
 *                  polling
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
#include "driverlib/timer.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "driverlib/pwm.h"

#include "MIL_BR_ESC.h" //ESC header
#include "MIL_CAN.h"
#include "Thruster_Kill_Board.h"

/*** CAN MESSAGES ***/
/* TX MESSAGES */
//C strings are terminated by Null character
static const uint8_t C_KILL_LEN = 5;
static const uint8_t C_GO_LEN = 3;
static const char HKS_Msg[C_KILL_LEN] = "KHS";    // 0x4B 0x48 0x53 0x00
static const char HKH_Msg[C_KILL_LEN] = "KHH";    // 0x4B 0x48 0x48 0x00
static const char SKS_Msg[C_KILL_LEN] = "KSS";    // 0x4B 0x53 0x53 0x00
static const char SKH_Msg[C_KILL_LEN] = "KSH";    // 0x4B 0x53 0x48 0x00
static const char GO_Msg[C_GO_LEN] = "GO";      // 0x47 0x00

/* RX MESSAGES */
//static const char Hard_Killed_CMD[C_KILL_LEN]   = "KCHA";    // 0x4B 0x43 0x48 0x41 0x00
//static const char Soft_Killed_CMD[C_KILL_LEN]   = "KCSA";    // 0x4B 0x43 0x53 0x41 0x00
//static const char Hard_UnKilled_CMD[C_KILL_LEN] = "KCHU";    // 0x4B 0x43 0x48 0x55 0x00
//static const char Soft_UnKilled_CMD[C_KILL_LEN] = "KCSU";    // 0x4B 0x43 0x53 0x55 0x00

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
void TKB_PWM0_Init(void){

    //PWM clock enable
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);

    //enable PWM functions on all PWM0 pins
    GPIOPinConfigure(GPIO_PB6_M0PWM0);
    GPIOPinConfigure(GPIO_PB7_M0PWM1);
    GPIOPinConfigure(GPIO_PB4_M0PWM2);
    GPIOPinConfigure(GPIO_PB5_M0PWM3);
    GPIOPinConfigure(GPIO_PE4_M0PWM4);
    GPIOPinConfigure(GPIO_PE5_M0PWM5);
    GPIOPinConfigure(GPIO_PC4_M0PWM6);
    GPIOPinConfigure(GPIO_PC5_M0PWM7);

    //Configure PF2 and PF3 as PWM
    GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_6 | GPIO_PIN_7 |
                                    GPIO_PIN_4 | GPIO_PIN_5);
    GPIOPinTypePWM(GPIO_PORTC_BASE, GPIO_PIN_4 | GPIO_PIN_5);
    GPIOPinTypePWM(GPIO_PORTE_BASE, GPIO_PIN_4 | GPIO_PIN_5);

    /*
     * Configure:
     * PWM Mod 0
     * ALL GENERATORS
     * Up down mode
     * No sync(you can sync generators together)
     */
    PWMGenConfigure(TKB_PWM_BASE,
                    TKB_FH_PWM_GEN,
                    PWM_GEN_MODE_UP_DOWN |
                    PWM_GEN_MODE_NO_SYNC);
    PWMGenConfigure(TKB_PWM_BASE,
                    TKB_FV_PWM_GEN,
                    PWM_GEN_MODE_UP_DOWN |
                    PWM_GEN_MODE_NO_SYNC);
    PWMGenConfigure(TKB_PWM_BASE,
                    TKB_BH_PWM_GEN,
                    PWM_GEN_MODE_UP_DOWN |
                    PWM_GEN_MODE_NO_SYNC);
    PWMGenConfigure(TKB_PWM_BASE,
                    TKB_BV_PWM_GEN,
                    PWM_GEN_MODE_UP_DOWN |
                    PWM_GEN_MODE_NO_SYNC);

    /* (desired period in seconds) * (clock frequency) = PWM Period */
    /*
     * This block sets each generator(all of them) to the Blue Robotics
     * designated period which is 2000us
     */
    PWMGenPeriodSet(TKB_PWM_BASE, TKB_FH_PWM_GEN, BR_ESC_PERIOD_SEC * (SysCtlClockGet()));
    PWMGenPeriodSet(TKB_PWM_BASE, TKB_FV_PWM_GEN, BR_ESC_PERIOD_SEC * (SysCtlClockGet()));
    PWMGenPeriodSet(TKB_PWM_BASE, TKB_BH_PWM_GEN, BR_ESC_PERIOD_SEC * (SysCtlClockGet()));
    PWMGenPeriodSet(TKB_PWM_BASE, TKB_BV_PWM_GEN, BR_ESC_PERIOD_SEC * (SysCtlClockGet()));

    //enable all generators on the pwm moduel
    PWMGenEnable(TKB_PWM_BASE, TKB_FH_PWM_GEN);
    PWMGenEnable(TKB_PWM_BASE, TKB_FV_PWM_GEN);
    PWMGenEnable(TKB_PWM_BASE, TKB_BH_PWM_GEN);
    PWMGenEnable(TKB_PWM_BASE, TKB_BV_PWM_GEN);

}


/*
 * Desc: Initializes communication with ESC by sending
 *       the stop pulse
 *
 *       ALSO SETS PWM OUTPUT TO TRUE
 */
void TKB_Init_ESC(void){

    //power the thrusters
    POWER_THRUSTERS();

    //delay 5 seconds to wait for thruster to power up
    for(uint8_t i = 0; i < 2;i++){ SEC_1_DELAY();}

    PWMPulseWidthSet(TKB_PWM_BASE,
                     TKB_PWM_FHL_PIN,
                     PWM_STOP_PER(TKB_PWM_BASE,TKB_FH_PWM_GEN));
    PWMPulseWidthSet(TKB_PWM_BASE,
                     TKB_PWM_FHR_PIN,
                     PWM_STOP_PER(TKB_PWM_BASE,TKB_FH_PWM_GEN));
    PWMPulseWidthSet(TKB_PWM_BASE,
                     TKB_PWM_FVL_PIN,
                     PWM_STOP_PER(TKB_PWM_BASE,TKB_FV_PWM_GEN));
    PWMPulseWidthSet(TKB_PWM_BASE,
                     TKB_PWM_FVR_PIN,
                     PWM_STOP_PER(TKB_PWM_BASE,TKB_FV_PWM_GEN));
    PWMPulseWidthSet(TKB_PWM_BASE,
                     TKB_PWM_BHL_PIN,
                     PWM_STOP_PER(TKB_PWM_BASE,TKB_BH_PWM_GEN));
    PWMPulseWidthSet(TKB_PWM_BASE,
                     TKB_PWM_BHR_PIN,
                     PWM_STOP_PER(TKB_PWM_BASE,TKB_BH_PWM_GEN));
    PWMPulseWidthSet(TKB_PWM_BASE,
                     TKB_PWM_BVL_PIN,
                     PWM_STOP_PER(TKB_PWM_BASE,TKB_BV_PWM_GEN));
    PWMPulseWidthSet(TKB_PWM_BASE,
                     TKB_PWM_BVR_PIN,
                     PWM_STOP_PER(TKB_PWM_BASE,TKB_BV_PWM_GEN));

    TKB_PWM_OUT_EN();

    for(uint8_t i = 0; i < 2;i++){ SEC_1_DELAY();}
}

/*
 * Desc: Executes thruster kill sequence
 *
 *       SEQUENCE:
 *       SEND STOP TO THRUSTERS
 *       KILL POWER TO THRUSTERS
 *       TRANSMIT HARD KILL TO MOBO
 *       KILL POWER TO MAIN
 */
void TKB_HardKill(void){
    for(uint8_t i = 0;i < 10;i++){SEC_1_DELAY();} //wait 10s
    KILL_MAIN();
    KILL_THRUSTERS();
    for(uint8_t i = 0;i < 10;i++){SEC_1_DELAY();} //wait 10s
    TKB_HardUnKill();
}

void TKB_HardUnKill(void){
    POWER_MAIN(); //give mobo power
    POWER_THRUSTERS(); //give thrusters power
    TKB_Init_ESC(); //init ESCs
}

/*
 * Desc: Executes thruster kill sequence
 *
 *       SEQUENCE:
 *       SEND STOP TO THRUSTERS //from SoftKill
 *       DISABLE PWM OUTPUT     //from SoftKill
 *       KILL POWER TO THRUSTERS
 *
 */
void TKB_SoftKill(void){
    KILL_THRUSTERS();
    TKB_IdleThrusters();
}

void TKB_SoftUnKill(void){
    POWER_THRUSTERS();
    TKB_Init_ESC();
}

/*
 * Desc: Executes idle
 *
 *       SEQUENCE:
 *       SEND STOP TO THRUSTERS
 *       DISABLE PWM OUTPUT
 */
void TKB_IdleThrusters(void){

    TKB_StopAllThrust();

}

/*
 * Desc: sends stop commands to all thrusters
 *
 * Assumes: PWM is initialized
 */
void TKB_StopAllThrust(void){

    PWMPulseWidthSet(TKB_PWM_BASE,
                     TKB_PWM_FHL_PIN,
                     PWM_STOP_PER(TKB_PWM_BASE,TKB_FH_PWM_GEN));
    PWMPulseWidthSet(TKB_PWM_BASE,
                     TKB_PWM_FHR_PIN,
                     PWM_STOP_PER(TKB_PWM_BASE,TKB_FH_PWM_GEN));
    PWMPulseWidthSet(TKB_PWM_BASE,
                     TKB_PWM_FVL_PIN,
                     PWM_STOP_PER(TKB_PWM_BASE,TKB_FV_PWM_GEN));
    PWMPulseWidthSet(TKB_PWM_BASE,
                     TKB_PWM_FVR_PIN,
                     PWM_STOP_PER(TKB_PWM_BASE,TKB_FV_PWM_GEN));
    PWMPulseWidthSet(TKB_PWM_BASE,
                     TKB_PWM_BHL_PIN,
                     PWM_STOP_PER(TKB_PWM_BASE,TKB_BH_PWM_GEN));
    PWMPulseWidthSet(TKB_PWM_BASE,
                     TKB_PWM_BHR_PIN,
                     PWM_STOP_PER(TKB_PWM_BASE,TKB_BH_PWM_GEN));
    PWMPulseWidthSet(TKB_PWM_BASE,
                     TKB_PWM_BVL_PIN,
                     PWM_STOP_PER(TKB_PWM_BASE,TKB_BV_PWM_GEN));
    PWMPulseWidthSet(TKB_PWM_BASE,
                     TKB_PWM_BVR_PIN,
                     PWM_STOP_PER(TKB_PWM_BASE,TKB_BV_PWM_GEN));

}
/*
 * Desc: will read in struct data and set thruster to the speed
 *       specified in speed variable
 *
 * Parameters:
 * thruster - your struct containg thruster data
 *
 * Assumes: Assumes PWM and ESCs are intialized
 */
void TKB_PWM_SetSpeed(tkb_thrust_data_t thruster){
    PWMPulseWidthSet(TKB_PWM_BASE,
                     thruster.pwm_out,
                     MIL_BR_linear_per(thruster.speed.speed_float,TKB_PWM_BASE,thruster.pwm_gen));
}


/*
 * Desc: Initialize timer to trigger overflow ISR
 *
 * Purpose: Will basically check every period if there's been
 *          recent data sent by motherboard
 */
void Timer0_OVF_Init(void (*pISR)(void), float period_ms){

    //We will be dividing the system clock by 255
    uint8_t prescale = 0xFF;

    //enable peripheral clock
    /*
     * A feature of ARM processors in general is that in order to use any
     * given peripheral(UART,Timers,etc.) you must enable their clocks
     */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);

    //wait for peripheral clock to stabilize
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER0));

    //configure timer for periodic count
    TimerConfigure(TIMER0_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_B_PERIODIC);

    //Set the prescale to divide system clock
    TimerPrescaleSet(TIMER0_BASE, TIMER_B, prescale);

    //set timer to interrupt every 1 second
    /*
     * The equation to determine timer period is
     * (Time in seconds) * (System Clock)/(Clock prescale)
     */
    TimerLoadSet(TIMER0_BASE, TIMER_B, period_ms/1000 * SysCtlClockGet()/prescale);

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
    TimerIntRegister(TIMER0_BASE, TIMER_B, pISR);

    //enable timer interrupts from CPU perspective
    IntEnable(INT_TIMER0B);

    //enable timer
    TimerEnable(TIMER0_BASE, TIMER_B);
}


void Timer1_OVF_Init(void (*pISR)(void), float period_ms){

    //We will be dividing the system clock by 255
    uint8_t prescale = 0xFF;

    //enable peripheral clock
    /*
     * A feature of ARM processors in general is that in order to use any
     * given peripheral(UART,Timers,etc.) you must enable their clocks
     */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);

    //wait for peripheral clock to stabilize
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER1));

    //configure timer for periodic count
    TimerConfigure(TIMER1_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_B_PERIODIC);

    //Set the prescale to divide system clock
    TimerPrescaleSet(TIMER1_BASE, TIMER_B, prescale);

    //set timer to interrupt every 1 second
    /*
     * The equation to determine timer period is
     * (Time in seconds) * (System Clock)/(Clock Prescale)
     */
    TimerLoadSet(TIMER1_BASE, TIMER_B, period_ms/1000 * SysCtlClockGet()/prescale);

    //enable interrupt triggered on timeout
    TimerIntEnable(TIMER1_BASE, TIMER_TIMB_TIMEOUT);

    //set ISR function
    /*
     * One of the features of the TI library is that the use can
     * pass a function pointer into this function to set their
     * own interrupt handler
     *
     * In this case I passed Timer0IntBlink into this function to
     * set it as my interrupt service routine
     */
    TimerIntRegister(TIMER1_BASE, TIMER_B, pISR);

    //enable timer interrupts from CPU perspective
    IntEnable(INT_TIMER1B);

    //enable timer
    TimerEnable(TIMER1_BASE, TIMER_B);
}


/*
 * Desc: Initialize All Hall effect sensor pins to input
 * Parameters:
 * pISR - Pointer to ISR(for kill handling)
 * activation_lvl -activation level of there being a
 *                 magnet
 *
 * NOTE: Interrupt is only attached to the ON_OFF
 *       signal. All other pins will be checked
 *       via other means
 */
void Init_HALL_IO(uint8_t activation_lvl){

    GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, HALL_HARDKILL_PIN | HALL_SOFTKILL_PIN | HALL_GO_PIN);

    //pull high or low in software
    //default to the magnets being removed
    uint32_t pull;

    //the state change that
    //indicates the ON/OFF magnet was removed
    //if active lo, this means a rising edge indicated the magnet was removed
    //if active hi, a falling edge indicated the magnet was removed
    uint32_t edge;

    switch(activation_lvl){
    case HALL_ACT_LO:
        pull = GPIO_PIN_TYPE_STD_WPU;
        edge = GPIO_RISING_EDGE;
        break;
    case HALL_ACT_HI:
        pull = GPIO_PIN_TYPE_STD_WPD;
        edge = GPIO_FALLING_EDGE;
        break;

    default:
        pull = GPIO_PIN_TYPE_STD_WPD;
        edge = GPIO_FALLING_EDGE;
        break;
    }

    GPIOPadConfigSet(GPIO_PORTB_BASE,
                     HALL_HARDKILL_PIN,
                     GPIO_STRENGTH_2MA,
                     pull);
    GPIOPadConfigSet(GPIO_PORTB_BASE,
                     HALL_SOFTKILL_PIN,
                     GPIO_STRENGTH_2MA,
                     pull);
    GPIOPadConfigSet(GPIO_PORTB_BASE,
                     HALL_GO_PIN,
                     GPIO_STRENGTH_2MA,
                     pull);
}


/*
 * Name: Check Hall functions
 * Desc: Each of these functions will
 *       output 0xFF is the inputs are
 *       high and 0x00 if the input
 *       is low
 */
uint8_t HALL_Check_Go(void){

    if(GPIOPinRead(GPIO_PORTB_BASE,HALL_GO_PIN) & HALL_GO_PIN){
        return HALL_HI;
    }
    else{return HALL_LO;}

}
uint8_t HALL_Check_Soft(void){

    if(GPIOPinRead(GPIO_PORTB_BASE,HALL_SOFTKILL_PIN) & HALL_SOFTKILL_PIN){
        return HALL_HI;
    }
    else{return HALL_LO;}

}
uint8_t HALL_Check_Hard(void){

    if(GPIOPinRead(GPIO_PORTB_BASE,HALL_HARDKILL_PIN ) & HALL_HARDKILL_PIN ){
        return HALL_HI;
    }
    else{return HALL_LO;}

}


/*
 * Desc: checks if the message is a kill message
 *       will return 1 if kill, 0 otherwise
 */
uint8_t TKB_Check_KillMsg(uint8_t *pMsg){
    if(pMsg[MSG_TYPE_IDX] == KILL_START_BYTE){
        return 0x01;
    }
    else{return 0x00;}
}

/*
 * Desc: checks if the message is a heartbeat message
 *       will return 1 if heartbeat, 0 otherwise
 */
uint8_t TKB_Check_HeartbeatMsg(uint8_t *pMsg){
    if(pMsg[MSG_TYPE_IDX] == HEARTBEAT_START_BYTE){
        return 0x01;
    }
    else{return 0x00;}
}

/*
 * Desc: checks if the message is a thruster message
 *       will return 1 if thruster, 0 otherwise
 */
uint8_t TKB_Check_ThrustMsg(uint8_t *pMsg){
    if(pMsg[MSG_TYPE_IDX] == THRUST_START_BYTE){
        return 0x01;
    }
    else{return 0x00;}
}

/*
 * Desc: Returns CR byte
 * Parameter: pointer to CAN message
 */
uint8_t TKB_Get_CRByte(uint8_t *pMsg){
    return pMsg[MSG_CR_IDX];
}

/*
 * Desc: Returns HS byte
 * Parameter: Pointer to CAN message
 */
uint8_t TKB_GetKill_HSByte(uint8_t *pMsg){
    return pMsg[MSG_HS_IDX];
}

/*
 * Desc: Returns UA byte
 * Parameter: pointer to CAN message
 */
uint8_t TKB_GetKill_UAByte(uint8_t *pMsg){
    return pMsg[MSG_UA_IDX];
}


/*
 * Desc: Will generate response message to be sent to motherboard after getting
 *       a thrust command
 */
void TKB_Gen_ThrustResponse(uint8_t thrust_id,float thrust_val, uint8_t pRMsg){



}






