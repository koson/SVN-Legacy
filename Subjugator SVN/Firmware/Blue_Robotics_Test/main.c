/*
 * Name: Blue_Robotics_Test
 * Author: Marquez Jones
 * Desc: Test program to prototype control
 *       of BR thrusters
 */

//includes
#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/pwm.h"
#include "driverlib/sysctl.h"

#include "MIL_BR_ESC.h"

/*********************************************FUNCTION PROTOTYPES**********************************/

/*
 * Desc: Configure PWM1 Gen3 ,Bits 6 and 7
 *       Will configure both pins for 90% duty
 *       cycle. Does not enable output(handled in main)
 *
 * Hardware Notes:
 * M1PWM6 : PF2
 * M1PWM7 : PF3
 *
 */
void Init_PWM1_Gen3_6(void);

//Desc: arbitrary software delay
void ArbSoftDelay(void);

/*********************************************MAIN*************************************************/

int main(void){

    //set system clock to 16MHZ
    SysCtlClockSet(SYSCTL_SYSDIV_1 |
                   SYSCTL_USE_OSC  |
                   SYSCTL_OSC_MAIN |
                   SYSCTL_XTAL_16MHZ);

    //Initialize PWM
    Init_PWM1_Gen3_6();

    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6,
                     PWMGenPeriodGet(PWM1_BASE, PWM_GEN_3)* BR_STOP_THRUST_DUTY);

    //set Blue on and green off
    PWMOutputState(PWM1_BASE, PWM_OUT_6_BIT, true);

    ArbSoftDelay();
    ArbSoftDelay();

    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6,
                     PWMGenPeriodGet(PWM1_BASE, PWM_GEN_3)* BR_MAX_FWD_THRUST_DUTY*0.5);

    for(uint8_t idx = 0; idx < 100; idx++){

        ArbSoftDelay();

    }

    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6,
                     PWMGenPeriodGet(PWM1_BASE, PWM_GEN_3)* BR_STOP_THRUST_DUTY);


    while(1){


    }

}

/*********************************************FUNCTION DEFINITIONS**********************************/

/*
 * Desc: Configure PWM1 Gen3 ,Bits 6 and 7
 *       Duty cycle and signal output enable
 *       will be handled externally
 *
 *       Blue robotics ESC expects as 24
 *
 * Hardware Notes:
 * M1PWM6 : PF2
 *
 */
void Init_PWM1_Gen3_6(void){

        //PWM clock enable
        SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);

        //port F clock enable
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

        //enable PWM functions on PF2 and PF3
        GPIOPinConfigure(GPIO_PF2_M1PWM6);

        //Configure PF2 and PF3 as PWM
        GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_2);

        /*
         * Configure:
         * PWM Mod 1
         * Generator 3
         * Up down mode
         * No sync(you can sync generators together)
         */
        PWMGenConfigure(PWM1_BASE,
                        PWM_GEN_3,
                        PWM_GEN_MODE_UP_DOWN |
                        PWM_GEN_MODE_NO_SYNC);

        PWMGenPeriodSet(PWM1_BASE, PWM_GEN_3, BR_ESC_PERIOD_SEC * (SysCtlClockGet()));

        //enable pwm
        PWMGenEnable(PWM1_BASE, PWM_GEN_3);

}

//Desc: arbitrary software delay
void ArbSoftDelay(void){

    for(uint16_t idx = 0; idx < 60000;idx++);
}







