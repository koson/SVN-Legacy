/*
 * Name: MIL_BR_ESC.c
 * Author: Marquez Jones
 * Desc: Useful tools for interfacing
 *       PWM with Blue Robotics Basic
 *       ESCs to control the T200 Thrusters
 *
 *       Also some non-hardware specific
 *       functions for mapping Video Ray
 *       input to Blue robotics thrusters
 *
 * Notes: View this link for details on BR Basic ESC
 * https://www.bluerobotics.com/store/thrusters/speed-controllers/besc30-r3/
 */

#include "MIL_BR_ESC.h"

/*
 * Desc: This function maps video thrust values to
 *       to a PWM duty cycle to be used on the blue
 *       robotics ESC.
 *
 *       The video ray protocol sends a float whose
 *       value ranges from 1 to -1 which is then
 *       interpreted by the video ray thrusters
 *
 *       If your firmware solution emulates the
 *       video ray protocol, you can use this function
 *       to map the thrust value to a duty cycle to be applied
 *       to the blue robotics PWM signal
 * Input: VR_Thrust value
 * Output: The corresponding duty cycle
 */
float MIL_BlueDuty_VidRay(float VR_Thrust){

    //check if thrust is 0
    if(VR_Thrust){

        //-1 corresponds to 1100us/2000us duty, 1 corresponds to 1900us/2000us duty cycle
        return (VR_Thrust * 0.4 + 1.5)/BR_ESC_PERIOD_MS;

    }
    /*
     * 0 thrust implies halt
     * which implies a 75% duty cycle for
     * the PWM signal
     */
    else{
        return BR_STOP_THRUST_DUTY;
    }



}
