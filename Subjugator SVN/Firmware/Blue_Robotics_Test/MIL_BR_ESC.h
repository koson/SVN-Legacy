/*
 * Name: MIL_BR_ESC.h
 * Author: Marquez Jones
 * Desc: Useful tools for interfacing
 *       PWM with Blue Robotics Basic
 *       ESCs to control the T200 Thrusters
 *
 *       Also some non-hardware specific
 *       functions for mapping Video Ray
 *       thrust to Blue robotics PWM signal
 *
 * Notes: View this link for details on BR Basic ESC
 * https://www.bluerobotics.com/store/thrusters/speed-controllers/besc30-r3/
 */

#ifndef MIL_BR_ESC_H_
#define MIL_BR_ESC_H_

//the ESC expects pwm signal to have a period of 2ms
#define BR_ESC_PERIOD_MS 2
//In seconds
#define BR_ESC_PERIOD_SEC 0.002

//useful duty cycles
/*
 * Our application processor effectively
 * sends commands to the ESC via PWM signals
 * which the ESC then uses to control the speed
 * of the thrusters
 */
#define BR_MAX_FWD_THRUST_DUTY 1.9/BR_ESC_PERIOD_MS //max forward
#define BR_MAX_REV_TRHUST_DUTY 1.1/BR_ESC_PERIOD_MS //max reverse
#define BR_STOP_THRUST_DUTY 1.5/BR_ESC_PERIOD_MS //full stop

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
float MIL_BlueDuty_VidRay(float VR_Thrust);

#endif /* MIL_BR_ESC_H_ */





