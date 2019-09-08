/*
 * Name: MIL_CAN.h
 * Author: Marquez Jones
 * Date Modified: 2/8/2019
 * Desc: A set of wrapper functions I designed
 *       to standardized CAN use in the Lab
 *       this primarily is to reduce the number
 *       of bugs caused by the CAN bus
 *
 * Notes: CAN devices should be run at 100k bps
 *        and PCBs on the network should have on
 *        board termination resistors
 */

#include <stdbool.h>

#ifndef MIL_CAN_H_
#define MIL_CAN_H_

/*
 *Desc: Port selection will come from this enum
 */
typedef enum {
    MIL_CAN_PORT_A,
    MIL_CAN_PORT_B,
	MIL_CAN_PORT_C,
	MIL_CAN_PORT_D,
    MIL_CAN_PORT_E,
    MIL_CAN_PORT_F
}mil_can_port_t;

typedef enum {

    MIL_CAN_RX,     // a message is received
    MIL_CAN_NORX    // a message is not received

}mil_can_status_t;

/*
 * Desc: enables CAN0 which can be enabled on
 *       Ports B,E, or F
 *
 * Notes: Does not enable interrupts
 *        or port clocks
 *        PORT and Interrupts
 *        must be enabled outside funciton
 *
 * Hardware Notes:
 * CAN0:
 * PF0 - CANRX  PB4 - CANRX  PE4 - CANRX
 * PF3 - CANTX  PB5 - CANTX  PE5 - CANTX
 * CAN1:
 * PA0 - CANRX
 * PA1 - CANTX
 *
 * Inputs: port from mil_port enum
 * Assumes: Port clocks are enabled
 */
void MIL_InitCAN(mil_can_port_t port,uint32_t base);


/*
 * Desc: Enables interrupts on CAN0
 *
 * Notes: The Tiva CAN system has two
 *        separate interrupt sources.
 *        Either a controller error has
 *        occured(Tiva side) or there's
 *        a status change which can result
 *        from message transfer or a system
 *        bus error.
 *
 *        Further diagnostics in required in
 *        the external program
 *
 * Inputs: A pointer to your custom ISR
 * Assumes: Nothing
 */
void MIL_CANIntEnable(void (*func_ptr)(void),uint32_t base);

/*
 * Desc: Enables the Port associated with the CAN system
 *
 * Note: For sake of your program making sense,
 *       only call this function if your CAN shares the
 *       port with no other GPIOs or peripherals
 *
 *       This just called the GPIO clock enable for
 *       the port
 */
void MIL_CANPortClkEnable(mil_can_port_t port);

/*
 * Desc: Easy to use function to transmit a message to the CAN bus
 * 	     This function declares a temporary Can message object and uses
 *	     object 0 to transmit the message
 * 
 * Inputs: 
 * canid - ID of your CAN node
 * pMsg  - pointer to your message 
 * MsgLen - the number of bytes in your message(up to 8 bytes)
 * base - which CAN module you want to use(CAN1_BASE or CAN0_BASE from tivaWare)
 */
void MIL_CANSimpleTX(uint32_t canid,uint8_t *pMsg,uint8_t MsgLen, uint32_t base);

/*
 * Desc: Easy to use function to receive a message to the CAN bus
 *       This function declares a temporary Can message object and uses
 *       object 1 to receive the message
 *
 * Inputs:
 *
 * useFilter - boolean that determines if you wish to use an ID filter
 * filterID - ID of messages you wish to filter for node
 * filterMask - Bit mask of important bits to filter for
 * pMsg  - pointer to your message buffer
 * MsgLen - the number of bytes in your message(up to 8 bytes)
 * base - which CAN module you want to use(CAN1_BASE or CAN0_BASE from tivaWare)
 *
 * Outputs:
 *  function will return whether or not a message has been received
 *  message returned will be loaded into the message pointer
 *
 *
 */
mil_can_status_t MIL_CANSimpleRX(bool     useFilter,  uint32_t filterID,
                                 uint32_t filterMask, uint8_t *pMsg,
                                 uint8_t MsgLen,      uint32_t base);

#endif /* MIL_CAN_H_ */
