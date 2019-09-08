/********************************************** Includes ********************************************/
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/can.h>
#include <libopencm3/stm32/flash.h>

/****************************************************************************************************/

/*********************************************** Defines **********************************************************************************************/
#define number_of_adc_channels 2
#define ttcm_off     0	/*In this mode, the internal counter of the CAN hardware is activated and used to generate the
						  Time Stamp value stored in the CAN_RDTxR/CAN_TDTxR registers, respectively (for Rx
						  and Tx mailboxes). The internal counter is incremented each CAN bit time (refer to
						  Section 29.7.7: Bit timing). The internal counter is captured on the sample point of the Start
						  Of Frame bit in both reception and transmission. (time triggered communication management) */
#define abom_on      1	/*Depending on the ABOM bit in the CAN_MCR register bxCAN will recover from Bus-Off
						  (become error active again) either automatically or on software request. But in both cases
						  the bxCAN has to wait at least for the recovery sequence specified in the CAN standard
						  (128 occurrences of 11 consecutive recessive bits monitored on CANRX).
						  If ABOM is set, the bxCAN will start the recovering sequence automatically after it has
						  entered Bus-Off state. (auto bus off management)*/
#define awum_on      1	/*On CAN bus activity detection, hardware automatically performs the wakeup sequence by
						  clearing the SLEEP bit if the AWUM bit in the CAN_MCR register is set. (auto wake-up management) */
#define nart_on      1	/*In this mode, each transmission is started only once. If the first attempt fails, due to an
						  arbitration loss or an error, the hardware will not automatically restart the message
						  transmission.(non auto re-transmission)*/
#define rflm_off     0	/*If the FIFO lock function is disabled (RFLM bit in the CAN_MCR register cleared) the
						  last message stored in the FIFO will be overwritten by the new incoming message. In
						  this case the latest messages will be always available to the application. ()receive fifo lock mode */
#define txfp_on	     1	/*The transmit mailboxes can be configured as a transmit FIFO by setting the TXFP bit in the
						  CAN_MCR register. In this mode the priority order is given by the transmit request order. (transmission fifo priority) */
#define sjw_2        2  /*a bit change is expected to occur within this
						  time segment. It has a fixed length of one time quantum (1 x tq). (synchronization jump width)*/
#define ts1_14      14	/*defines the location of the sample point. It includes the
						  PROP_SEG and PHASE_SEG1 of the CAN standard. Its duration is programmable
						  between 1 and 16 time quanta but may be automatically lengthened to compensate for
						  positive phase drifts due to differences in the frequency of the various nodes of the
						  network. (transmit segment 1) */
#define ts2_3        3  /*defines the location of the transmit point. It represents the
						  PHASE_SEG2 of the CAN standard. Its duration is programmable between 1 and 8
						  time quanta but may also be automatically shortened to compensate for negative
						  phase drifts. (transmit segment 2)*/
#define can_baud_psc 3  //using clock frequency
#define loopback_off 0  /*"In Loop Back Mode, the bxCAN treats its own transmitted messages as received
						messages and stores them (if they pass acceptance filtering) in a Receive mailbox.*/
#define silent_off   0  /*In Silent mode, the bxCAN is able to receive valid data frames and valid remote frames, but
						it sends only recessive bits on the CAN bus and it cannot start a transmission*/

#define can_id	0xAAAA   		//Can message id. Need to change later
#define can_ext_id_off 0 		//not need for extended can id
#define can_request_trans 1
#define can_8bytes_message 8	//8 bytes

#define ADC_MAX	4095
#define VREF 3.3

#define RESISTOR1 800000
#define RESISTOR2 100000
/******************************************************************************************************************************************************/

/***************************************Data Types and Structures************************************/
typedef uint16_t sample_t;

union data_t {
   uint64_t packet_uint64;
   uint32_t packet_uint32[2];
   float packet_float[2];
   uint8_t packet_char[8];
} data_t;

typedef struct sensor_t
{
	sample_t sensor_reading; //ADC sensor reading
} sensor_t;

typedef struct CanTx_t {
	uint32_t std
} CanTx_t;

/****************************************************************************************************/

/***************************************Global Variables*********************************************/
sensor_t voltage;
sensor_t current;
uint32_t packed_sensor_data;
uint8_t channel_array[] = {ADC_CHANNEL0, ADC_CHANNEL1};
/****************************************************************************************************/
