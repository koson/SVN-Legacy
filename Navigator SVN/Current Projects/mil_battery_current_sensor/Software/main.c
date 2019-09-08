/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2009 Uwe Hermann <uwe@hermann-uwe.de>
 * Copyright (C) 2011 Stephen Caudle <scaudle@doceme.com>
 * Modified by Fernando Cortes <fermando.corcam@gmail.com>
 * modified by Guillermo Rivera <memogrg@gmail.com>
 * modified by Frantisek Burian <BuFran@seznam.cz>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

/* PA0 - Channel(0)
 * PA1 - Channel(1)*/

#include <CurrentVoltageSense.h>

void InitCAN(void)
{
	rcc_periph_clock_enable(RCC_CAN);
	gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO9);
	can_reset(CAN1);
	if (can_init
			(
				CAN1,
				ttcm_off,			/* TTCM: Time triggered comm mode? pg.822 */
				abom_on,			/* ABOM: Automatic bus-off management? Pg.829 */
				awum_on,			/* AWUM: Automatic wakeup mode? Pg.817 */
				nart_on,			/* NART: No automatic retransmission? Pg.821 */
				rflm_off,			/* RFLM: Receive FIFO locked mode? Pg.823 */
				txfp_on,			/* TXFP: Transmit FIFO priority? Pg.829*/
				CAN_BTR_SJW_4TQ,	/* Synchronization Segment Pg.829*/
				CAN_BTR_TS1_13TQ,	/* Bit Segment 1 Pg.829 http://www.bittiming.can-wiki.info/ */
				CAN_BTR_TS2_2TQ,	/* Bit Segment 2 Pg.829 http://www.bittiming.can-wiki.info// */
				can_baud_psc,		/* Baud Prescaler Pg.843 */
				loopback_off,		/* Loop back mode Pg.819 */
				silent_off
			)
		)		/* Silent Mode? Pg.818 */
	{
		while(1){
			//Error!!!
			gpio_toggle(GPIOC, GPIO8);	/* LED on/off */
			for (int i = 0; i < 100000; i++) {	/* Wait a bit. */
				__asm__("nop");
			}
		}
	}
	/* The bxCAN provides up to 14 scalable/configurable
	identifier filter banks, for selecting the
	incoming messages, that the software needs and
	discarding the others.*/

//	can_filter_init(
//				0,     /* Filter ID */
//				0,     /* (True) 32 bit or (False) 16 bit Filter ID */
//				0,     /* (True) ID lists or (False) ID Mask*/
//				0,     /* FIFO assignment (here: FIFO0) | 1 for FIFO1 */
//				0,	   /* FIFO ID */
//				true/* Enable the filter. */
//	);

}

void InitLED(){
	/* Enable GPIOC clock. */
	/* Manually: */
	//RCC_AHBENR |= RCC_AHBENR_GPIOCEN;
	/* Using API functions: */
	rcc_periph_clock_enable(RCC_GPIOC);


	/* Set GPIO8 (in GPIO port C) to 'output push-pull'. */
	/* Using API functions: */
	gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO8);
}

InitCAN2(){
	/* (1) Enter CAN init mode to write the configuration */
	/* (2) Wait the init mode entering */
	/* (3) Exit sleep mode */
	/* (4) Loopback mode, set timing to 1Mb/s: BS1 = 4, BS2 = 3,
	prescaler = 6 */
	/* (5) Leave init mode */
	/* (6) Wait the init mode leaving */
	/* (7) Enter filter init mode, (16-bit + mask, filter 0 for FIFO 0) */
	/* (8) Acivate filter 0 */
	/* (9) Set the Id and the mask (all bits of standard id care */
	/* (10) Leave filter init */
	/* (11) Set FIFO0 message pending IT enable */
	CAN->MCR |= CAN_MCR_INRQ; /* (1) */
	while ((CAN->MSR & CAN_MSR_INAK) != CAN_MSR_INAK) /* (2) */
	{
	/* add time out here for a robust application */
	}
	CAN->MCR &=~ CAN_MCR_SLEEP; /* (3) */
	CAN->BTR |= CAN_BTR_LBKM | 2 << 20 | 3 << 16 | 5 << 0; /* (4) */
	CAN->MCR &=~ CAN_MCR_INRQ; /* (5) */
	while ((CAN->MSR & CAN_MSR_INAK) == CAN_MSR_INAK) /* (6) */
	{
	/* add time out here for a robust application */
	}
	CAN->FMR |= CAN_FMR_FINIT; /* (7) */
	CAN->FA1R |= CAN_FA1R_FACT0; /* (8) */
	CAN->sFilterRegister[0].FR1 = CAN_ID << 5 | 0xFF70U << 16; /* (9) */
	CAN->FMR &=~ CAN_FMR_FINIT; /* (10) */
	CAN->IER |= CAN_IER_FMPIE0; /* (11) */
}

void InitADC()
{
	  /*During wakeup from Stop or Standby mode, SYSCLK takes the default setting: HSI 8 MHz.*/

	  /*RCC Pg. 96: The ADC clock which is derived (selected by software) from one of the two following sources:
	      – dedicated HSI14 clock, to run always at the maximum sampling rate
	      – APB clock (PCLK) divided by 2 or 4*/

	  /*Note: When the peripheral clock is not active, the peripheral register values may not be readable
	  by software and the returned value is always 0x0.*/

	  /*The elapsed time between the start of a conversion and the end of conversion is the sum of
	  the configured sampling time plus the successive approximation time depending on data resolution*/

	gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO0);
	gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO1);


	rcc_periph_clock_enable(RCC_ADC);
	rcc_periph_clock_enable(RCC_GPIOA);

    adc_power_off(ADC1);                                 //Disabling ADC during configuration
	adc_set_clk_source(ADC1, ADC_CLKSOURCE_PCLK_DIV2);   //In this case, I am using the PCLK(8 MHz)/2
    adc_calibrate_start(ADC1);                           //Initially calibrating ADC
    adc_set_operation_mode(ADC1, ADC_MODE_SEQUENTIAL);   /*In this mode, after the trigger event a single channel is converted and the
 	 	 	 	 	 	 	 	 	 	 	 	 	 	 next channel in the list is prepared to convert on next trigger edge.*/
    adc_disable_external_trigger_regular(ADC1);          //Hardware trigger detection disabled (conversions can be started by software)
    adc_set_right_aligned(ADC1);                         //Right Align Data
    adc_set_sample_time_on_all_channels(ADC1, ADC_SMPTIME_001DOT5); //Sampling ADC every 1.5 ADC (PCLK/2 in this case) clock cycles
    adc_set_regular_sequence(ADC1, number_of_adc_channels, channel_array);    //using adc channel 0 and 1       ADC Channel 0(1) is PA0(PA1); Pin 10(11)
    adc_set_resolution(ADC1, ADC_RESOLUTION_12BIT);      //Using a 12 bit ADC
	adc_disable_analog_watchdog(ADC1);                   //Not using analog watchdog
	adc_power_on(ADC1);                                  //Enabling ADC

	/* Wait for ADC starting up. */
	int i;
	for (i = 0; i < 800000; i++) {    /* Wait a bit. */
		__asm__("nop");
	}
}

void can_send(uint8_t *data)
{
	can_transmit(CAN1, can_id, can_ext_id_off, can_request_trans, can_8bytes_message, &data);
}

sample_t read_adc()
{
	adc_start_conversion_regular(ADC1);
	while (!(adc_eoc(ADC1)));
	return (adc_read_regular(ADC1));

	/*uint32_t adc_channel_reading;

	adc_start_conversion_regular(ADC1);
	while (!(adc_eoc(ADC1)));
	adc_channel_reading = adc_read_regular(ADC1);

	adc_start_conversion_regular(ADC1);
	while (!(adc_eoc(ADC1)));
	adc_channel_reading = (adc_read_regular(ADC1) << 16) | adc_channel_reading;

	return adc_channel_reading;*/
}

float adc_to_voltage(uint16_t adc_reading)
{
	double voltage_raw = (((double)adc_reading)/((double)(ADC_MAX)))*((double)VREF);
	double voltage_corrected = (voltage_raw*0.9006)-0.01960;
	return (float)voltage_corrected;
}

float battery_voltage(float adc_voltage)
{
	return (float)((((double)(RESISTOR1 + RESISTOR2))/((double)RESISTOR2))*((double)adc_voltage));
}

//float current_reading

uint8_t* read_packed_sensor_data(uint32_t lpf_taps, sensor_t *voltage, sensor_t *current)
{
	union data_t sensor_data;
	voltage->sensor_reading = 0;
	current->sensor_reading = 0;
	for (int i = 0; i < lpf_taps; i++)
	{
		current->sensor_reading += read_adc();
		voltage->sensor_reading += read_adc();
		asm(" nop");
	}
	voltage->sensor_reading /= lpf_taps;
	current->sensor_reading /= lpf_taps;

	//sensor_data.packet_uint64 = ((uint32_t)(voltage->sensor_reading)<<16) | ((uint32_t)(current->sensor_reading));
	sensor_data.packet_float[0] = adc_to_voltage(current->sensor_reading);
	sensor_data.packet_float[1] = battery_voltage(adc_to_voltage(voltage->sensor_reading));
	float test1 = sensor_data.packet_float[0];
	return &(sensor_data.packet_char);
}

int main(void)
{
	//InitADC();
	InitLED();
	InitCAN();
	char test[8] = {1,2,3,4,5,6,7,8};
	while (1) {
		//packed_sensor_data = read_packed_sensor_data(10, &voltage, &current);
		//float test = adc_to_voltage(read_adc());
		can_send(test);
		for (int i = 0; i < 800000; i++) {   /* Wait a bit. */
			__asm__("nop");
		}
	}

	return 0;
}

