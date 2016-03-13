/* --------------------------------------------------------------------------                                 
    Reference cell main
    File name: refcell.c
    Author: Charith Perera
    Description: The reference cell board was initially designed as a
                 current shunt board to measure the insolation; however
                 since being designed, we are trying to convert it to an
                 IV sweeper board by measuring I and V as the supercap
                 charges.
                 
                 Some useful info about the design:
                 ADC0 - Regulator in (to be hacked)
                 ADC7 - Current measurement
                 ADC5 - Thermistor (now Voltage sense)
                 LED_RED - P2.1
                 LED_YLW - P0.3
                 
                 Rough usable rate - ~70KS/s (2 reads and multiply)

    Copyright (C) Charith Perera, 2011

    Created: 23/12/2011
   -------------------------------------------------------------------------- */

/* 
 * This file is part of the Sunswift Reference Cell project.
 * 
 * This tempalte is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 
 * It is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 
 * You should have received a copy of the GNU General Public License
 * along with the project.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <scandal/engine.h>
#include <scandal/message.h>
#include <scandal/leds.h>
#include <scandal/utils.h>
#include <scandal/uart.h>
#include <scandal/stdio.h>

#include <project/driver_config.h>
#include <project/target_config.h>

#include <arch/can.h>
#include <arch/uart.h>
#include <arch/timer.h>
#include <arch/gpio.h>
#include <arch/types.h>
#include <arch/adc.h>
#include <arch/i2c.h>

#define ADC_REG 0
#define ADC_CUR 7
#define ADC_VOL 5

/* Do some general setup for clocks, LEDs and interrupts
 * and UART stuff on the MSP430 */
void setup(void) {
#if defined(lpc11c14) || defined(lpc1768)
	GPIO_Init();
	GPIO_SetDir(RED_LED_PORT,RED_LED_BIT,1); //Red LED, Out
	GPIO_SetDir(YELLOW_LED_PORT,YELLOW_LED_BIT,1); //Yel LED, Out
    ADC_Init(2);
    ADC_EnableChannel(ADC_VOL);
    ADC_EnableChannel(ADC_CUR);
    ADC_EnableChannel(ADC_REG);

    ADC_Read(ADC_CUR);
    ADC_Read(ADC_REG);

#endif // lpc1768 || lpc11c14
} // setup

/* This is an in-channel handler. It gets called when a message comes in on the
 * registered node/channel combination as set up using the USB-CAN. */
void in_channel_0_handler(int32_t value, uint32_t src_time) {
	UART_printf("in_channel_0_handler got called with value %d time at source %u\n\r", (int)value, (unsigned int)src_time);
}

/* This is your main function! You should have an infinite loop in here that
 * does all the important stuff your node was designed for */
int main(void) {
	int i = 0;
    uint32_t time, oldTime;
    uint32_t p_mp, p_new;
    uint32_t v_mp, i_mp;
    
	uint32_t value = 0xaa;

	setup();

	scandal_init();

	UART_Init(115200);

	sc_time_t one_sec_timer = sc_get_timer(); /* Initialise the timer variable */
	sc_time_t test_in_timer = sc_get_timer(); /* Initialise the timer variable */

	/* Set LEDs to known states */
    GPIO_SetValue(YELLOW_LED_PORT,YELLOW_LED_BIT,1);
    GPIO_SetValue(RED_LED_PORT, RED_LED_BIT,1);

	scandal_delay(100); /* wait for the UART clocks to settle */

	scandal_register_in_channel_handler(0, &in_channel_0_handler);
    
    GPIO_SetValue(YELLOW_LED_PORT,YELLOW_LED_BIT,0);
    GPIO_SetValue(RED_LED_PORT, RED_LED_BIT,0);
    oldTime=sc_get_timer();
    for(i=0; i<2; i++){
        ADC_Read(ADC_VOL);
        ADC_Read(ADC_CUR);
        p_new=ADCValue[ADC_VOL]*ADCValue[ADC_CUR];
        //UART_printf("%d, %d", i, ADCValue[ADC_VOL]);
    }
    time=sc_get_timer()-oldTime;
        GPIO_SetValue(YELLOW_LED_PORT,YELLOW_LED_BIT,0);
        GPIO_SetValue(RED_LED_PORT, RED_LED_BIT,0);
        
        int average_current=0;
        int average_counter=0;
	/* This is the main loop, go for ever! */
	while (1) {
		/* This checks whether there are pending requests from CAN, and sends a heartbeat message.
		 * The heartbeat message encodes some data in the first 4 bytes of the CAN message, such as
		 * the number of errors and the version of scandal */
		handle_scandal();
        //ADC_BurstRead;
        ADC_Read(ADC_VOL);
        scandal_naive_delay(5);
        ADC_Read(ADC_CUR);
        scandal_naive_delay(5);
        //scandal_delay(1);
        ADC_Read(ADC_REG);
        scandal_naive_delay(5);
        //average_current += ;
        UART_printf("ADC: %d, %d, %d\n\r", ADCValue[ADC_CUR], ADCValue[ADC_VOL], (SystemCoreClock/((LPC_SYSCON->SYSAHBCLKDIV)*ADC_MAXCLK)));

       
   

	}
}
