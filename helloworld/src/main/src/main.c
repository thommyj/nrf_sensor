#include <stdio.h>

#include <gpio.h>
#include <uart.h>
#include <adc.h>
#include "reg24le1.h"

void initialize(void);
void led_toggle(void);
void delay_ms(unsigned int ms);

void main()
{

	initialize(); //initialize IO, UART, set up nRF24L01 as TX
	printf("\r\n\r\nfile: " __FILE__ ", build:" __DATE__ "\r\n");
	//main program loop
	while(1)
	{
		uint16_t meas = adc_start_single_conversion_get_value(ADC_CHANNEL_1_THIRD_VDD);
		printf("Measurment of VDC: %d\r\n", meas);
		
		printf("LED (P0.5) is off\r\n");
		delay_ms(1000);
		led_toggle(); 
		printf("LED (P0.5) is on\r\n");
		delay_ms(1000);
		led_toggle();
	}
}

void initialize()
{
	//Set up LED pin
	gpio_pin_configure(GPIO_PIN_ID_P0_5,
					   GPIO_PIN_CONFIG_OPTION_DIR_OUTPUT |
					   GPIO_PIN_CONFIG_OPTION_OUTPUT_VAL_CLEAR |
					   GPIO_PIN_CONFIG_OPTION_PIN_MODE_OUTPUT_BUFFER_NORMAL_DRIVE_STRENGTH);

        //Set up UART pins
        gpio_pin_configure(GPIO_PIN_ID_FUNC_RXD,
                                           GPIO_PIN_CONFIG_OPTION_DIR_INPUT |
                                           GPIO_PIN_CONFIG_OPTION_PIN_MODE_INPUT_BUFFER_ON_NO_RESISTORS);

        gpio_pin_configure(GPIO_PIN_ID_FUNC_TXD,
                                           GPIO_PIN_CONFIG_OPTION_DIR_OUTPUT |
                                           GPIO_PIN_CONFIG_OPTION_OUTPUT_VAL_SET |
                                           GPIO_PIN_CONFIG_OPTION_PIN_MODE_OUTPUT_BUFFER_NORMAL_DRIVE_STRENGTH);

        //Set up UART for 38400 baud
        uart_configure_manual_baud_calc(UART_CONFIG_OPTION_ENABLE_RX |
                                                                        UART_CONFIG_OPTION_MODE_1_UART_8_BIT |
                                                                        UART_CONFIG_OPTION_CLOCK_FOR_MODES_1_3_USE_BR_GEN |
                                                                        UART_CONFIG_OPTION_BIT_SMOD_SET,
                                                                        1011);
	adc_configure (ADC_CONFIG_OPTION_RESOLUTION_8_BITS |
			ADC_CONFIG_OPTION_SAMPLING_MODE_SINGLE_ENDED |
			ADC_CONFIG_OPTION_ACQ_TIME_3_US |
			ADC_CONFIG_OPTION_RESULT_JUSTIFICATION_RIGHT);
}

void led_toggle(void)
{
	gpio_pins_val_complement(P0,1<<5);
}


#define SPINS_PER_MS (1500)
void delay_ms(unsigned int ms)
{
	unsigned int i;
	for(;ms>0;ms--){
		for(i=0;i<SPINS_PER_MS;i++){
		/*NOP*/
		}
	}
}

void putchar(char c)
{
        uart_send_wait_for_complete(c);
}

char getchar()
{
        return uart_wait_for_rx_and_get();
}
