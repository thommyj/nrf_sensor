#include <stdio.h>
#include <string.h>

#include <gpio.h>
#include <uart.h>
#include <adc.h>
#include <pwr_clk_mgmt.h>
#include <interrupt.h>
#include <rtc2.h>
#include <delay.h>

#include "reg24le1.h"


void setup_hw(void);

void main()
{
	int i;
	uint8_t rr = pwr_clk_mgmt_get_reset_reason();

	//pwr_clk_mgmt_clear_reset_reasons(); TODO: this lib-call is broken, write is needed
	RSTREAS = 0xFF;

	setup_hw();
	
	printf("\r\nRunning: " __FILE__ ", build:" __DATE__ "\r\n");
	printf("reset reason: 0x%hhx\r\n", rr);

	while(1){
	printf("VDC measure: %d\r\n",
		adc_start_single_conversion_get_value(ADC_CHANNEL_1_THIRD_VDD));
	}
}

void setup_hw()
{
	//Setup UART pins
        gpio_pin_configure(GPIO_PIN_ID_FUNC_RXD,
                                           GPIO_PIN_CONFIG_OPTION_DIR_INPUT |
                                           GPIO_PIN_CONFIG_OPTION_PIN_MODE_INPUT_BUFFER_ON_NO_RESISTORS);

        gpio_pin_configure(GPIO_PIN_ID_FUNC_TXD,
                                           GPIO_PIN_CONFIG_OPTION_DIR_OUTPUT |
                                           GPIO_PIN_CONFIG_OPTION_OUTPUT_VAL_SET |
                                           GPIO_PIN_CONFIG_OPTION_PIN_MODE_OUTPUT_BUFFER_NORMAL_DRIVE_STRENGTH);
        //38400 baud
        uart_configure_manual_baud_calc(UART_CONFIG_OPTION_ENABLE_RX |
                                                                        UART_CONFIG_OPTION_MODE_1_UART_8_BIT |
                                                                        UART_CONFIG_OPTION_CLOCK_FOR_MODES_1_3_USE_BR_GEN |
                                                                        UART_CONFIG_OPTION_BIT_SMOD_SET,
                                                                        1011);
	
	//setup adc
	adc_configure (ADC_CONFIG_OPTION_RESOLUTION_8_BITS |
			ADC_CONFIG_OPTION_SAMPLING_MODE_SINGLE_ENDED |
			ADC_CONFIG_OPTION_ACQ_TIME_3_US |
			ADC_CONFIG_OPTION_RESULT_JUSTIFICATION_RIGHT);

}

void putchar(char c)
{
        uart_send_wait_for_complete(c);
}

char getchar()
{
        return uart_wait_for_rx_and_get();
}
