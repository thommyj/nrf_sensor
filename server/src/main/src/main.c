#include <stdio.h>
#include <string.h>

#include <gpio.h>
#include <uart.h>
#include <adc.h>
#include <pwr_clk_mgmt.h>
#include <interrupt.h>
#include <rtc2.h>
#include <delay.h>
#include <rf.h>

#include "reg24le1.h"
#include "sensor.h"

struct status_packet packet;

void setup_hw(void);

void main()
{
	uint8_t rr = pwr_clk_mgmt_get_reset_reason();
	uint16_t i;
	uint8_t rf_status = 0;
	uint8_t rf_fifo_status = 0;
	uint16_t rpd = 0;

	//pwr_clk_mgmt_clear_reset_reasons(); TODO: this lib-call is broken, write is needed
	RSTREAS = 0xFF;

	setup_hw();
	
	printf("\r\nRunning: " __FILE__ ", build:" __DATE__ "\r\n");
	printf("reset reason: 0x%hhx\r\n", rr);

	for(i=0; ; i++){
		uint8_t buffer;

		if(rf_irq_rx_dr_active()){
			uint8_t pkt_cnt = 0;
			uint8_t status;
			do{
				rf_read_rx_payload((uint8_t*)packet, sizeof(packet));
				printf("packet received (%d)!\r\n", ++pkt_cnt);
				printf("   magic       0x%04X\r\n", packet.magic);
				printf("   sequence nr 0x%04X\r\n", packet.sequence_nr);
				printf("   wakeups     0x%04X\r\n", packet.wakeups);
				printf("   timeouts    0x%04X\r\n", packet.timeouts);
				printf("   vdc         0x%02X\r\n", packet.vdc);
				printf("   version     0x%02hhX\r\n", packet.version);
				printf("   trap active 0x%02hhX\r\n", packet.status[0]);
				printf("   P0          0x%02hhX\r\n", packet.status[1]);
				printf("   P1          0x%02hhX\r\n", packet.status[2]);

			}while((rf_get_status() & RF_STATUS_RX_P_NO_RX_FIFO_EMPTY) != RF_STATUS_RX_P_NO_RX_FIFO_EMPTY);
			rf_irq_clear_all();		//TODO: possible race? level/edge?
		}

		rf_status |= rf_get_status();
		rf_read_register(RF_FIFO_STATUS, &buffer, 1);
		rf_fifo_status |= buffer;
		rpd += rf_is_rpd_active();

		if(i == 0){
			printf("VDC measure: %d\r\n",
				adc_start_single_conversion_get_value(ADC_CHANNEL_1_THIRD_VDD));
			printf("RF status: 0x%02X\r\n", rf_status);
			printf("RF FIFO status: 0x%02X\r\n",rf_fifo_status);
			printf("RF RPD: %d\r\n", rpd);

			rf_status = 0;
			rf_fifo_status = 0;
			rpd = 0;
		}
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

	//TODO: calculate reasonable ARD
	//TODO: gain control?
	rf_configure(RF_CONFIG_MASK_TX_DS |
			RF_CONFIG_MASK_MAX_RT |
			RF_CONFIG_EN_CRC |
			RF_CONFIG_CRCO |
			RF_CONFIG_PWR_UP |
			RF_CONFIG_PRIM_RX,
		true,			/* take rx to active */
		RF_EN_AA_ENAA_P0, 	/* enable enhanced shochburst channel 0 */
		RF_EN_RXADDR_ERX_P0,	/* enable channel 0 */
		RF_SETUP_AW_5BYTES,	/* use 5byte address */
		RF_SETUP_RETR_ARD_4000 | RF_SETUP_RETR_ARC_15,
		RF_RF_CH_DEFAULT_VAL,	/* use default channel */
		RF_RF_SETUP_RF_DR_1_MBPS | RF_RF_SETUP_RF_PWR_0_DBM,
		client0_address,
		NULL, 0, 0, 0, 0,	/* not interested in other channel addresses*/
		server_address,
		sizeof(struct status_packet),  /* size of expected packet */ 
		0,0,0,0,0,		
		RF_DYNPD_DPL_NONE,	/* no dynamic payload size */
		0);
}		

void putchar(char c)
{
        uart_send_wait_for_complete(c);
}

char getchar()
{
        return uart_wait_for_rx_and_get();
}
