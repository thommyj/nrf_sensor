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

//#define DEBUG 

struct status_packet packet;

void setup_hw(void);
bool get_latest_pkt(struct status_packet *pkt);
uint16_t adc_convert_to_mv(uint8_t adc);

void main()
{
	uint8_t rr = pwr_clk_mgmt_get_reset_reason();
	uint16_t i;
	uint16_t rpd = 0;

	//pwr_clk_mgmt_clear_reset_reasons(); TODO: this lib-call is broken, write is needed
	RSTREAS = 0xFF;

	setup_hw();
	
	printf("\r\nRunning: " __FILE__ ", build:" __DATE__ "\r\n");
	printf("reset reason: 0x%hhx\r\n", rr);

	for(i=0; ; i++){
		uint16_t saved_timeouts = 0;

		if(get_latest_pkt(&packet)){
			if(packet.magic == STATUS_PACKET_MAGIC){
				printf("Status pkt %d received\r\n", packet.sequence_nr);
				printf("   trap %d, vdc %dmV\r\n",
					packet.status[0],
					adc_convert_to_mv(packet.vdc));
				if(saved_timeouts != packet.timeouts){
					printf("WARNING: peer timeout increased %d->%d\r\n",
						saved_timeouts, packet.timeouts);
				}
			}else{
				printf("WARNING: corrupt packet received (magic 0x%08x)\r\n",
						packet.magic);
			}
		}

		rpd += rf_is_rpd_active();

		if(i == 0){
			printf("Current local voltage: %dmV\r\n",
				adc_convert_to_mv(adc_start_single_conversion_get_value(ADC_CHANNEL_1_THIRD_VDD)));
			printf("RF over -64dBm: %d times\r\n", rpd);

			rpd = 0;
		}
	}
}


/*
 * measurment is done by dividing vdc with 3, and using
 * 1.2V as a reference.
 * So to get voltage in mV: 3 * 1.2 * 1000 / 256 or
 * 3600/256 => 14 + 1/16, which shouldn't overflow in 16bit
 */
uint16_t adc_convert_to_mv(uint8_t adc)
{
	uint16_t adc_16 = (uint16_t)adc;
	return 14 * adc_16 + adc_16/16;
}

bool get_latest_pkt(struct status_packet *pkt)
{
	uint8_t pkt_cnt = 0;
		
	if(rf_irq_rx_dr_active()){
		do{
			if( pkt_cnt++ > 0){
				printf("WARNING: more than one pkt in que, discarding\r\n");
			}
			rf_read_rx_payload((uint8_t*)pkt, sizeof(struct status_packet));
#ifdef DEBUG
			printf("packet received (%d)!\r\n", pkt_cnt);
			printf("   magic       0x%04X\r\n", pkt.magic);
			printf("   sequence nr 0x%04X\r\n", pkt.sequence_nr);
			printf("   wakeups     0x%04X\r\n", pkt.wakeups);
			printf("   timeouts    0x%04X\r\n", pkt.timeouts);
			printf("   vdc         0x%02X\r\n", pkt.vdc);
			printf("   version     0x%02hhX\r\n", pkt.version);
			printf("   trap active 0x%02hhX\r\n", pkt.status[0]);
			printf("   P0          0x%02hhX\r\n", pkt.status[1]);
			printf("   P1          0x%02hhX\r\n", pkt.status[2]);
#endif
		}while((rf_get_status() & RF_STATUS_RX_P_NO_RX_FIFO_EMPTY) != RF_STATUS_RX_P_NO_RX_FIFO_EMPTY);
		rf_irq_clear_all();		//TODO: possible race? level/edge?
	}

	return pkt_cnt > 0;
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
