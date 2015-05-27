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
#include <sspi.h>

#include "reg24le1.h"
#include "sensor.h"

//#define DEBUG 

struct spi_packet
{
	uint16_t seq;
	uint16_t lost;
	uint8_t  trap;
	uint8_t  vdc;
};

struct spi_packet tx_spipkt;
struct status_packet rec_rfpkt;

void setup_hw(void);
bool get_latest_pkt(struct status_packet *pkt);
uint16_t adc_convert_to_mv(uint8_t adc);
void prepare_spipkt(struct spi_packet *spi, struct status_packet *rf);


void main()
{
	uint8_t rr = pwr_clk_mgmt_get_reset_reason();
	uint16_t i;
	uint16_t rpd = 0;
	uint8_t spi_cnt;
	uint16_t saved_timeouts = 0;
	uint16_t saved_seqnr = 0;
	//pwr_clk_mgmt_clear_reset_reasons(); TODO: this lib-call is broken, write is needed
	RSTREAS = 0xFF;

	setup_hw();
	
	printf("\r\nRunning: " __FILE__ ", build:" __DATE__ "\r\n");
	printf("reset reason: 0x%hhx\r\n", rr);

	//initialize spi and rf to all '1's
	memset(&tx_spipkt, 0xFF, sizeof(tx_spipkt));
	sspi_send_data(0xFF);
	spi_cnt = 1;
	memset(&rec_rfpkt, 0xFF, sizeof(rec_rfpkt));

	for(i=0; ; i++){

		if(get_latest_pkt(&rec_rfpkt)){
			if(rec_rfpkt.magic == STATUS_PACKET_MAGIC){
				printf("Status pkt %d received\r\n", rec_rfpkt.sequence_nr);
				if(++saved_seqnr != rec_rfpkt.sequence_nr){
					printf("WARNING: expected seq nr %d\r\n", saved_seqnr);
				}
				saved_seqnr = rec_rfpkt.sequence_nr++;

				printf("   trap %d, vdc %dmV\r\n",
					rec_rfpkt.status[0],
					adc_convert_to_mv(rec_rfpkt.vdc));

				if(saved_timeouts < rec_rfpkt.timeouts){
					printf("WARNING: peer timeout pkts increased %d->%d\r\n",
						saved_timeouts, rec_rfpkt.timeouts);
				}
				saved_timeouts = rec_rfpkt.timeouts;
			}else{
				printf("WARNING: corrupt packet received (magic 0x%08x)\r\n",
						rec_rfpkt.magic);
			}
			printf("\r\n");

			gpio_pin_val_write(GPIO_PIN_ID_P1_4, rec_rfpkt.status[0]);
		}

		//all data sent, copy new data to spi buffer 
		if(spi_cnt == sizeof(tx_spipkt)){
			prepare_spipkt(&tx_spipkt, &rec_rfpkt);
			spi_cnt = 0;
		}

		//if data has been sent on SPI, put in new data in FIFO
		if(SPISSTAT & SPISSTAT_INT_SPI_SLAVE_DONE_FLAG){
			uint8_t *p = (uint8_t*)&tx_spipkt; 
			sspi_send_data(p[spi_cnt]);
			spi_cnt++;
		}

		rpd += rf_is_rpd_active();

		if(i == 0){
			printf("Local info:\r\n   Current VDC: %dmV\r\n",
				adc_convert_to_mv(adc_start_single_conversion_get_value(ADC_CHANNEL_1_THIRD_VDD)));
			printf("   RF over -64dBm: %d times\r\n", rpd);
			printf("\r\n");
			rpd = 0;
		}

	}
}

void prepare_spipkt(struct spi_packet *spi, struct status_packet *rf)
{
	spi->seq = rf->sequence_nr;
	spi->lost = rf->timeouts;
	spi->trap = (uint8_t)rf->status[0];
	spi->vdc = rf->vdc;
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
	//setup P1.4 as output pin
	gpio_pin_configure(GPIO_PIN_ID_P1_4,
				GPIO_PIN_CONFIG_OPTION_DIR_OUTPUT |
				GPIO_PIN_CONFIG_OPTION_PIN_MODE_OUTPUT_BUFFER_NORMAL_DRIVE_STRENGTH);
	gpio_pin_val_write(GPIO_PIN_ID_P1_4, false);

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

	//setup spi, mode 0, msb and disable all interrupts
	sspi_configure(SSPI_CONFIG_OPTION_CPHA_SAMPLE_ON_MSCK_EDGE_LEADING | 
			SSPI_CONFIG_OPTION_CPOL_MSCK_ACTIVE_HIGH |
			SSPI_CONFIG_OPTION_DATA_ORDER_MSB_FIRST |
			SSPI_CONFIG_OPTION_SPI_SLAVE_DONE_INT_DISABLE |
			SSPI_CONFIG_OPTION_CSN_LOW_INT_DISABLE |
			SSPI_CONFIG_OPTION_CSN_HIGH_INT_DISABLE);
  	gpio_pin_configure(GPIO_PIN_ID_FUNC_SSCK,
				GPIO_PIN_CONFIG_OPTION_DIR_INPUT |
				GPIO_PIN_CONFIG_OPTION_PIN_MODE_INPUT_BUFFER_ON_NO_RESISTORS);
  	gpio_pin_configure(GPIO_PIN_ID_FUNC_SMOSI,
				GPIO_PIN_CONFIG_OPTION_DIR_INPUT |
				GPIO_PIN_CONFIG_OPTION_PIN_MODE_INPUT_BUFFER_ON_NO_RESISTORS);
  	gpio_pin_configure(GPIO_PIN_ID_FUNC_SMISO,
				GPIO_PIN_CONFIG_OPTION_DIR_OUTPUT |
				GPIO_PIN_CONFIG_OPTION_PIN_MODE_OUTPUT_BUFFER_NORMAL_DRIVE_STRENGTH);
  	gpio_pin_configure(GPIO_PIN_ID_FUNC_SCSN,
				GPIO_PIN_CONFIG_OPTION_DIR_INPUT |
				GPIO_PIN_CONFIG_OPTION_PIN_MODE_INPUT_BUFFER_ON_PULL_UP_RESISTOR);
	sspi_enable();	

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
		NULL,
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
