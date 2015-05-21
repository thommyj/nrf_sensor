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

#define RTC_FIRE_TIME		(2)
#define WAKEUP_TIME_DEBUG	(10)
#define WAKEUP_TIME		(30*60)
#define STATE_MAGIC		(0xAA12)

void setup_hw(void);
uint8_t init_state(uint8_t powerdown_reason);
uint8_t is_wakeuptime(void);
void powerdown(void);
void prepare_pkt(struct status_packet *pkt, uint8_t vdc_meas);
int send_pkt(struct status_packet *pkt);

struct state_t{
	uint16_t wakeups;
	uint16_t magic;
	uint16_t sent_pkts;
	uint16_t no_answer;
	uint16_t wakeups_before_run;
};

/* 
 * xdata between 0x0-0x1FF will be kept during sleep
 * sdcc will not allocate space for variables with __at attribute,
 * so makefile needs to set start of xram after this variable
 */
__xdata __at(0x0) struct state_t state;

struct status_packet packet;

/*
 * dont really do anything when rtc2 fire, just use as wakeup source*/
interrupt_isr_rtc2()
{
	/*NOP*/
}

void main()
{
	uint8_t vdc;
	uint8_t pr = PWRDWN; //TODO: doesn't seem to exist getters for PWRDWN? add
	uint8_t state_ok;

	state_ok = init_state(pr);

	if(!is_wakeuptime()){
		powerdown();
	}
	
	setup_hw();

	if(!state_ok){
		printf("\r\n\r\nWARNING: state reset. This is normal if this is powerup");
	}

	printf("\r\nRunning: " __FILE__ ", build:" __DATE__ "\r\n");
	printf("reset reason: 0x%hhx\r\n", pwr_clk_mgmt_get_reset_reason());
	printf("powerdown: 0x%hhx\r\n", pr);

	//pwr_clk_mgmt_clear_reset_reasons(); TODO: this lib-call is broken, write is needed
	RSTREAS = 0xFF;
	
	sti();

	vdc = adc_start_single_conversion_get_value(ADC_CHANNEL_1_THIRD_VDD);

	printf("VDC measure: %d\r\n", vdc);	
	
	prepare_pkt(&packet, vdc);
	if(!send_pkt(&packet)){
		state.no_answer++;
	}
	state.sent_pkts++;

	powerdown();
}

/*
 * start rtc2 and go to sleep and wake for it to fire
 * note that the retention latch is not set, we dont have
 * any GPIO worth saving
 */
void powerdown(void)
{
	//setup rtc2 here instead of in setup_hw, this avoids setting up hw each wakeup
	pwr_clk_mgmt_clklf_configure(PWR_CLK_MGMT_CLKLF_CONFIG_OPTION_CLK_SRC_RCOSC32K);
	rtc2_configure(RTC2_CONFIG_OPTION_DISABLE |
			RTC2_CONFIG_OPTION_COMPARE_MODE_0_RESET_AT_IRQ |
			RTC2_CONFIG_OPTION_DO_NOT_CAPTURE_ON_RFIRQ,
			0xFFFF);
	interrupt_control_rtc2_enable();
	
	while(1){
		rtc2_run();
		pwr_clk_mgmt_enter_pwr_mode_memory_ret_tmr_on();
		printf("waiting for sleep\r\n");
	}
}

/*
 * Expect wakeup beeing from a tick and that previous powerdown
 * mode was memory retention with timer on. If not, this is either
 * first start, or some other errnous state. Either way, the only
 * thing todo is to reset state. This is also done if magic word
 * in the state is wrong
 * Returns 1 if state was ok, 0 if reset
 */
uint8_t init_state(uint8_t powerdown)
{
	uint8_t prev_pd_reason = powerdown & PWRDWN_PWR_CNTL_MASK;
	uint8_t wakeup_from_tick = powerdown & PWRDWN_PWR_IS_WAKE_FROM_TICK;

	if (prev_pd_reason == PWR_CLK_MGMT_PWRDWN_MODE_MEMORY_RET_TMR_ON &&
	    wakeup_from_tick == PWRDWN_PWR_IS_WAKE_FROM_TICK &&
	    state.magic == STATE_MAGIC){
		state.wakeups++;
		return 1;	
	}else{
		memset(&state, 0x0, sizeof(state));
		state.magic = STATE_MAGIC;
		state.wakeups_before_run = check_wakeuptime();
		return 0;
	}
}

uint16_t check_wakeuptime(void)
{
	//setup P1.3 as input pin
	gpio_pin_configure(GPIO_PIN_ID_P1_3,
				GPIO_PIN_CONFIG_OPTION_DIR_INPUT |
				GPIO_PIN_CONFIG_OPTION_PIN_MODE_INPUT_BUFFER_ON_PULL_UP_RESISTOR);

	//if P1.3 is high, use normal time for debugging
	if(gpio_pin_val_read(GPIO_PIN_ID_P1_3)){
		return WAKEUP_TIME/RTC_FIRE_TIME;
	}else{
		return WAKEUP_TIME_DEBUG/RTC_FIRE_TIME;
	}
}

uint8_t is_wakeuptime(void)
{
	return (state.wakeups % state.wakeups_before_run)==0;
}

void setup_hw()
{

	//setup P1.4 as input pin
	gpio_pin_configure(GPIO_PIN_ID_P1_4,
				GPIO_PIN_CONFIG_OPTION_DIR_INPUT |
				GPIO_PIN_CONFIG_OPTION_PIN_MODE_INPUT_BUFFER_ON_PULL_DOWN_RESISTOR);
        
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

void prepare_pkt(struct status_packet *pkt, uint8_t vdc_meas)
{
	pkt->magic       = STATUS_PACKET_MAGIC;
	pkt->sequence_nr = state.sent_pkts; 
	pkt->wakeups     = state.wakeups;
	pkt->timeouts    = state.no_answer;
	pkt->vdc         = vdc_meas;
	pkt->version     = STATUS_PACKET_HDR_VER;
	pkt->status[0]   = gpio_pin_val_read(GPIO_PIN_ID_P1_4);
        pkt->status[1]   = P0; //TODO: move out port reads
        pkt->status[2]   = P1;
        pkt->status[3]   = 0;
}

int send_pkt(struct status_packet *pkt)
{
	uint8_t pkt_sent = 0;
	uint8_t status;
	
	//TODO: calculate reasonable ARD
	//TODO: gain control?
	rf_configure(RF_CONFIG_MASK_RX_DR |
			RF_CONFIG_EN_CRC |
			RF_CONFIG_CRCO |
                        RF_CONFIG_PWR_UP,
                false,                   /* take rx to active */
                RF_EN_AA_ENAA_P0,       /* enable enhanced shochburst channel 0 */
                RF_EN_RXADDR_ERX_P0,    /* enable channel 0 */
                RF_SETUP_AW_5BYTES,     /* use 5byte address */
                RF_SETUP_RETR_ARD_4000 | RF_SETUP_RETR_ARC_15,
                RF_RF_CH_DEFAULT_VAL,   /* use default channel */
                RF_RF_SETUP_RF_DR_1_MBPS | RF_RF_SETUP_RF_PWR_0_DBM,
                server_address,
                NULL, 0, 0, 0, 0,       /* not interested in other channel addresses*/
                client0_address,
                sizeof(struct status_packet),  /* size of expected packet */
                0,0,0,0,0,
                RF_DYNPD_DPL_NONE,      /* no dynamic payload size */
                0);

	rf_write_tx_payload((const uint8_t*)pkt, sizeof(struct status_packet), true); //transmit received char over RF

	//wait until the packet has been sent or the maximum number of retries has been reached
	//TODO: timeout?
	while(!rf_irq_pin_active()){
		/*NOP*/
	}

	status = rf_get_status();
	printf("send_packet: irq active, status 0x%02X\r\n", status);
	if(rf_is_tx_ds_active_in_status_val(status)){
		pkt_sent = 1;
	}
	rf_irq_clear_all(); //clear all interrupts in the 24L01

	rf_power_down();

	return pkt_sent;
}


void putchar(char c)
{
        uart_send_wait_for_complete(c);
}

char getchar()
{
        return uart_wait_for_rx_and_get();
}
