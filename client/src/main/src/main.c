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


uint16_t check_sendpkttime(void);
void setup_hw(void);
bool init_state(uint8_t powerdown_reason);
bool is_sendpkttime(void);
void powerdown(void);
void prepare_pkt(struct status_packet *pkt, uint8_t vdc_meas, bool trap_active);
bool send_pkt(struct status_packet *pkt);
bool is_p14_active(void);

struct state_t{
	uint16_t wakeups;
	uint16_t magic;
	uint16_t sent_pkts;
	uint16_t no_answer;
	uint16_t wakeups_before_run;
	bool	saved_trap_active;
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
	bool	trap_active;

	state_ok = init_state(pr);
	trap_active = is_p14_active();

	if(!is_sendpkttime() &&
	   trap_active == state.saved_trap_active &&
	    state_ok){
		powerdown();
	}
	
	setup_hw();

	if(!state_ok){
		printf("\r\n\r\nWARNING: state reset. This is normal if this is powerup");
	}

	printf("\r\nBuild date: " __DATE__ "\r\n");
	printf("reset: 0x%hhx, pd 0x%hhd\r\n", pwr_clk_mgmt_get_reset_reason(), pr);
	printf("trap %d (%d)\r\n", trap_active, state.saved_trap_active);
	printf("wakeups %d, pkts %d (%d)\r\n", state.wakeups, state.sent_pkts, state.no_answer);	

	//pwr_clk_mgmt_clear_reset_reasons(); TODO: this lib-call is broken, write is needed
	RSTREAS = 0xFF;
	
	sti();

	vdc = adc_start_single_conversion_get_value(ADC_CHANNEL_1_THIRD_VDD);

	printf("VDC measure: %d\r\n", vdc);	
	
	prepare_pkt(&packet, vdc, trap_active);
	if(!send_pkt(&packet)){
		printf("WARNING: no answer when sending packet\r\n");
		state.no_answer++;
	}else{
		//only save state if pkt was succesfully send
		state.saved_trap_active = trap_active;
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
bool init_state(uint8_t powerdown)
{
	uint8_t prev_pd_reason = powerdown & PWRDWN_PWR_CNTL_MASK;
	uint8_t wakeup_from_tick = powerdown & PWRDWN_PWR_IS_WAKE_FROM_TICK;

	if (prev_pd_reason == PWR_CLK_MGMT_PWRDWN_MODE_MEMORY_RET_TMR_ON &&
	    wakeup_from_tick == PWRDWN_PWR_IS_WAKE_FROM_TICK &&
	    state.magic == STATE_MAGIC){
		state.wakeups++;
		return true;	
	}else{
		memset(&state, 0x0, sizeof(state));
		state.magic = STATE_MAGIC;
		state.wakeups_before_run = check_sendpkttime();
		state.saved_trap_active= false;
		return false;
	}
}

uint16_t check_sendpkttime(void)
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

/*
 * time to send pkt if enough wakeups
 */
bool is_sendpkttime(void)
{
	return (state.wakeups % state.wakeups_before_run) == 0;
}

/*
 * trap is connected to P1.4
 */
bool is_p14_active(void)
{
	//setup P1.4 as input pin
	gpio_pin_configure(GPIO_PIN_ID_P1_4,
				GPIO_PIN_CONFIG_OPTION_DIR_INPUT |
				GPIO_PIN_CONFIG_OPTION_PIN_MODE_INPUT_BUFFER_ON_PULL_DOWN_RESISTOR);

	return gpio_pin_val_read(GPIO_PIN_ID_P1_4);
}

/*
 * Init devices and pins. Initialization of devices needed before
 * it is known if this startup should send a packet, is done in
 * method that needs it. This due to energy saving
 */
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

void prepare_pkt(struct status_packet *pkt, uint8_t vdc_meas, bool trap_active)
{
	pkt->magic       = STATUS_PACKET_MAGIC;
	pkt->sequence_nr = state.sent_pkts; 
	pkt->wakeups     = state.wakeups;
	pkt->timeouts    = state.no_answer;
	pkt->vdc         = vdc_meas;
	pkt->version     = STATUS_PACKET_HDR_VER;
	pkt->status[0]   = trap_active;
        pkt->status[1]   = P0; //TODO: move out port reads
        pkt->status[2]   = P1;
        pkt->status[3]   = 0;
}

bool send_pkt(struct status_packet *pkt)
{
	bool pkt_sent = false;
	uint8_t status;
	uint16_t timeout;
	
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
                client0_address,	/* pipe 0 address */
                NULL, 0, 0, 0, 0,       /* not interested in other channel addresses*/
                client0_address,        /* tx address, only one address for one connection, so same as rx */
                sizeof(struct status_packet),  /* size of expected packet */
                0,0,0,0,0,
                RF_DYNPD_DPL_NONE,      /* no dynamic payload size */
                0);

	rf_write_tx_payload((const uint8_t*)pkt, sizeof(struct status_packet), true); //transmit received char over RF

	/*
	 * Wait until the packet has been sent or the maximum number of retries has been reached
	 * use polling of status to see when ACK has been received
	 */
	for(timeout = 0; ; timeout++){
		status = rf_get_status();
		if(rf_is_tx_ds_active_in_status_val(status)){
			pkt_sent = true;
			break;
		}else if(rf_is_max_rt_active_in_status_val(status)){
			pkt_sent = false;
			printf("WARNING: max rt active 0x%02X, timeout %d\r\n", status, timeout);
			break;
	 	/* 
		 * ARD 4000us, max 15 tries => 4000*14=56ms max time
		 * So it should be safe to say that we shouldn't get stuck
		 * in this loop for more than 56*2ms
		 * rf_get_status seems to take ~38.5us, and each loop has
		 * an additional 5us delay. => 112*1000/(38.5+5)~2574 spins
 		 */
		}else if(timeout == 2600){
			pkt_sent = false;
			printf("WARNING: timeout reached, status 0x%02X\r\n", status);
			break;
		}

		delay_us(5);
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
