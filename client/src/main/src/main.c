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

#define RTC_FIRE_TIME		(2)
#define WAKEUP_TIME		(120)
#define WAKEUPS_BEFORE_RUN 	(WAKEUP_TIME/RTC_FIRE_TIME)
#define STATE_MAGIC		(0xAA12)

void setup_hw(void);
uint8_t check_state(uint8_t powerdown_reason);
uint8_t is_wakeuptime(void);
void powerdown(void);

struct state_t{
	uint16_t wakeups;
	uint16_t magic;
};

/* 
 * xdata between 0x0-0x1FF will be kept during sleep
 * sdcc will not allocate space for variables with __at attribute,
 * put this variable in the end and hope for the best
 */
__xdata __at(0x1F0) struct state_t state;

interrupt_isr_pwr_fail()
{
	int i;
	printf("power fail");
	for(i=0;i<10;i++){
		printf(", %d", adc_start_single_conversion_get_value(ADC_CHANNEL_1_THIRD_VDD));
	}
	for(;;){
		/*NOP*/
	}

}

/*
 * dont really do anything when rtc2 fire, just use as wakeup source*/
interrupt_isr_rtc2()
{
	/*NOP*/
}

void main()
{
	int i;
	uint8_t rr = pwr_clk_mgmt_get_reset_reason();
	uint8_t pr = PWRDWN; //TODO: doesn't seem to exist getters for PWRDWN? add

	//pwr_clk_mgmt_clear_reset_reasons(); TODO: this lib-call is broken, write is needed
	RSTREAS = 0xFF;

	//TODO: seperate hw setup. Not everything needs to be setup if we are about to sleep	
	setup_hw();
	
	if(!check_state(pr)){
		printf("\r\n\r\nWARNING: state reset. This is normal if this is powerup");
	}else{
		printf(".");
	}

	if(!is_wakeuptime()){
		powerdown();
	}
	
	printf("\r\nRunning: " __FILE__ ", build:" __DATE__ "\r\n");
	printf("reset reason: 0x%hhx\r\n", rr);
	printf("powerdown: 0x%hhx\r\n", pr);

	sti();

	printf("VDC measure: %d\r\n",
		adc_start_single_conversion_get_value(ADC_CHANNEL_1_THIRD_VDD));

	powerdown();
}

/*
 * start rtc2 and go to sleep and wake for it to fire
 * note that the retention latch is not set, we dont have
 * any GPIO worth saving
 */
void powerdown(void)
{
	rtc2_run();
	pwr_clk_mgmt_enter_pwr_mode_memory_ret_tmr_on();
	while(1){
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
uint8_t check_state(uint8_t powerdown)
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
		return 0;
	}
}

uint8_t is_wakeuptime(void)
{
	return (state.wakeups % WAKEUPS_BEFORE_RUN)==0;
}

void setup_hw()
{
	//setup rtc2
	pwr_clk_mgmt_clklf_configure(PWR_CLK_MGMT_CLKLF_CONFIG_OPTION_CLK_SRC_RCOSC32K);
	rtc2_configure(RTC2_CONFIG_OPTION_DISABLE |
			RTC2_CONFIG_OPTION_COMPARE_MODE_0_RESET_AT_IRQ |
			RTC2_CONFIG_OPTION_DO_NOT_CAPTURE_ON_RFIRQ,
			0xFFFF);
	interrupt_control_rtc2_enable();
        
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

	//setup early brown out detection
	pwr_clk_mgmt_pwr_failure_configure(PWR_CLK_MGMT_PWR_FAILURE_CONFIG_OPTION_POF_THRESHOLD_2_7V |
						PWR_CLK_MGMT_PWR_FAILURE_CONFIG_OPTION_POF_ENABLE);
	interrupt_control_pwr_fail_enable();
}

void putchar(char c)
{
        uart_send_wait_for_complete(c);
}

char getchar()
{
        return uart_wait_for_rx_and_get();
}
