#include "auxiliary.h"
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/usart.h>


#include <errno.h>
#include <unistd.h>
#include <stdio.h>

static uint32_t DELAY_TIMER = 0x0;
static uint32_t USART_CONSOLE = 0x0;

int _write(int file, char *ptr, int len)
{
	int i;

	if (file == STDOUT_FILENO || file == STDERR_FILENO) {
		for (i = 0; i < len; i++) {
			if (ptr[i] == '\n') {
				usart_send_blocking(USART_CONSOLE, '\r');
			}
			usart_send_blocking(USART_CONSOLE, ptr[i]);
		}
		return i;
	}
	errno = EIO;
	return -1;
}

static void timer_setup(void) {

	//  Edge-aligned mode. The counter counts only up, one-pulse mode
    TIM_CR1(DELAY_TIMER) = TIM_CR1_DIR_UP | TIM_CR1_CMS_EDGE;

	// /* Prescaler */
    TIM_PSC(DELAY_TIMER) = rcc_apb1_frequency/1000000;
}

void delay_us(const uint32_t us) {

    TIM_ARR(DELAY_TIMER) = us<<1;
	TIM_EGR(DELAY_TIMER) = TIM_EGR_UG;
    TIM_CNT(DELAY_TIMER) = 0x0;

    // reset status register
    TIM_SR(DELAY_TIMER) = 0x0;

    // Counter enable
    TIM_CR1(DELAY_TIMER) |= TIM_CR1_CEN;

    while(!(TIM_SR(DELAY_TIMER) & 0x1));
    TIM_SR(DELAY_TIMER) = 0x0;
    TIM_CR1(DELAY_TIMER) &= ~(0x1UL);
}

static void usart_setup(void) {

	usart_set_baudrate(USART_CONSOLE, 115200);
	usart_set_databits(USART_CONSOLE, 8);
	usart_set_stopbits(USART_CONSOLE, USART_STOPBITS_1);
	usart_set_mode(USART_CONSOLE, USART_MODE_TX);
	usart_set_parity(USART_CONSOLE, USART_PARITY_NONE);
	usart_set_flow_control(USART_CONSOLE, USART_FLOWCONTROL_NONE);

	/* Finally enable the USART. */
	usart_enable(USART_CONSOLE);
}

uint8_t init_auxiliary(const uint32_t delay_timer, const uint32_t usart_console) {
    DELAY_TIMER = delay_timer;
	USART_CONSOLE = usart_console;
	usart_setup();
    timer_setup();
    return 0;
}

