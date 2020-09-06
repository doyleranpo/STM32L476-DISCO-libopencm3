/* USART Echo Module */
/* Transmit to serial what is received from serial */

/* This discovery board has USART2 routed to PD5 and PD6 */
/* PD5 and PD6 are passed through the on-board ST-Link as a COM Port */

/* Library includes */
#include <errno.h>
#include <stdio.h>
#include <unistd.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/usart.h>

#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>

/* Local includes */
#include "main.h"

/* Local defines */
#define GREEN_LED_PORT      GPIOE
#define GREEN_LED_PIN       GPIO8
#define RED_LED_PORT        GPIOB
#define RED_LED_PIN         GPIO2

#define DEBUG_USART         USART2

/* Global vars */
volatile uint32_t us_ticks = 0;         // 1us ticks

void clock_setup(void)
{
    // turn on the HSI Oscillator
    rcc_osc_on(RCC_HSI16);
    rcc_wait_for_osc_ready(RCC_HSI16);

    // set appropriate flash latency
    // Flash can't be read from at such high speeds
    // for 80MHz, CubeMX sets it to 4.
    flash_set_ws(4);

    // HSI is 16MHz
    // We will use the PLL to ramp this up to 80MHz
    rcc_set_main_pll(RCC_PLLCFGR_PLLSRC_HSI16, 2, 20,
                     0, 0, RCC_PLLCFGR_PLLR_DIV2);
    rcc_osc_on(RCC_PLL);
    rcc_wait_for_osc_ready(RCC_PLL);

    // switch system clock to PLL
    rcc_set_sysclk_source(RCC_CFGR_SW_PLL);
    rcc_wait_for_sysclk_status(RCC_PLL);

    // set global clock value vars
    rcc_ahb_frequency = 80e6;
	rcc_apb1_frequency = 80e6;
	rcc_apb2_frequency = 80e6;

    // enable clock for GPIOB and GPIOE where the user LEDs are
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_GPIOE);

    // enable clock for USART
    rcc_periph_clock_enable(RCC_GPIOD);
    rcc_periph_clock_enable(RCC_USART2);
}

void systick_setup(void)
{
    systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);

    // needed frequency for 1 microsecond ticks at 80MHz
    systick_set_frequency(1000000, 80e6);

    systick_interrupt_enable();

    systick_counter_enable();
}

void sys_tick_handler(void)
{
    // increment counter needed for delay()
    us_ticks++;
}

void delay_us(uint32_t dly_ticks)
{
      uint32_t cur_ticks;
 
      cur_ticks = us_ticks;
      while ((us_ticks - cur_ticks) < dly_ticks) ;
}

void delay_ms(uint32_t ms_count)
{
    uint32_t i;
    for (i = 0; i < ms_count; i++) {
        delay_us(1000);
    }
}

void usart_setup(uint32_t usart, uint32_t baurdrate)
{
    // Setup GPIO for USART
    gpio_mode_setup(GPIOD, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO5|GPIO6);
    gpio_set_af(GPIOD, GPIO_AF7, GPIO5|GPIO6);    // AF7 is USART

    // set USART config
    // standard 8N1 config
    usart_set_baudrate(usart, baurdrate);
    usart_set_databits(usart, 8);
    usart_set_stopbits(usart, USART_STOPBITS_1);
    usart_set_mode(usart, USART_MODE_TX_RX);
    usart_set_parity(usart, USART_PARITY_NONE);
    usart_set_flow_control(usart, USART_FLOWCONTROL_NONE);

    // enable the usart
    usart_enable(usart);

    // enable rx interrupt
    usart_enable_rx_interrupt(usart);

    // enable usart2 interrupt
    nvic_enable_irq(NVIC_USART2_IRQ);
}

// syscall to newlib
// borrowed from libopencm3 examples
// Github: libopencm3-examples/examples/stm32/l4/stm32l476g-disco/basics/basics.c 
int _write(int file, char *ptr, int len)
{
	int i;

	if (file == STDOUT_FILENO || file == STDERR_FILENO) {
		for (i = 0; i < len; i++) {
			if (ptr[i] == '\n') {
				usart_send_blocking(DEBUG_USART, '\r');
			}
			usart_send_blocking(DEBUG_USART, ptr[i]);
		}
		return i;
	}
	errno = EIO;
	return -1;
}

void usart2_isr(void)
{
	static uint8_t data;

	// Is it an rx interrupt?
	if (((USART_CR1(USART2) & USART_CR1_RXNEIE) != 0) &&
	    ((USART_ISR(USART2) & USART_ISR_RXNE) != 0)) {

		// get the data
		data = usart_recv(USART2);

		// enable tx interrupt so we can send the data back
		usart_enable_tx_interrupt(USART2);
	}

	// is it a tx interrupt?
	if (((USART_CR1(USART2) & USART_CR1_TXEIE) != 0) &&
	    ((USART_ISR(USART2) & USART_ISR_TXE) != 0)) {

		// put data into transmit register
		usart_send(USART2, data);

		// disable tx interrupt
        // we don't need it enabled since we are only data
        // back if we receive data
		usart_disable_tx_interrupt(USART2);
	}
}
int main(void)
{
    clock_setup();
    systick_setup();

    // setup debug usart
    usart_setup(DEBUG_USART, 115200);

    // set up GPIO LEDs
    gpio_mode_setup(GREEN_LED_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE,
                    GREEN_LED_PIN);
    gpio_mode_setup(RED_LED_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE,
                    RED_LED_PIN);

    while (1) {
        // keep LEDs blinking so that it shows that USART rx/tx does not
        // stop the processor from doing other tasks
        gpio_toggle(GREEN_LED_PORT, GREEN_LED_PIN);
        delay_ms(1000);
        gpio_toggle(RED_LED_PORT, RED_LED_PIN);
    }

    return 0;
}