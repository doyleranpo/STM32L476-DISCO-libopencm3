#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/scb.h>

#include "../include/main.h"
// 1us ticks
volatile uint32_t us_ticks = 0;

static void Clock_config(void) {
    rcc_osc_on(RCC_HSI16);

    flash_prefetch_enable();
	flash_set_ws(4);
	flash_dcache_enable();
	flash_icache_enable();

    rcc_set_main_pll(RCC_PLLCFGR_PLLSRC_HSI16, 4, 40, 0, 0, RCC_PLLCFGR_PLLQ_DIV2);
    rcc_osc_on(RCC_PLL);

    
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_GPIOD);
    rcc_periph_clock_enable(RCC_GPIOE);

    rcc_periph_clock_enable(RCC_USART2);

    rcc_set_sysclk_source(RCC_CFGR_SW_PLL);
    rcc_wait_for_sysclk_status(RCC_PLL);
    rcc_periph_clock_enable(RCC_SYSCFG);

    rcc_ahb_frequency = 80e6;
	rcc_apb1_frequency = 80e6;
	rcc_apb2_frequency = 80e6;
}

static void USART_config(void) {
    nvic_enable_irq(NVIC_USART2_IRQ);

    gpio_mode_setup(GPIOD, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO5|GPIO6);

    usart_set_baudrate(USART2, 115200);
    usart_set_databits(USART2, 8);
    usart_set_stopbits(USART2, USART_STOPBITS_1);
    usart_set_parity(USART2, USART_PARITY_NONE);
    usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);
    usart_set_mode(USART2, USART_MODE_TX_RX);

    USART_CR1(USART2) |= USART_CR1_RXNEIE;

    usart_enable(USART2);

}

void systick_config(void)
{
    systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);

    systick_set_frequency(1000000, 80e6);

    systick_interrupt_enable();

    systick_counter_enable();
}

void sys_tick_handler(void)
{
    /* Increment counter necessary in delay()*/
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

void usart2_isr(void) {
    static uint8_t data = 'a';

    if( ( (USART_CR1(USART2) & USART_CR1_RXNEIE) != 0) && ((USART_ISR(USART2) & USART_ISR_RXNE) != 0)) {
        data = usart_recv(USART2);

    }

    if (((USART_CR1(USART2) & USART_CR1_TXEIE) != 0) && ((USART_ISR(USART2) & USART_ISR_TXE) != 0)) {
        usart_send(USART2, data);

        USART_CR1(USART2) &= ~USART_CR1_TXEIE;
    }
}

int main(void) {
    
    Clock_config();
    USART_config();

    while(1)
        __asm__("nop");
    return 0;
}
