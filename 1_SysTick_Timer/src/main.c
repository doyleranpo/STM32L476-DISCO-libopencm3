#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>

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
    rcc_periph_clock_enable(RCC_GPIOE);

    rcc_set_sysclk_source(RCC_CFGR_SW_PLL);
    rcc_wait_for_sysclk_status(RCC_PLL);
    rcc_periph_clock_enable(RCC_SYSCFG);

    rcc_ahb_frequency = 80e6;
	rcc_apb1_frequency = 80e6;
	rcc_apb2_frequency = 80e6;
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

int main(void) {
    Clock_config();
    systick_config();

    gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO2);
    gpio_mode_setup(GPIOE, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO8);

    while(1) {
        gpio_toggle(GPIOB, GPIO2);

        delay_ms(1000);

        gpio_toggle(GPIOE, GPIO8);
    }
    return 0;
}