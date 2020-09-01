#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/flash.h>

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

int main(void) {
    Clock_config();

    gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO2);
    gpio_mode_setup(GPIOE, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO8);

    while(1) {
        gpio_toggle(GPIOB, GPIO2);

        for (int i = 0; i < 4000000; i++)
            __asm__("NOP"); 

        gpio_toggle(GPIOE, GPIO8);
    }
    return 0;
}