void clock_setup(void);
void systick_setup(void);
void delay_us(uint32_t dly_ticks);
void delay_ms(uint32_t ms_count);
void usart_setup(uint32_t usart, uint32_t baurdrate);
int _write(int file, char *ptr, int len);