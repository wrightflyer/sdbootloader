#include <avr/io.h>
#include <avr/pgmspace.h>

void UART_init(void);
void UART_put(uint8_t c);
void UART_puts(const char * str);
void UART_putsP(const char * str);
void UART_putnibble(uint8_t c);
void UART_puthex(uint8_t c);
void UART_newline(void);
void UART_dumpsector(uint8_t * Buff);
