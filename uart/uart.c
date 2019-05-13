#include "uart.h"

void UART_init(void) {
	UCSRB = (1 << TXEN);
	UBRRL = 5; // 38400 @ 3.6864MHz
}

void UART_put(uint8_t c) {
	while (!(UCSRA & (1 << UDRE)));
	UDR = c;
}

void UART_puts(const char * str) {
	while (*str) {
		UART_put(*str++);
	}
}

void UART_newline(void){
	UART_put('\r');
	UART_put('\n');
}

void UART_putnibble(uint8_t c) {
	if (c < 10) {
		UART_put('0' + c);
	}
	else {
		UART_put('A' + c - 10);
	}
}

void UART_puthex(uint8_t c) {
	UART_putnibble(c >> 4);
	UART_putnibble(c & 0x0F);
}

void UART_puthex16(uint16_t n) {
	UART_puthex(n >> 8);
	UART_puthex(n & 0xFF);
}

void UART_putsP(const char * str, uint16_t n) {
	while (pgm_read_byte(str) != 0) {
		UART_put(pgm_read_byte(str++));
	}
	UART_puthex16(n);
	UART_newline();
}

void UART_dumpsector(uint8_t * Buff) {
	for (uint16_t i=0; i<512; i++) {
		if ((i % 16) == 0) {
			UART_put(' ');
			for(uint16_t j=(i -16); j<=i; j++) {
				UART_put(((Buff[j]>=(uint8_t)32) && (Buff[j]<(uint8_t)127)) ? Buff[j] : '.');
			}
			UART_newline();
		}
		UART_puthex(Buff[i]);
		UART_put(' ');
	}
	UART_newline();
}