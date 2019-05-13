/*
 * avrapp.c
 *
 * Created: 19/10/2012 13:11:55
 *  Author: cliff
 */ 

#include <avr/io.h>
#include <util/delay.h>
#include <avr/pgmspace.h>
#include "uart.h"

int main(void)
{
	UART_init();
	UART_putsP(PSTR("App at 0\r\n"));
	DDRD = 0xFF;
	PORTD = 0x55;
    while(1)
    {
		PORTD ^= 0xFF;
		_delay_ms(80);
    }
}