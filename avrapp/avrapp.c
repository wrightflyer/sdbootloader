/*
 * avrapp.c
 *
 * Created: 19/10/2012 13:11:55
 *  Author: cliff
 */ 

#include <avr/io.h>
#include <util/delay.h>

int main(void)
{
	DDRD = 0xFF;
	PORTD = 0x55;
    while(1)
    {
		PORTD ^= 0xFF;
		_delay_ms(250);
    }
}