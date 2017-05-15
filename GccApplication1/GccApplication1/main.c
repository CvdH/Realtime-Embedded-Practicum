/*
 * GccApplication1.c
 *
 * Created: 26-4-2017 10:13:10
 * Author : Benjamin
 */ 
#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdbool.h>

#define BAUD 9600
#define MYUBBR (F_CPU/16/BAUD-1)

void wait(unsigned);

bool received = false;
uint8_t data = 0;

void initUSART(uint16_t ubbr)
{
	//set prescalar/baudrate
	UBRR0H = (uint8_t) (ubbr >> 8);
	UBRR0L = (uint8_t) ubbr;
	//enable receiver and transmitter and enable interrupt
	UCSR0B = (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0);
	//set frame format
	UCSR0C = (1 << USBS0) | (3 << UCSZ00);
}

/*unsigned char USART_Receive(void)
{
	while (!(UCSR0A & (1 << RXC0)));
	
	return UDR0;
}
*/

int main() {
	sei();
	initUSART(MYUBBR);
	uint8_t delay = 0;
	//set pin7 to output
	DDRB = (1 << PB7);

	while (1)
	{
		if (received)
		{
			switch (data)
			{
				case '1': delay = 10; break;
				case '2': delay = 5; break;
				case '3': delay = 1; break;
			}
			received = false;
		}
		
		PORTB ^= (1 << PB7);
		wait(delay);
	}
}

void wait(unsigned a) {
	while(a--)
	_delay_ms(100);
}

ISR(USART0_RX_vect)
{
	data = UDR0;
	received = true;
}