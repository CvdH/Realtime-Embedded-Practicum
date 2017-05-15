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

#define BAUD 9600
#define MYUBBR (F_CPU/16/BAUD-1)

void wait(unsigned);

void initUSART(uint16_t ubbr)
{
	//set prescalar/baudrate
	UBRR0H = (uint8_t) (ubbr >> 8);
	UBRR0L = (uint8_t) ubbr;
	//enable receiver and transmitter
	UCSR0B = (1 << RXEN0) | (1 << TXEN0);
	//set frame format
	UCSR0C = (1 << USBS0) | (3 << UCSZ00);
}

unsigned char USART_Receive(void)
{
	while (!(UCSR0A & (1 << RXC0)));
	
	return UDR0;
}

int main() {
	initUSART(MYUBBR);
	uint8_t frequentie = USART_Receive();
	uint8_t delay = 0;
	switch (frequentie)
	{
		case '1': delay = 10; break;
		case '2': delay = 5; break;
		case '3': delay = 1; break;
	}
	//set pin7 to output
	DDRB = (1 << PB7);

	while(1) {
		//invert pin PB
		PORTB ^= (1 << PB7);
		wait(delay);
	}
}

void wait(unsigned a) {
	while(a--)
	_delay_ms(100);
}
