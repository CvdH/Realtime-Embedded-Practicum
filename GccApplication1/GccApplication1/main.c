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

//pwm frequentie = clock/(2*prescalar*TOP)
//16 bit timer phase correct
//pre scalar 8
// top 20000
// 20000 = 20 ms
// 900   = 0.9ms = 0 graden
// 1500  = 1.5ms = 90 graden
// 2100  = 2.1ms = 180 graden

void timer1_init()
{
	TCCR1A |= (1 << WGM11);
	TCCR1B |= (1 << CS11) | (1 << WGM13);
	// initialize counter
	ICR1 = 20000;
	OCR1A = 18500;
	// enable overflow interrupt
	TIMSK1 |= (1 << 1);
	// enable global interrupts
	sei();
}

// called whenever TCNT1 overflows
ISR(TIMER1_COMPA_vect)
{
	PORTB ^= (1 << PB7);  // toggles the led
}

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

void turnServo(uint16_t degrees)
{	
	 OCR1A = 20000 - ((degrees * 1200 / 180) + 900);
}


int main() {
	sei();
	//initUSART(MYUBBR);
	timer1_init();
	uint8_t delay = 0;
	//set pin7 to output
	DDRB = (1 << PB7);	
	DDRA = (1 << PA7);

	while (1)
	{
		turnServo(90);
		wait(10);
		turnServo(120);
		wait(10);
/*		if (received)
		{
			switch (data)
			{
				case '1': delay = 10; break;
				case '2': delay = 5; break;
				case '3': delay = 1; break;
			}
			received = false;
		}
		
		//PORTB ^= (1 << PB7);
		//wait(delay);
		uint16_t count = 0;
		count = TCNT1H << 8;
		count += TCNT1L;
		if (count == 15625)
		{
			PORTB ^= (1 << PB7);
		}
*/
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

