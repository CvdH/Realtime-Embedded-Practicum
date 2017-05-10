/*
 * opd1.c
 *
 *  Created on: Apr 28, 2016
 *      Author: john
 *
 * opd1.c
 *
 *  Created on: Apr 28, 2016
 *      Author: john
 */
 #ifndef F_CPU
 #define F_CPU 16000000UL
 #endif


#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>


#define BAUD 9600
#define MYUBRR (((F_CPU / (16UL * BAUD))) - 1)

#define RECEIVED_TRUE 1
#define RECEIVED_FALSE 0

void wait(unsigned int);
void UART_Init( unsigned int ubrr );
unsigned char UART_Receive( void );
void UART_Transmit( unsigned char data );

char ontvang; // global variable to store received data
int state = RECEIVED_FALSE;

int main() 
{
	unsigned int wacht = 10;

	DDRB = (1 << PB7); // arduino MEGA
	DDRB = (1 << PB0); //trigger pin
	//DDRB = (1 << PB5); // arduino UNO


	UART_Init(MYUBRR);

	while(1) 
	{
		PORTB ^= (1 << PB7); // arduino MEGA
		//PORTB ^= (1 << PB5); // arduino UNO

		//UART_Transmit('A');
		//ontvang = UART_Receive();
		if(state == RECEIVED_TRUE)
		{
			switch(ontvang)
			{
				case '1':
				{
					wacht = 10;
					break;
				}
				case '2':
				{
					wacht = 5;
					break;
				}
				case '3':
				{
					wacht = 1;
					break;
				}
			}
			state = RECEIVED_FALSE;
		}
		
		wait( wacht );
	}
}

void wait(unsigned int a)
 {
	while(a--)
		_delay_ms(50);
}


void UART_Init( unsigned int ubrr)
{
	/* Set baud rate */
	UBRR0H = (ubrr>>8);
	UBRR0L = ubrr;

	/* Enable receiver and transmitter and enable RX interrupt */
	UCSR0B = (1<<RXEN0) | (1<<TXEN0) | (1<<RXCIE0);
	/* Set frame format: 8data, 1 stop bit */
	UCSR0C = (3<<UCSZ00);
	sei();
}

void UART_Transmit( unsigned char data )
{
	/* Wait for empty transmit buffer */
	while ( !( UCSR0A & (1<<UDRE0)) )
	;
	/* Put data into buffer, sends the data */
	UDR0 = data;
}

unsigned char UART_Receive( void )
{
	/* Wait for data to be received */
	while ( !(UCSR0A & (1<<RXC0)) )
	;
	/* Get and return received data from buffer */
	return UDR0;
}

ISR(USART0_RX_vect)
{
	ontvang = UDR0;
	UART_Transmit(ontvang);
	state = RECEIVED_TRUE;
}
