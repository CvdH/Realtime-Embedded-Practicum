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
 
#define TRIGGER PB0
#define ECHO PB1

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <time.h>


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
	//unsigned int wacht = 10;
	//bool echo = false;

	DDRB = (1 << TRIGGER); //trigger pin
	TCCR1B |= (1 << CS10) | (1 << CS12);							//prescaler 1024
	TCNT1 = 0;

	UART_Init(MYUBRR);

	while(1) 
	{
		
		PORTB |= (1 << TRIGGER);										//zet trigger op 1
		_delay_us(10);													//wacht 10 microseconden
		PORTB &= ~(1 << TRIGGER);										//zet trigger op 0

		//clock_t begin = clock();										//start timer0, normal mode, 		
		while( !(PINB & (1 << ECHO))) {}								//wacht totdat echo 1 is
			

		//clock_t end = clock();											//start timer
																			//wacht totdat echo 0 is
																			//stop timer
		//double time_spent = (double)(end - begin) / CLOCKS_PER_SEC;		//bereken tijd
		double afstand; // = time_spent / 340 / 2;							//bereken afstand
		
		char* out[256];													//transmit
		sprintf(out, "%f",afstand);
		for(int i=0;i<sizeof(out);i++){
			UART_Transmit(out[i]);
		}
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
	sei(); // enable global interrupt
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
