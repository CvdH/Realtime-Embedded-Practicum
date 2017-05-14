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
//#define ECHO PB1

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#define BAUD 9600
#define MYUBRR (((F_CPU / (16UL * BAUD))) - 1)

#define RECEIVED_TRUE 1
#define RECEIVED_FALSE 0

#define INSTR_PER_US 16           // instructions per microsecond (depends on MCU clock, 16MHz current)
#define INSTR_PER_MS 16000        // instructions per millisecond (depends on MCU clock, 16MHz current)
#define MAX_RESP_TIME_MS 200      // timeout - max time to wait for low voltage drop (higher value increases measuring distance at the price of slower sampling)
#define DELAY_BETWEEN_TESTS_MS 50 // echo cancelling time between sampling

void wait(unsigned int);
void UART_Init( unsigned int ubrr );
unsigned char UART_Receive( void );
void UART_Transmit( unsigned char data );
void UART_Transmit_String(const char *stringPtr);

void INT1_init( void );
void pulse( void );
void timer1_init( void );

char ontvang; // global variable to store received data
int state = RECEIVED_FALSE;
//volatile uint8_t tot_overflow;
char out[256];	

volatile uint8_t running = 0; // state to see if the pulse has been send.
volatile uint8_t up = 0;
volatile uint32_t timerCounter = 0;
volatile unsigned long result = 0;

int main() 
{
	DDRB = (1 << TRIGGER); //trigger pin

	UART_Init(MYUBRR);
	INT1_init();
	timer1_init();
	sei();


	while(1) 
	{
		if(running == 0)
		{
			_delay_ms(DELAY_BETWEEN_TESTS_MS);
			pulse();
			sprintf(out, "Afstand = %dCM", result);
			UART_Transmit_String(out);
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
}

void UART_Transmit( unsigned char data )
{
	/* Wait for empty transmit buffer */
	while ( !( UCSR0A & (1<<UDRE0)) )
	;
	/* Put data into buffer, sends the data */
	UDR0 = data;
}

void UART_Transmit_String(const char *stringPtr)
{
	while(*stringPtr != 0x00)
	{
		UART_Transmit(*stringPtr);
		stringPtr++;
	}
	
	UART_Transmit('\n');
	UART_Transmit('\r');
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

void INT1_init()
{
	EICRA |= (1 << ISC10) | (0 << ISC11); // set rising or falling edge on INT1
	EIMSK |= (1 << INT1); //enable INT1
}
ISR(INT1_vect)
{
	if(running) // check if the pulse has been send.
	{
		if(up == 0) // voltage rise
		{
			up = 1;
			timerCounter = 0;
			TCNT1 = 0;
		}
		else // faling edge
		{
			up = 0;
			result = ((timerCounter * 65535 + TCNT1) / 58) / 2;
			running = 0;
		}
	}
}
void pulse()
{
	PORTB &= ~(1 << TRIGGER);
	_delay_us(1);


	PORTB |= (1 << TRIGGER);										//zet trigger op 1
	running = 1;
	_delay_us(10);													//wacht 10 microseconden
	PORTB &= ~(1 << TRIGGER);										//zet trigger op 0
}


void timer1_init()
{
	TCCR1B |= (0 << CS10) | (1 << CS11) | (0 << CS12);			//prescaler 1024 so we get 15625Hz clock ticks. or 4.19 s per tick.
	TCNT1 = 0;													//init counter
	TIMSK1 |= (1 << TOIE1);										//enable overflow interrupt
	//tot_overflow = 0;
}
ISR(TIMER1_OVF_vect)
{
	if(up)
	{
		timerCounter++;
		uint32_t ticks = timerCounter * 65535 + TCNT1;
		uint32_t maxTicks = (uint32_t)MAX_RESP_TIME_MS * INSTR_PER_MS;
		if(ticks > maxTicks)
		{
			up = 0;
			running = 0;
			result = -1;
		}
	}
}