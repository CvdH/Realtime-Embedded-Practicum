/*
 * opd1.c
 *
 *  Created on: Apr 28, 2016
 *      Author: Marcel Vincourt, Casper van der Hout
 */
 //#ifndef F_CPU
 #define F_CPU 16000000UL
 //#endif
 

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h" 
#include "queue.h"
#include "semphr.h" 
#include "serial.h"

//Start sonar stuff
#define BAUD 9600
#define MYUBRR (((F_CPU / (16UL * BAUD))) - 1)
#define TRIGGER PB0
#define GELUID PB1

#define RECEIVED_TRUE 1
#define RECEIVED_FALSE 0

#define INSTR_PER_US 16												// instructions per microsecond (depends on MCU clock, 16MHz current)
#define INSTR_PER_MS 16000											// instructions per millisecond (depends on MCU clock, 16MHz current)
#define MAX_RESP_TIME_MS 200										// timeout - max time to wait for low voltage drop
#define DELAY_BETWEEN_TESTS_MS 50									// echo cancelling time between sampling

void INT1_init( void );
void pulse( void );
void timer3_init( void );

char ontvang;														// Global variable to store received data
int state = RECEIVED_FALSE;
char out[256];

volatile uint8_t running = 0;										// State to see if the pulse has been send.
volatile uint8_t up = 0;
volatile uint32_t timerCounter = 0;
volatile unsigned long result = 0;
//end sonar stuff

//start servo stuff
//end servo stuff

//start main stuff
QueueHandle_t sonarAfstand;
QueueHandle_t servoHoek;
void initQ();
void readQ();
void writeQ();
void queueTaak();
void sonarTaak();
void servoTaak();
void UART_Init( unsigned int ubrr );
void UART_Transmit( unsigned char data );
void UART_Transmit_String(const char *stringPtr);
void turnServo(uint8_t degrees);
void initServo();

int afstand;
int hoek;
//end main stuff

int main() 
{
	DDRB |= (1 << TRIGGER);											// Trigger pin
	DDRB &= ~(1 << GELUID);
	UART_Init(MYUBRR);
	INT1_init();
	timer3_init();
	initServo();
	sei();
	initQ();
	UART_Transmit_String("Setup done");

	xTaskCreate(queueTaak,"Queue Taken",256,NULL,3,NULL);			//task voor lezen uit sonar queue en schrijven naar servo queue
	xTaskCreate(sonarTaak,"Sonar Sensor",256,NULL,3,NULL);			//lees sonar sensor uit en schrijf afstand naar sonar queue
	xTaskCreate(servoTaak,"Servo Motor",256,NULL,3,NULL);			//code van Joris & Benjamin 

	vTaskStartScheduler();
}

void sonarTaak()
{
	//UART_Transmit_String("taak uitgevoerd");
	//wdt_enable(WDTO_2S);											// enable watchdog timer at 2 seconds
	while(1)
	{
		if(running == 0)
		{
			_delay_ms(DELAY_BETWEEN_TESTS_MS);
			pulse();
			xQueueSend(sonarAfstand, (void*) &result,0);
			//UART_Transmit_String("loop");
			//wdt_reset();
		}
	}
}


void servoTaak()
{
	while(1)
	{
		turnServo(afstand);
	}
}

void queueTaak()
{
	while(1){
		//_delay_ms(DELAY_BETWEEN_TESTS_MS);
		readQ();
		if (afstand > 300) {
			afstand = 300;
		}
		hoek = afstand / 300.0 * 180.0;

		if(! (PINB & (1 << PB1))) //If soud is pressed
		{
			sprintf(out, "Afstand = %dCM, Hoek = %d, Geluid = JA", afstand,hoek);
		}
		else
		{
			sprintf(out, "Afstand = %dCM, Hoek = %d, Geluid = NEIN!!!", afstand,hoek);
		}
		UART_Transmit_String(out);
		writeQ();
	}
}

void wait(unsigned int a)
{
	while(a--)
		_delay_ms(50);
}

//main functions
void initQ(){
	sonarAfstand = xQueueCreate(10,sizeof(int));
	if(sonarAfstand==0) sonarAfstand = xQueueCreate(10,sizeof(int));

	servoHoek = xQueueCreate(10,sizeof(int));
	if(servoHoek==0) servoHoek = xQueueCreate(10,sizeof(int));
}

void writeQ( int data)
{
	xQueueSend(servoHoek, (void*) &hoek, 0);
}

void readQ()
{
	xQueueReceive(sonarAfstand, &afstand, 0);
}

void UART_Init( unsigned int ubrr)
{
	UBRR0H = (ubrr>>8);												// Set baud rate
	UBRR0L = ubrr;

	UCSR0B = (1<<RXEN0) | (1<<TXEN0) | (1<<RXCIE0);					// Enable receiver and transmitter and enable RX interrupt
	UCSR0C = (3<<UCSZ00);											// Set frame format: 8data, 1 stop bit
}

void UART_Transmit( unsigned char data )
{
	while ( !( UCSR0A & (1<<UDRE0)) ) {};							// Wait for empty transmit buffer
	UDR0 = data;													// Put data into buffer, sends the data
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

ISR(USART0_RX_vect)
{
	ontvang = UDR0;
	UART_Transmit(ontvang);
	state = RECEIVED_TRUE;
}

//Sonar functions
void INT1_init()
{
	EICRA |= (1 << ISC10) | (0 << ISC11);							// set rising or falling edge on INT1
	EIMSK |= (1 << INT1);											// Enable INT1
}

ISR(INT1_vect)
{
	if(running)														// check if the pulse has been send.
	{
		if(up == 0)													// voltage rise
		{
			up = 1;
			timerCounter = 0;
			TCNT3 = 0;
		}
		else														// faling edge
		{
			up = 0;
			result = ((timerCounter * 65535 + TCNT3) / 58) / 2;
			running = 0;
		}
	}
}

void pulse()
{
	PORTB &= ~(1 << TRIGGER);
	_delay_us(1);


	PORTB |= (1 << TRIGGER);										// zet trigger op 1
	running = 1;
	_delay_us(10);													// wacht 10 microseconden
	PORTB &= ~(1 << TRIGGER);										// zet trigger op 0
}

void timer3_init()
{
	TCCR3B |= (0 << CS30) | (1 << CS31) | (0 << CS32);				// prescaler 1024 so we get 15625Hz clock ticks. or 4.19 s per tick.
	TCNT3 = 0;														// init counter
	TIMSK3 |= (1 << TOIE3);											// enable overflow interrupt
}

ISR(TIMER3_OVF_vect)
{
	if(up)
	{
		timerCounter++;
		uint32_t ticks = timerCounter * 65535 + TCNT3;
		uint32_t maxTicks = (uint32_t)MAX_RESP_TIME_MS * INSTR_PER_MS;
		if(ticks > maxTicks)
		{
			up = 0;
			running = 0;
			result = -1;
		}
	}
}

void turnServo(uint8_t degrees)
{
	OCR1B = 20000 - (degrees * (1300 / 180) + 800);
}

void initServo()
{
	DDRB |= (1 << PB6);
	TCCR1A = (1 << WGM11) | (1 << COM1B0) | (1 << COM1B1);
	TCCR1B = (1 << WGM13) | (1 << CS11);
	ICR1 = 20000;
	TCNT1 = 0;
	turnServo(0);
}