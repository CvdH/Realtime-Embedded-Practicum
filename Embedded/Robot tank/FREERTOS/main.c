/*
 * opd1.c
 *
 *  Created on: Apr 28, 2016
 *      Author: MMMMMarcel Vincourt, Casper van der Hout, Joris van Tets, Benjamin van Vuuren
 */
 //#ifndef F_CPU
 #define F_CPU 16000000UL
 //#endif
 
#include <string.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

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

#define L_PLUS PC1
#define L_MIN PD7
#define L_EN PL5

#define R_PLUS PG1
#define R_MIN PL7
#define R_EN PL3
#define SERVO_PIN PL1

#define L_GELUID PA3
#define R_GELUID PA1
#define PIN_GELUID PINA

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

//start main stuff
//QueueHandle_t sonarAfstand;
//QueueHandle_t servoHoek;
QueueHandle_t motorCmd;
QueueHandle_t sonarResult;
void initQ();
void sonarTaak();
void servoTaak();
void motorTaak();
void soundTaak();
void UART_Init( unsigned int ubrr );
void UART_Transmit( unsigned char data );
void UART_Transmit_String(const char *stringPtr);
void turnServo(uint16_t degrees);
void initServo();
void initMotor();
void setSpeed(uint8_t speed);
void vooruit();
void achteruit();
void rechts();
void links();
void enable();
void M_stop();

void TaskWheels( void *pvParameters );

int afstand;
int hoek;
//end main stuff
char send[20];

bool meetSonar = false;
bool servoMeet = false;

TaskHandle_t geluidHandle;

int main() 
{
	DDRB |= (1 << TRIGGER);											// Trigger pin
	//DDRB &= ~(1 << GELUID);
	initMotor();
	UART_Init(MYUBRR);
	INT1_init();
	timer3_init();
	initServo();
	sei();
	initQ();
	UART_Transmit_String("Setup done");

	

	xTaskCreate(motorTaak, "Motor Taak", 256, NULL, 3, NULL);
	xTaskCreate(soundTaak, "Geluid taak", 256, NULL, 3, &geluidHandle);
	xTaskCreate(sonarTaak,"Sonar Sensor",256,NULL,3,NULL);			//lees sonar sensor uit en schrijf afstand naar sonar queue
	xTaskCreate(servoTaak,"Servo Taak",256,NULL,3,NULL);			//code van Joris & Benjamin 

	vTaskStartScheduler();
}

void motorTaak()
{
	vTaskSuspend( geluidHandle );

	uint8_t temp;
	int16_t speed = 0;
	int16_t currentSpeed = 0;
	bool riding = false;
	setSpeed(speed);
	while (1)
	{
		if (xQueueReceive(motorCmd, &temp, 0))
		{
			UART_Transmit(temp);
			switch(temp)
			{
				case 'w':
					currentSpeed = 0;
					setSpeed(currentSpeed);
					riding = true;
					meetSonar = true;
					M_stop();
					vooruit();
					break;
				case 'a':
					riding = true;
					meetSonar = false;
					M_stop(); 
					links();
					break;
				case 's':
					currentSpeed = 0;
					setSpeed(currentSpeed);
					riding = true;
					meetSonar = false;
					M_stop(); 
					achteruit();
					break;
				case 'd':
					riding = true;
					meetSonar = false;
					M_stop();
					rechts();
					break;
				case 'x':
					riding = false;
					meetSonar = false;
					M_stop();  
					break;
				case 'r': 
					meetSonar = false;
					servoMeet = true;
					M_stop(); 
					break;
				case 'A':
					//currentSpeed = 100;
					//setSpeed(currentSpeed);
					vTaskSuspend( geluidHandle );
					meetSonar = false;
					//M_stop();
					links();
					vTaskDelay(100);
					M_stop();
					
					break;
				case 'D':
					//currentSpeed = 100;
					//setSpeed(currentSpeed);
					vTaskSuspend( geluidHandle );
					meetSonar = false;
					//M_stop();
					rechts();
					vTaskDelay(100);
					M_stop();
					
					break;
				case '+':
					speed += 10;
					if (speed > 100)
						speed = 100;
					break;
				case '-':
					speed -= 10;
					if (speed < 0)
						speed = 0;
					break;
				case 'g':
					currentSpeed = 100;
					setSpeed(currentSpeed);
					riding = false;
					vTaskResume( geluidHandle );
					M_stop();
					break;
			}
		}

		if (riding)
		{
			if (currentSpeed <= speed)
			{
				currentSpeed += 10;
				if (currentSpeed > 100)
					currentSpeed = 100;
				setSpeed(currentSpeed);
				vTaskDelay(25);
			}
			else if(currentSpeed >= speed)
			{
				currentSpeed -= 10;
				if (currentSpeed < 0)
					currentSpeed = 0;
				setSpeed(currentSpeed);
				vTaskDelay(25);
			}
		}
	
	}
}

void setSpeed(uint8_t speed)
{
	itoa(speed, send, 10);
	UART_Transmit_String(send);
	uint16_t newSpeed = 20000 - (20000 / 100 * speed);
	itoa(newSpeed, send, 10);
	UART_Transmit_String(send);
	OCR5A = newSpeed;
	OCR5C = newSpeed;  
}

void initMotor()
{
	DDRC |= (1 << L_PLUS);
	DDRD |= (1 << L_MIN);
	DDRL |= (1 << L_EN) | (1 << R_MIN) | (1 << R_EN);
	DDRG |= (1 << R_PLUS);

	TCCR5A = (1 << WGM51) | (1 << COM5A0) | (1 << COM5A1) |
			 (1 << COM5C0) | (1 << COM5C1);
	TCCR5B = (1 << WGM53) | (1 << CS51);
	ICR5 = 20000;
	TCNT5 = 0;

	enable();
}

void soundTaak()
{
	uint8_t temp = 0;
	while(1)
	{
		if(!(PIN_GELUID & (1 << R_GELUID)))
		{
			temp = 'D';
			UART_Transmit_String("RECHTS\t");
			xQueueSend(motorCmd, (void*)&temp, 0);
		}
		else if(!(PIN_GELUID & (1 << L_GELUID)))
		{
			temp = 'A';
			UART_Transmit_String("LINKS\n");
			xQueueSend(motorCmd, (void*)&temp, 0);
		}
	}
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
	xQueueSend(motorCmd, (void*) &UDR0, 0);
}

void initQ()
{
	motorCmd = xQueueCreate(10, sizeof(uint8_t));
	sonarResult = xQueueCreate(10, sizeof(uint16_t));
}

void sonarTaak()
{
	uint16_t temp = 0;
	
	//UART_Transmit_String("in sonartaak");
	while(1)
	{
		_delay_ms(DELAY_BETWEEN_TESTS_MS);
		if(meetSonar)
		{
		pulse();
			if( xQueueReceive(sonarResult, &temp, 0) && temp < 60)
			{
				uint8_t temp2 = 'x';
				xQueueSend(motorCmd, (void*) &temp2,0);
			}
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

void servoTaak()
{
	uint16_t temp = 0;
	while(1)
	
	{
		if (servoMeet)
		{
			for(int i = 0; i < 180; i+=20)
			{
				pulse();
				while(!xQueueReceive(sonarResult, &temp, 0));
				char buf[20] = "Afstand: ";
				itoa(temp, send, 10);
				strcat(buf, send);
				UART_Transmit_String(buf);
				turnServo(i);
				vTaskDelay(100);
			}
			turnServo(110);
			
			servoMeet = false;
		}
		else
		{
			servoMeet = false;
		}
	}
}

void wait(unsigned int a)
{
	while(a--)
		_delay_ms(50);
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
			//itoa(result, send, 10);
			//UART_Transmit_String(send);
			xQueueSend(sonarResult, (void*) &result, 0);
			running = 0;
		}
	}
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

void turnServo(uint16_t degrees)
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
	turnServo(110);
}

void vooruit()
{
	PORTG |= (1 << R_PLUS);
	PORTC |= (1 << L_PLUS);
}

void achteruit()
{
	PORTL |= (1 << R_MIN);
	PORTD |= (1 << L_MIN);
}

void rechts()
{
	PORTL |= (1 << R_MIN);
	PORTC |= (1 << L_PLUS);
}

void links()
{
	PORTG |= (1 << R_PLUS);
	PORTD |= (1 << L_MIN);
}

void enable()
{
	PORTL |= (1 << R_EN);
	PORTL |= (1 << L_EN);
}

void M_stop()
{
	PORTG &= ~(1 << R_PLUS);
	PORTL &= ~(1 << R_MIN);
	PORTC &= ~(1 << L_PLUS);
	PORTD &= ~(1 << L_MIN);
}