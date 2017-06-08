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
#include <avr/wdt.h>

#define BAUD 9600
#define MYUBBR (F_CPU/16/BAUD-1)

void wait(unsigned);

#define nRxBuffer 10

//pwm frequentie = clock/(2*prescalar*TOP)
//16 bit timer phase correct
//pre scalar 8
// top 20000
// 20000 = 20 ms
// 900   = 0.9ms = 0 graden
// 1500  = 1.5ms = 90 graden
// 2100  = 2.1ms = 180 graden

void turnServo(uint16_t degrees)
{
	OCR1A = 20000 - ((degrees * 1200 / 180) + 900);
}

void usart_transmit(uint8_t data[], uint8_t length)
{
	for (uint8_t i = 0; i < length; i++)
	{
		//wait for empty transmit buffer
		while (!(UCSR0A & (1 << UDRE0)));
		UDR0 = data[i];
	}
}
void parseBuffer(uint8_t buffer[], uint8_t length)
{
	uint16_t angle = 0;
	for (uint8_t i = length; i > 0; i--)
	{
		uint8_t temp = buffer[length - i] - 48;
		for (uint8_t j = 0; j < i - 1; j++)
		{
			temp *= 10;
		}
		angle += temp;
	}
	
	turnServo(angle);
	uint8_t message[15] = "Turned servo: ";
	usart_transmit(message, 15);
	usart_transmit(buffer, length);
	uint8_t message2[11] = " degrees.\n";
	usart_transmit(message2, 11);
}

void addToRxBuffer(uint8_t data)
{
	static int rxBufferIndex = 0;
	static uint8_t rbuffer[nRxBuffer];
	if (data == '\n')
	{
		parseBuffer(rbuffer, rxBufferIndex);
		rxBufferIndex = 0;
	}
	else if (rxBufferIndex < nRxBuffer)
	{
		rbuffer[rxBufferIndex++] = data;
	}
	
	if (rxBufferIndex == nRxBuffer)
	{
		parseBuffer(rbuffer, rxBufferIndex);
		rxBufferIndex = 0;
	}
}

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
	PORTA ^= (1 << PA7);  // toggles the led
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

void initWatchDog()
{
	//disable interrupt
	cli();
	wdt_reset();
	//start timed sequence
	WDTCSR |= (1 << WDCE) | (1 << WDE);
	//set prescaler and mode interrupt and system reset
	WDTCSR = (1 << WDP0) | (1 << WDP3) | (1 << WDE) | (1 << WDIE);
	sei();
}


int main() {
	sei();
	initUSART(MYUBBR);
	uint8_t msg[9] = "reboot!\n";
	usart_transmit(msg, 9);
	initWatchDog();
	timer1_init();
	//set pin7 to output
	DDRB = (1 << PB7);	
	DDRA = (1 << PA7);

	while (1)
	{

	}
}

void wait(unsigned a) {
	while(a--)
	_delay_ms(100);
}

ISR(USART0_RX_vect)
{
	wdt_reset();
	addToRxBuffer(UDR0);
}

ISR(WDT_vect)
{
	turnServo(90);
	uint8_t message[21] = "Servo to save state\n";
	usart_transmit(message, 20);
}

