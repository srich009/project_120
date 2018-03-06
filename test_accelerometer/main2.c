#include <avr/io.h>
#include <util/delay.h>

#include "timer.h"

// values and data for A2D accelerometer
// threshold: 500, ... {value between 0 - 1023}
#define XTHRES 410
#define YTHRES 410
#define ZTHRES 410
#define CONDS (adc_resultX < XTHRES && adc_resultY < YTHRES && adc_resultZ < ZTHRES)

// global functions
void adc_init();
uint16_t adc_read(uint8_t ch);

// state machine
void accel_tick();
//-----------------------------------------------------------------------------

int main()
{
	DDRC = 0xff;           // to connect led to PC7-pc5
	
	// initialize adc
	adc_init();
	
	// timer
	TimerSet(100);
	TimerOn();

	while(1)
	{
		accel_tick();
		
		while(!TimerFlag){}
		TimerFlag = 0;
	}
}
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------

// initialize adc
void adc_init()
{
	// AREF = AVcc
	ADMUX = (1<<REFS0);
	
	// ADC Enable and prescaler of 128
	// 16000000/128 = 125000
	ADCSRA = (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);
}
//-----------------------------------------------------------------------------

// read adc value
uint16_t adc_read(uint8_t ch)
{
	// select the corresponding channel 0~7
	// ANDing with '7' will always keep the value
	// of 'ch' between 0 and 7
	ch &= 0b00000111;  // AND operation with 7
	ADMUX = (ADMUX & 0xF8)|ch;     // clears the bottom 3 bits before ORing
	
	// start single conversion
	// write '1' to ADSC
	ADCSRA |= (1<<ADSC);
	
	// wait for conversion to complete
	// ADSC becomes '0' again
	// till then, run loop continuously
	while(ADCSRA & (1<<ADSC));
	
	return (ADC);
}
//-----------------------------------------------------------------------------

void accel_tick()
{
	uint16_t adc_resultX; // pin PA4
	uint16_t adc_resultY; // pin PA5
	uint16_t adc_resultZ; // pin PA6
	
	adc_resultX = adc_read(4);      // read adc value at PA4
	adc_resultY = adc_read(5);      // read adc value at PA5
	adc_resultZ = adc_read(6);		// read adc value at PA6
	
	// condition for led to glow
	(CONDS) ? (PORTC = 0xff) : (PORTC = 0x00) ;	
}