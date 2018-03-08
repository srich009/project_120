/*

ACCELEROMETER
3 axis input in portA upper half:
x = PA6
y = PA5
z = PA4

write to led on port c when detected 
x - PC5
y - PC6
z - PC7

*/

/*
control a servo motor on portD with 2 buttons on portA
up button PC0
down button PC1
servo PD5
*/
//-------------------------------------

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h> // needed for the timer interrupt

#include "bit.h"
#include "timer.h"

// servo state machine
enum Servo_States{S_START,S_UP,S_DOWN,S_RESET} servo_state;
void servo_up();
void servo_down();
void servo_tick(unsigned char c);
//-------------------------------------

// values and data for A2D accelerometer
// threshold: 500, ... {value between 0 - 1023}
#define XTHRES 338 // calibrated
#define YTHRES 340 // calibrated
#define ZTHRES 410 // calibrated

// global functions
void adc_init();
uint16_t adc_read(uint8_t ch);

// state machine
void accel_tick();

// global
unsigned char _motion = 0x00;

//-----------------------------------------------------------------------------

int main()
{
	
	//DDRA = 0x00; PORTA = 0xFF; // portA input
	DDRB = 0xFF; PORTB = 0x00; // Configure port B's 8 pins as outputs led on 
	DDRC = 0xF0; PORTC = 0x0F; // Configure port C's 8 pins as 4 lower inputs, 4 upper outputs to connect led to PC7-pc5
	
	//Configure TIMER1
	TCCR1A|=(1<<COM1A1)|(1<<COM1B1)|(1<<WGM11);        //NON Inverted PWM
	TCCR1B|=(1<<WGM13)|(1<<WGM12)|(1<<CS11)|(1<<CS10); //PRESCALER=64 MODE 14(FAST PWM)
	ICR1=4999;  //fPWM=50Hz (Period = 20ms Standard).
	DDRD|=(1<<PD4)|(1<<PD5);   //PWM Pins as Out

	// initialize adc
	adc_init();
	
	// timer
	TimerSet(100);
	TimerOn();

	// set up sm initial states
	servo_state = S_START;

	while(1)
	{
		accel_tick();
		
		servo_tick(_motion);
		
		while(!TimerFlag){}
		TimerFlag = 0;
	}
}
//=====================================================================================================================
//=====================================================================================================================

void servo_up()
{
	OCR1A=130;   // 90 degree
}
//-----------------------------------------------------------------------------

void servo_down()
{
	OCR1A=250;   //0 degree
}
//-----------------------------------------------------------------------------

void servo_tick(unsigned char cnt)
{
	unsigned char inputC = ~PINC; // read input
	static unsigned char waiter = 0; // count cycles
	
	switch(servo_state)
	{
		case S_START:
		if(waiter < 9) // wait 10 cycles
		{
			waiter++;
			break;
		}
		servo_state = S_DOWN;
		waiter = 0;
		break;
		
		case S_DOWN:
		(cnt >= 1) ? (servo_state = S_UP) : (servo_state = S_DOWN) ; // if 1 motion detects then servo up
		break;
		
		case S_UP:
		(inputC & 0x02) ? (servo_state = S_RESET) : (servo_state = S_UP) ; // if button on PC1, then reset count && back servo down
		break;
		
		case S_RESET:
		servo_state = S_START;
		break;
		
		default:
		servo_state = S_START;
		break;
	}
	
	switch(servo_state)
	{
		case S_START:
		servo_down();
		break;
		
		case S_DOWN:
		servo_down();
		break;
		
		case S_UP:
		servo_up();
		break;
		
		case S_RESET:
		_motion = 0x00; // reset global count
		break;
		
		default:
		break;
	}
}
//-----------------------------------------------------------------------------

//=====================================================================================================================
//=====================================================================================================================


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
	unsigned char tmp = 0x00; 
	
	uint16_t adc_resultX; // pin PA6
	uint16_t adc_resultY; // pin PA5
	uint16_t adc_resultZ; // pin PA4
	
	adc_resultX = adc_read(6);      // read X adc value at PA6
	adc_resultY = adc_read(5);      // read Y adc value at PA5
	adc_resultZ = adc_read(4);		// read Z adc value at PA4
	
	// condition for led to glow
	(adc_resultX < XTHRES) ? ( tmp = SetBit(tmp, 5, 1) ) : ( tmp = SetBit(tmp, 5, 0) ) ; // x - PC5
	(adc_resultY < YTHRES) ? ( tmp = SetBit(tmp, 6, 1) ) : ( tmp = SetBit(tmp, 6, 0) ) ; // y - PC6
	(adc_resultZ < ZTHRES) ? ( tmp = SetBit(tmp, 7, 1) ) : ( tmp = SetBit(tmp, 7, 0) ) ; // z - PC7

	//PORTC = tmp;
	
	
	if(tmp)
	{
		_motion += 1;
	}
}
