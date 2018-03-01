// basic setup -- PIR
// get pir to detect motion, signal input on PA0
// output to led on PB0 when motion detected,
//----------------------------------------------------------------

// libraries
#include <avr/io.h>
#include <avr/interrupt.h> // needed for the timer interrupt
#include "timer.h"

//----------------------------------------------------------------

// function declarations
void sm_tick(unsigned char tmpA);

// global variables
enum SM_State{START,WAIT,ALERT} state;

int main(void)
{
	// setup ports
	DDRA = 0x00; PORTA = 0xFF; // Configure port A's 8 pins as inputs
	DDRB = 0xFF; PORTB = 0x00; // Configure port B's 8 pins as outputs
	//DDRC = 0xFF; PORTC = 0x00; // Configure port C's 8 pins as outputs
	//DDRD = 0xFF; PORTC = 0x00; // Configure port C's 8 pins as outputs

	// setup timer
	TimerOn();
	TimerSet(100); // ms
	
	// set sm initial state
	state = START;
	
	unsigned char tmpA = 0x00; // temp var to read input
	
	while(1)
	{
		tmpA = ~PINA; // read input pin
		
		sm_tick(tmpA);
		
		while(!TimerFlag){}
		TimerFlag = 0;
	}
}

void sm_tick(unsigned char tmpA)
{
	unsigned char tmpB = 0x00; // write to portb output
	
	switch(state) // transitions
	{
		case START:
		state = WAIT;
		break;
		
		case WAIT:
		( (tmpA & 0x01) ) ? (state = ALERT) : (state = WAIT); // if motion detected or not
		break;
		
		case ALERT:
		( !(tmpA & 0x01) ) ? (state = WAIT) : (state = ALERT);
		break;
		
		default:
		state = START;
		break;
	}
	
	switch(state) // actions
	{
		case WAIT:
		tmpB = 0x00; // light off
		break;
		
		case ALERT:
		tmpB = 0x01; // light on
		break;
		
		default:
		break;
	}
	
    PORTB = tmpB;
}