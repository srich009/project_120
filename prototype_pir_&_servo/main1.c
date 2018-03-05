/*
PIR
the pir detects motion, input on PA0
output to led on PC0 when motion detected,
output to buzzer pwm on port B when motion detected: PB6
*/

/*
SERVO
control a servo motor on portD with 2 inputs on portA
up: global count variable, if count >= 2 then servo_up()
down: PA1
servo: PD5

servo positions from on breadboard
OCR1A=250;   // 0 degree ??
OCR1A=130;   // 90 degree ??
*/

//-----------------------------------------------------------------------------

// libraries
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h> // needed for the timer interrupt

#include "timer.h"
#include "pwm.h"

// servo state machine
enum Servo_States{S_START,S_UP,S_DOWN,S_RESET} servo_state;
void servo_up();
void servo_down();
void servo_tick(unsigned char c);

// PIR state machine
enum PIR_State{P_START,P_WAIT,P_ALERT} p_state;
void pir_tick(unsigned char tmpA);

// globals
unsigned char _count = 0; // count motion detects

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------

int main()
{
	// setup ports
	DDRA = 0x00; PORTA = 0xFF; // Configure port A's 8 pins as inputs
	DDRB = 0xFF; PORTB = 0x00; // Configure port B's 8 pins as outputs
	DDRC = 0xFF; PORTC = 0x00; // Configure port C's 8 pins as outputs
	//DDRD = 0xFF; PORTD = 0x00; // Configure port D's 8 pins as outputs
	
	// PORT D ...PWM Stuff
	//Configure TIMER1
	TCCR1A|=(1<<COM1A1)|(1<<COM1B1)|(1<<WGM11);        //NON Inverted PWM
	TCCR1B|=(1<<WGM13)|(1<<WGM12)|(1<<CS11)|(1<<CS10); //PRESCALER=64 MODE 14(FAST PWM)
	ICR1=4999;					//fPWM=50Hz (Period = 20ms Standard).
	DDRD|=(1<<PD4)|(1<<PD5);    //PWM Pins as Out

	// setup timer
	TimerSet(100);
	TimerOn();

	// pwm on for speaker on PORTB
	PWM_on();
	
	// set up pir sm initial states
	p_state = P_START;

	// set up servo sm initial state
	servo_down();
	servo_state = S_START;

	unsigned char tmpA = 0x00; // temp var to read input

	while(1)
	{
		tmpA = ~PINA; // read input pin

		pir_tick(tmpA);
		servo_tick(_count);
		
		while(!TimerFlag){}
		TimerFlag = 0;	
	}
}
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------

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
	unsigned char inputA = ~PINA; // read input
	
	switch(servo_state)
	{
		case S_START:
			servo_state = S_DOWN;
		break;
		
		case S_DOWN:
			(cnt >= 2) ? (servo_state = S_UP) : (servo_state = S_DOWN) ; // if 2 motion detects then servo up
		break;
		
		case S_UP:
			(inputA & 0x02) ? (servo_state = S_RESET) : (servo_state = S_UP) ; // if button on PA1, then reset count && back servo down
		break;
		
		case S_RESET:
			servo_state = S_DOWN;
		break;
		
		default:
			servo_state = S_START;
		break;
	}
	
	switch(servo_state)
	{
		case S_DOWN:
			servo_down();
		break;
		
		case S_UP:
			servo_up();
		break;
		
		case S_RESET:
			_count = 0; // reset global count
		break;
		
		default:
		break;
	}
}
//-----------------------------------------------------------------------------

void pir_tick(unsigned char tmpA)
{
	unsigned char tmpC = 0x00; // write to portc output
	
	switch(p_state) // transitions
	{
		case P_START:
		p_state = P_WAIT;
		break;
		
		case P_WAIT:
		( (tmpA & 0x01) ) ? (p_state = P_ALERT) : (p_state = P_WAIT); // check if motion detected or not
		break;
		
		case P_ALERT:
		( !(tmpA & 0x01) ) ? (p_state = P_WAIT) : (p_state = P_ALERT); 
		break;
		
		default:
		p_state = P_START;
		break;
	}
	
	switch(p_state) // actions
	{
		case P_WAIT:
		tmpC = 0x01; // light off
		set_PWM(0);    // speaker off
		break;
		
		case P_ALERT:
		tmpC = 0x00;  // light on
		set_PWM(440.0); // speaker on
		_count++;	// increment times seen
		break;
		
		default:
		break;
	}
	
	PORTC = tmpC;
}
//-----------------------------------------------------------------------------

