/*
PIR
the pir detects motion, input on PC0
output to buzzer pwm on port B when motion detected: PB6
*/

/*
SERVO
control a servo motor on portD with 2 inputs
up: global count variable, if count >= 2 then servo_up()
down: PC1
servo: PD5

reset servo down has a 1 second wait time after

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

// globals
unsigned char _count = 0; // count motion detects by PIR

// PIR state machine
enum PIR_State{P_START,P_WAIT,P_ALERT} pir_state;
void pir_tick(unsigned char tmpC);

// servo state machine
enum Servo_States{S_START,S_UP,S_DOWN,S_RESET} servo_state;
void servo_up();
void servo_down();
void servo_tick(unsigned char c);

#define PORT_ON(port,pin)   port |= (1<<pin)
#define PORT_OFF(port,pin)  port &= ~(1<<pin)

//=====================================================================================================================
//=====================================================================================================================

int main()
{
	// setup ports
	DDRB = 0xFF; PORTB = 0x00; // Configure port B's 8 pins as outputs
	DDRC = 0xF0; PORTC = 0x0F; // Configure port C's 8 pins as 4 upper outputs, 4 lower inputs
	//DDRA = 0x00; PORTA = 0xFF; // Configure port A's 8 pins as inputs
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
	
	// set up sm initial states
	pir_state = P_START;
	servo_state = S_START;

	unsigned char tmpC = 0x00; // temp var to read input

	while(1)
	{
		tmpC = ~PINC; // read input pin

		pir_tick(tmpC);
		
		servo_tick(_count);
		
		while(!TimerFlag){}
		TimerFlag = 0;
	}
}

//=====================================================================================================================
//=====================================================================================================================

void pir_tick(unsigned char tmpC)
{
	switch(pir_state) // transitions
	{
		case P_START:
		pir_state = P_WAIT;
		break;
		
		case P_WAIT:
		( (tmpC & 0x01) ) ? (pir_state = P_ALERT) : (pir_state = P_WAIT); // check if motion detected or not
		break;
		
		case P_ALERT:
		( !(tmpC & 0x01) ) ? (pir_state = P_WAIT) : (pir_state = P_ALERT);
		break;
		
		default:
		pir_state = P_START;
		break;
	}
	
	switch(pir_state) // actions
	{
		case P_WAIT:
		set_PWM(0);    // speaker off
		break;
		
		case P_ALERT:
		set_PWM(440.0); // speaker on
		_count++;	// increment times seen
		break;
		
		default:
		break;
	}
}
//-----------------------------------------------------------------------------

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
		(cnt >= 2) ? (servo_state = S_UP) : (servo_state = S_DOWN) ; // if 2 motion detects then servo up
		break;
		
		case S_UP:
		(inputC & 0x02) ? (servo_state = S_RESET) : (servo_state = S_UP) ; // if button on PA1, then reset count && back servo down
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
		_count = 0; // reset global count
		break;
		
		default:
		break;
	}
}
//-----------------------------------------------------------------------------

