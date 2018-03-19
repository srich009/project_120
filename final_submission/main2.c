/*

PIR
the pir detects motion, input on PC0
output to global variable _mic
output to led on PB0
buzzer pwm on port B when motion detected: PB6

SERVO
control a servo motor on portD with 2 inputs
up: global _count variable, if count >= 2 then servo_up()
down: PC1
servo: PD5
reset servo down has a 1 second wait time after
servo positions from on breadboard
OCR1A=250;   // 0 degree 
OCR1A=130;   // 90 degree 

MICROPHONE
microphone detects sound on PC2
if it is triggered it will raise the servo

ACCELEROMETER
3 axis input in portA upper half:
x = PA6
y = PA5
z = PA4

write to led on port c when detected (optional)
x - PB1
y - PB2
z - PB3

*/

//-----------------------------------------------------------------------------

// libraries
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h> // needed for the timer interrupt

#include "bit.h"
#include "timer.h"
#include "pwm.h"
#include "adc.h"

// shared globals
unsigned char _count = 0;   // count motion detects by PIR
unsigned char _mic = 0;		// count the mic
unsigned char _starts = 0;  // is in start or has been reset

// PIR state machine
enum PIR_State{P_START,P_WAIT,P_ALERT} pir_state;
int pir_tick(int pir_state, unsigned char tmpC);

// servo state machine
enum Servo_States{S_START,S_UP,S_DOWN,S_RESET} servo_state;
int servo_tick(int servo_state, unsigned char c1, unsigned char c2);

// microphone state machine
enum M_State{M_START,M_WAIT,M_ALERT} m_state;
int micro_tick(int m_state, unsigned char tmpC);

// values and data for A2D accelerometer
// threshold: value between 0 - 1023
#define XTHRES 338 // calibrated for x axis
#define YTHRES 340 // calibrated for y axis
#define ZTHRES 410 // calibrated for z axis

// accelerometer state machine
enum Accel_State{A_START} acc_state;
int accel_tick(int acc_state);

//=====================================================================================================================
//=====================================================================================================================

int main()
{
	// setup ports
	DDRB = 0xFF; PORTB = 0x00; // Configure port B's 8 pins as outputs
	DDRC = 0xF0; PORTC = 0x0F; // Configure port C's 8 pins as 4 upper outputs, 4 lower inputs
	//DDRA = 0x00; PORTA = 0xFF; // Configure port A's 8 pins as inputs
	//DDRD = 0xFF; PORTD = 0x00; // Configure port D's 8 pins as outputs
	
	// setup timer
	TimerSet(100);
	TimerOn();

	// pwm on for speaker on PORTB - PB6
	PWM_on();
	
	// PWM PORT PD5
	PWM_ON_PORTD();
	
	// initialize adc PORTA - PA4,PA5,PA6
	adc_init();
	
	// set up sm initial states
	int pir_state   = P_START;
	int servo_state = S_START;
	int m_state = M_START;
	int acc_state = A_START;

	unsigned char tmpC = 0x00; // temp var to read input

	while(1)
	{
		tmpC = ~PINC; // read input pin

		acc_state = accel_tick(acc_state);

		m_state = micro_tick(m_state, tmpC);

		pir_state = pir_tick(pir_state, tmpC);
		
		servo_state = servo_tick(servo_state,_count,_mic);
		
		if(_starts) // showing reset
		{
			unsigned char dd = PORTB; 
			PORTB = SetBit(dd,4,1); //pb4
			_count = 0;
			_mic = 0;
		}
		
		while(!TimerFlag){}
		TimerFlag = 0;
	}
}

//=====================================================================================================================
//=====================================================================================================================

int pir_tick(int pir_state, unsigned char tmpC)
{
	unsigned char disp = 0x00;
	
	switch(pir_state) // transitions
	{
		case P_START:
		pir_state = P_WAIT;
		break;
		
		// check if motion detected or not, PIR has enclosure on it
		case P_WAIT:
		( !(tmpC & 0x01) ) ? (pir_state = P_ALERT) : (pir_state = P_WAIT); 
		break;
		
		case P_ALERT:
		( (tmpC & 0x01) ) ? (pir_state = P_WAIT) : (pir_state = P_ALERT); 
		break;
		
		default:
		pir_state = P_START;
		break;
	}
	
	switch(pir_state) // actions
	{
		case P_WAIT:
		set_PWM(0);    // speaker off
		disp = 0x01;   // turn off PA0
		break;
		
		case P_ALERT:
		set_PWM(440.0); // speaker on
		disp = 0xFE;    // turn on PA0
		_count++;	    // increment times seen
		break;
		
		default:
		break;
	}
	
	PORTB |= disp;
	return pir_state;
}

//=====================================================================================================================
//=====================================================================================================================

int servo_tick(int servo_state, unsigned char cntPIR, unsigned char cntMIC)
{
	unsigned char inputC = ~PINC; // read input
	static unsigned char waiter = 0; // count cycles
	
	switch(servo_state)	// actions
	{
		case S_START:
		if(waiter < 10) // wait 10 cycles
		{
			waiter++;
			break;
		}
		servo_state = S_DOWN;
		waiter = 0;
		_starts = 1;
		break;
		
		case S_DOWN:
		if(cntPIR >= 1 || cntMIC >= 1) 	// if detects motion or sound then servo up
		{
			servo_state = S_UP;
		}
		else
		{
			servo_state = S_DOWN;
		}
		break;
		
		case S_UP:
		(inputC & 0x02) ? (servo_state = S_RESET) : (servo_state = S_UP) ; // if button on PC1, then reset && back servo down
		break;
		
		case S_RESET:
		servo_state = S_START;
		break;
		
		default:
		servo_state = S_START;
		break;
	}
	
	switch(servo_state) // transitions
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
		
		case S_RESET: // reset globals
		_count  = 0; 
		_mic = 0;
		_starts = 0;
		break;
		
		default:
		break;
	}
	
	return servo_state;
}

//=====================================================================================================================
//=====================================================================================================================

int micro_tick(int m_state, unsigned char tmpC)
{
	switch(m_state) // transitions
	{
		case M_START:
		m_state = M_WAIT;
		break;
		
		case M_WAIT:
		( (tmpC & 0x04) ) ? (m_state = M_ALERT) : (m_state = M_WAIT); // check if motion detected or not
		break;
		
		case M_ALERT:
		( !(tmpC & 0x04) ) ? (m_state = M_WAIT) : (m_state = M_ALERT);
		break;
		
		default:
		m_state = M_START;
		break;
	}
	
	switch(m_state)		// actions
	{
		case M_WAIT:
		break;
		
		case M_ALERT:
		_mic++;	    // increment count
		break;
		
		default:
		break;
	}
	
	return m_state;
}

//=====================================================================================================================
//=====================================================================================================================

int accel_tick(int acc_state)
{
	unsigned char tmp = 0x00; // output
	uint16_t adc_resultX;     // pin PA6
	uint16_t adc_resultY;     // pin PA5
	uint16_t adc_resultZ;     // pin PA4
	
	switch(acc_state) // transition
	{
		case A_START:
		acc_state = A_START;
		break;
		
		default:
		acc_state = A_START;
		break;
	}
	
	switch(acc_state) // action
	{
		case A_START:
			// read inputs
			adc_resultX = adc_read(6);      // read X adc value at PA6
			adc_resultY = adc_read(5);      // read Y adc value at PA5
			adc_resultZ = adc_read(4);		// read Z adc value at PA4
	
			// conditions for led to glow
			(adc_resultX < XTHRES) ? ( tmp = SetBit(tmp, 1, 1) ) : ( tmp = SetBit(tmp, 1, 0) ) ; // x - PB1
			(adc_resultY < YTHRES) ? ( tmp = SetBit(tmp, 2, 1) ) : ( tmp = SetBit(tmp, 2, 0) ) ; // y - PB2
			(adc_resultZ < ZTHRES) ? ( tmp = SetBit(tmp, 3, 1) ) : ( tmp = SetBit(tmp, 3, 0) ) ; // z - PB3
		break;
		
		default:
		break;
	}
	
	PORTB = tmp;
	return acc_state;
}

//=====================================================================================================================
//=====================================================================================================================