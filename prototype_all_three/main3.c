/*
PIR
the pir detects motion, input on PC0
output to buzzer pwm on port B when motion detected: PB6
output to led on PB0
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


/*
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

// globals
unsigned char _count = 0;  // count motion detects by PIR
unsigned char _motion = 0; // count the accelerometer
unsigned char _starts = 0;  // is in start or has been reset

// PIR state machine
enum PIR_State{P_START,P_WAIT,P_ALERT} pir_state;
void pir_tick(unsigned char tmpC);

// servo state machine
enum Servo_States{S_START,S_UP,S_DOWN,S_RESET} servo_state;
void servo_up();
void servo_down();
void servo_tick(unsigned char c1, unsigned char c2 );

// values and data for A2D accelerometer
// threshold: 500, ... {value between 0 - 1023}
#define XTHRES 338 // calibrated
#define YTHRES 340 // calibrated
#define ZTHRES 410 // calibrated
#define CONDS (adc_resultX < XTHRES && adc_resultY < YTHRES && adc_resultZ < ZTHRES)

// global functions
void adc_init();
uint16_t adc_read(uint8_t ch);

// state machine
void accel_tick();

// global
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

	// pwm on for speaker on PORTB - PB6
	PWM_on();
	
	// initialize adc PORTA - PA4,PA5,PA6
	adc_init();
	
	// set up sm initial states
	pir_state   = P_START;
	servo_state = S_START;

	unsigned char tmpC = 0x00; // temp var to read input

	while(1)
	{
		tmpC = ~PINC; // read input pin

		accel_tick();

		pir_tick(tmpC);
		
		servo_tick(_count,_motion);
		
		if(_starts) // show reset light on PB4
		{
			unsigned char dd = PORTB;
			PORTB = SetBit(dd,4,1);
			_count = 0;
			_motion = 0;
		}
		
		while(!TimerFlag){}
		TimerFlag = 0;
	}
}

//=====================================================================================================================
//=====================================================================================================================

void pir_tick(unsigned char tmpC)
{
	unsigned char disp = 0x00;
	
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

void servo_tick(unsigned char cntPIR, unsigned char cntACC)
{
	unsigned char inputC = ~PINC; // read input
	static unsigned char waiter = 0; // count cycles
	
	switch(servo_state)
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
		// if detects then servo up
		if(cntPIR >= 1 || cntACC >= 1) // ************ here error ??
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
		
		case S_RESET: // reset globals
		_count  = 0; 
		_motion = 0;
		_starts = 0;
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

	(adc_resultX < XTHRES) ? ( tmp = SetBit(tmp, 1, 1) ) : ( tmp = SetBit(tmp, 1, 0) ) ; // x - PB1
	(adc_resultY < YTHRES) ? ( tmp = SetBit(tmp, 2, 1) ) : ( tmp = SetBit(tmp, 2, 0) ) ; // y - PB2
	(adc_resultZ < ZTHRES) ? ( tmp = SetBit(tmp, 3, 1) ) : ( tmp = SetBit(tmp, 3, 0) ) ; // z - PB3
		
	PORTB = tmp;
	
	if(tmp)
	{
		_motion += 1;
	}
}