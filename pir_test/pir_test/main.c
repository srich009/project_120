// basic setup -- PIR
// get pir to detect motion, signal input on PA0
// output to led on PB0 when motion detected,
//-----------------------

// libraries
#include <avr/io.h>
#include <avr/interrupt.h> // needed for the timer interrupt

// Timer stuff
//------------------------------------------------------------------------------------------------
volatile unsigned char TimerFlag = 0; // TimerISR() sets this to 1. C programmer should clear to 0.

// Internal variables for mapping AVR's ISR to our cleaner TimerISR model.
unsigned long _avr_timer_M = 1; // Start count from here, down to 0. Default 1 ms.
unsigned long _avr_timer_cntcurr = 0; // Current internal count of 1ms ticks

void TimerOn() {
	// AVR timer/counter controller register TCCR1
	TCCR1B = 0x0B;// bit3 = 0: CTC mode (clear timer on compare)
	// bit2bit1bit0=011: pre-scaler /64
	// 00001011: 0x0B
	// SO, 8 MHz clock or 8,000,000 /64 = 125,000 ticks/s
	// Thus, TCNT1 register will count at 125,000 ticks/s

	// AVR output compare register OCR1A.
	OCR1A = 125;	// Timer interrupt will be generated when TCNT1==OCR1A
	// We want a 1 ms tick. 0.001 s * 125,000 ticks/s = 125
	// So when TCNT1 register equals 125,
	// 1 ms has passed. Thus, we compare to 125.
	// AVR timer interrupt mask register
	TIMSK1 = 0x02; // bit1: OCIE1A -- enables compare match interrupt

	//Initialize avr counter
	TCNT1=0;

	_avr_timer_cntcurr = _avr_timer_M;
	// TimerISR will be called every _avr_timer_cntcurr milliseconds

	//Enable global interrupts
	SREG |= 0x80; // 0x80: 1000000
}

void TimerOff() {
	TCCR1B = 0x00; // bit3bit1bit0=000: timer off
}

void TimerISR() {
	TimerFlag = 1;
}

// In our approach, the C programmer does not touch this ISR, but rather TimerISR()
ISR(TIMER1_COMPA_vect) {
	// CPU automatically calls when TCNT1 == OCR1 (every 1 ms per TimerOn settings)
	_avr_timer_cntcurr--; // Count down to 0 rather than up to TOP
	if (_avr_timer_cntcurr == 0) { // results in a more efficient compare
		TimerISR(); // Call the ISR that the user uses
		_avr_timer_cntcurr = _avr_timer_M;
	}
}

// Set TimerISR() to tick every M ms
void TimerSet(unsigned long M) {
	_avr_timer_M = M;
	_avr_timer_cntcurr = _avr_timer_M;
}

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