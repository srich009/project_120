/*
control a servo motor on portD with 2 buttons on portA
up button PA0
down button PA1
servo PD5
*/
//-------------------------------------

#include <avr/io.h>
#include <util/delay.h>

#include "timer.h"

// state machine
enum Servo_States{S_START,S_UP,S_DOWN} servo_state;
void servo_tick();
void servo_up();
void servo_down();
//-------------------------------------

int main()
{
	//Configure TIMER1
	TCCR1A|=(1<<COM1A1)|(1<<COM1B1)|(1<<WGM11);        //NON Inverted PWM
	TCCR1B|=(1<<WGM13)|(1<<WGM12)|(1<<CS11)|(1<<CS10); //PRESCALER=64 MODE 14(FAST PWM)

	ICR1=4999;  //fPWM=50Hz (Period = 20ms Standard).

	DDRD|=(1<<PD4)|(1<<PD5);   //PWM Pins as Out

	DDRA = 0x00; PORTA = 0xFF; // portA input

	// setup timer
	TimerSet(100);
	TimerOn();

	// set up servo initial state
	servo_down();
	servo_state = S_START;

	while(1)
	{
		servo_tick();
		
		while(!TimerFlag){}
		TimerFlag = 0;	
	}
}
//-------------------------------------

void servo_tick()
{
	unsigned char tmpA = ~PINA; // read input
	
	switch(servo_state)
	{
		case S_START:
			servo_state = S_DOWN;
		break;
		
		case S_DOWN:
			(tmpA & 0x01) ? (servo_state = S_UP) : (servo_state = S_DOWN) ;
		break;
		
		case S_UP:
			(tmpA & 0x02) ? (servo_state = S_DOWN) : (servo_state = S_UP) ;
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
		
		default:
		break;
	}
}
//-------------------------------------

void servo_up()
{
	OCR1A=130;   // 90 degree
}
//-------------------------------------

void servo_down()
{
	OCR1A=250;   //0 degree
}
//-------------------------------------

/*

servo positions from on breadboard

OCR1A=250;   // 0 degree ??
OCR1A=130;   // 90 degree ??

*/