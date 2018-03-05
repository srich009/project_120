#include <avr/io.h>
#include <util/delay.h>

#include "timer.h"

// global variables
int servo_state = 0; // 0 = down, 1 = up

// global functions
void servo_wait();
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

	TimerSet(100);
	TimerOn();

	unsigned char tmpA = 0x00;
	servo_down();
	servo_state = 0;

	while(1)
	{
		tmpA = ~PINA; // read input
		
		if(tmpA & 0x01)
		{
			servo_up();
			servo_state = 1;
		}
		else if(tmpA & 0x02)
		{
			servo_down();
			servo_state = 0;
		}
		else
		{
			// nothing
		}
		
		while(!TimerFlag){}
		TimerFlag = 0;	
	}
}
//-------------------------------------

//Simple Wait Function
void servo_wait()
{
	uint8_t i;
	for(i=0;i<50;i++)
	{
		_delay_loop_2(0);
		_delay_loop_2(0);
		_delay_loop_2(0);
	}
}
//-------------------------------------

void servo_up()
{
	OCR1A=130;   // 90 degree
	//servo_wait();
}
//-------------------------------------

void servo_down()
{
	OCR1A=250;   //0 degree
	//servo_wait();
}
//-------------------------------------

/*

servo positions from on breadboard

OCR1A=250;   // 0 degree ??
OCR1A=130;   // 90 degree ??

*/