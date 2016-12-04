#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/delay.h>

void delay_ms(uint32_t time) {
  uint32_t i;
  for (i = 0; i < time; i++) {
    _delay_ms(1);
  }
}

#define NOOP asm volatile("nop" ::)


static void setup_clock( void )
{
	/*Examine Page 33*/

	CLKPR = 0x80;	/*Setup CLKPCE to be receptive*/
	CLKPR = 0x00;	/*No scalar*/
	OSCCAL = 0x80; //B8 is bottom E8 is top. 
}

int main( )
{
	cli();
	setup_clock();
	DDRB = _BV(4);
	uint8_t marker;

	while(1)
	{
		marker = 0x05;
		do{
			PORTB = _BV(4);
			marker--;
			PORTB = 0;
		} while( marker );

		_delay_us(1000000/60);
	}

	return 0;
} 
