/*
 * Christian_Ovchinikov_Lab2_SourceCode.c
 *
 * Created: 1/17/2022 6:09:55 PM
 * Author : Christian Ovchinikov
*/ 

/*
This code will cause a TekBot connected to the AVR board to
move forward and when it touches an obstacle, it will reverse
and turn away from the obstacle and resume forward motion.

PORT MAP
Port B, Pin 4 -> Output -> Right Motor Enable
Port B, Pin 5 -> Output -> Right Motor Direction
Port B, Pin 7 -> Output -> Left Motor Enable
Port B, Pin 6 -> Output -> Left Motor Direction
Port D, Pin 1 -> Input -> Left Whisker
Port D, Pin 0 -> Input -> Right Whisker
*/

#define F_CPU 16000000
#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>

int main(void)
{
	// Outputs
	DDRB = 0b11110000;			// Sets 7-4th bits as outputs
	PORTB = 0b11110000;			// Initially turns on 7-4th LED
	
	// Inputs
	DDRD = 0b11111100;			// Sets 1-0th bits as inputs
	PORTD = 0b11111111;			// enables pull up resistor

	while (1)				// loop forever
	{
		
		PORTB = 0b01100000;		// Tekbot moves forward indefinitely

		// HitLeft Routine
		if(PIND == 0b11111101)		// Left Bumper condition
		{
			PORTB = 0b00000000;	// Tekbot moves backwards
			_delay_ms(1000);	// 1 second delay
			PORTB = 0b01000000;	// Tekbot turns right
			_delay_ms(1000);
			PORTB = 0b01100000;	// Tekbot continues moving forward
		}
		
		// HitRight and HitBoth Routine
		else if(PIND == 0b11111110 || PIND == 0b11111100) // Both and Right Bumper condition
		{
			PORTB = 0b00000000;	// Tekbot moves backwards
			_delay_ms(1000);
			PORTB = 0b00100000;	// Tekbot turns left
			_delay_ms(1000);
			PORTB = 0b01100000;	// Tekbot continues moving forward
		}
	}

}
