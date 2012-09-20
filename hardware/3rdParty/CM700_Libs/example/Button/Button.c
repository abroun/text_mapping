//##########################################################
//##                      R O B O T I S                   ##
//## CM-700 (Atmega2561) Example code for Button.         ##
//##                                           2009.11.10 ##
//##########################################################

#include <avr/io.h>

#define LED_BAT 0x01
#define LED_TxD 0x02
#define LED_RxD 0x04
#define LED_AUX 0x08
#define LED_MANAGE 0x10
#define LED_PROGRAM 0x20
#define LED_PLAY 0x40

#define SW_START 0x01

int main(void)
{
	DDRC  = 0x7F;
	PORTC = 0x7E;
	
	DDRD  = 0x70;
	PORTD = 0x11;

	while (1)
	{
		if(~PIND & SW_START)
			PORTC = ~(LED_BAT|LED_TxD|LED_RxD|LED_AUX|LED_MANAGE|LED_PROGRAM|LED_PLAY);
		else PORTC = LED_BAT|LED_TxD|LED_RxD|LED_AUX|LED_MANAGE|LED_PROGRAM|LED_PLAY;
	}
	return 1;
}
