//declare global arrays for two patterns
unsigned char p1[] = { 0b00100001,
						0b00010010,
						0b00001100,
						0b00001100,
						0b00010010,
						0b00100001 };

unsigned char p2[] = { 0b00111111,
						0b00011110,
						0b00001100,
						0b00001100,
						0b00011110,
						0b00111111 };

int main()
{
	//loop counter
	unsigned char i;

	//Port B as output
	DDRB = 0xFF;

	//keep all LEDs off
	PORTB = 0x00;

	//Port C as input
	DDRC = 0x00;

	//enable pull ups for
	PORTC |= 0b00000011;

	//only first two pins
	while (1)
	{
		//# if SW0 is pressed show pattern 1
		if ((PINC & 0b00000001) == 0)
		{
			for (i = 0; i < 6; i++)
			{
				//output data
				PORTB = p1[i];

				//wait for some time
				_delay_ms(250);
			}

			//turn off all LEDs
			PORTB = 0;
		}

		//# if SW1 is pressed show pattern 2
		if ((PINC & 0b00000010) == 0)
		{
			for (i = 0; i < 6; i++)
			{
				//output data
				PORTB = p2[i];

				//wait for some time
				_delay_ms(250);
			}

			//turn off all LEDs
			PORTB = 0;
		}
	};

	return 0;
}
