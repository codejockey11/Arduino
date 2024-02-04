// 1 / 1023 * 5
const float multiplier = 0.004887585532746823069403714565;

double rA0;
double rA1;

void setup()
{
	 Serial.begin(115200);

	 InitADC();

}

void loop()
{
	 rA0 = ReadADC(0) * multiplier;
	 rA1 = ReadADC(1) * multiplier;

	 Serial.print(rA0, 5);
	 Serial.print("\t");
	 Serial.println(rA1, 5);

}

void InitADC()
{
	// Select Vref=AVcc
	ADMUX |= (1 << REFS0);

	//set prescaller to 128 and enable ADC
	ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0) | (1 << ADEN);

}

uint16_t ReadADC(uint8_t ADCchannel)
{
	//select ADC channel with safety mask
	ADMUX = (ADMUX & 0xF0) | (ADCchannel & 0x0F);

	//single conversion mode
	ADCSRA |= (1 << ADSC);

	// wait until ADC conversion is complete
	while( ADCSRA & (1 << ADSC) );

	return ADC;

}
