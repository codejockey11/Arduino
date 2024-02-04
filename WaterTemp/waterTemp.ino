#include <MyArduinoLibrary.h>

AnalogSensor waterTemp(A0, 5.0, 47000.0, 0.0);

float temperature;

void setup()
{
	Serial.begin(115200);
}

void loop()
{
	waterTemp.ReadState();

	if (waterTemp.R2 >= 60000)
	{
		temperature = 20;
	}
	else if (waterTemp.R2 >= 38000)
	{
		temperature = 30;
	}
	else if (waterTemp.R2 >= 25500)
	{
		temperature = 40;
	}
	else if (waterTemp.R2 >= 17500)
	{
		temperature = 50;
	}
	else if (waterTemp.R2 >= 14600)
	{
		temperature = 55;
	}
	else if (waterTemp.R2 >= 12300)
	{
		temperature = 60;
	}
	else if (waterTemp.R2 >= 10400)
	{
		temperature = 65;
	}
	else if (waterTemp.R2 >= 8800)
	{
		temperature = 70;
	}
	else if (waterTemp.R2 >= 7500)
	{
		temperature = 75;
	}
	else if (waterTemp.R2 >= 6400)
	{
		temperature = 80;
	}
	else if (waterTemp.R2 >= 5500)
	{
		temperature = 85;
	}
	else if (waterTemp.R2 >= 4800)
	{
		temperature = 90;
	}
	else if (waterTemp.R2 >= 4150)
	{
		temperature = 95;
	}
	else if (waterTemp.R2 >= 3610)
	{
		temperature = 100;
	}
	else if (waterTemp.R2 >= 2760)
	{
		temperature = 110;
	}
	else if (waterTemp.R2 >= 2130)
	{
		temperature = 120;
	}
	else if (waterTemp.R2 >= 1650)
	{
		temperature = 130;
	}
	else if (waterTemp.R2 >= 1300)
	{
		temperature = 140;
	}
	else if (waterTemp.R2 >= 1020)
	{
		temperature = 150;
	}
	else if (waterTemp.R2 >= 780)
	{
		temperature = 160;
	}
	else if (waterTemp.R2 >= 610)
	{
		temperature = 170;
	}
	else if (waterTemp.R2 >= 460)
	{
		temperature = 180;
	}

	Serial.print(5.0 - waterTemp.voltage);
	Serial.print("\t");

	Serial.print(waterTemp.voltage);
	Serial.print("\t");

	Serial.print(waterTemp.R2);
	Serial.print("\tc:");

	Serial.print(temperature);     
	Serial.println("");
}
