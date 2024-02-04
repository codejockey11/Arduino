// https://www.circuitbasics.com/arduino-thermistor-temperature-sensor-tutorial/

#include <MyArduinoLibrary.h>

AnalogSensor temperature(A0, 5.0, 10000.0, 0.0);

float logR2, R2, T, Tf;
float c1 = 1.009249522e-03, c2 = 2.378405444e-04, c3 = 2.019202697e-07;

float prevR2;

void setup()
{
	Serial.begin(115200);
}

void loop()
{
	temperature.ReadState();

	// https://en.wikipedia.org/wiki/Steinhart%E2%80%93Hart_equation
	logR2 = log(temperature.R2);
	T = (1.0 / (c1 + c2 * logR2 + c3 * logR2 * logR2 * logR2));
	T = T - 273.15;
	Tf = (T * 9.0)/ 5.0 + 32.0;

	if (temperature.R2 != prevR2)
	{
		prevR2 = temperature.R2;

		Serial.print(temperature.voltage);

		Serial.print("\tC:");

		Serial.print(T);

		Serial.print("\tF:");

		Serial.print(Tf);

		Serial.println("");
	}
}
