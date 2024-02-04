#include <MyArduinoLibrary.h>

AnalogSensor pressure(A0, 5.0, 1000.0, 0.0);

double prevVoltage;
double psi;

void setup()
{
	Serial.begin(115200);
}

void loop()
{
	pressure.ReadState();

	if (prevVoltage != pressure.voltage)
	{
		Serial.print(pressure.voltage, 5);

		Serial.print("\t");

		psi = map(pressure.voltage, 0.5, 4.5, 0.0, 150.0);

		Serial.print(psi);

		Serial.println("");

		prevVoltage = pressure.voltage;
	}
}
