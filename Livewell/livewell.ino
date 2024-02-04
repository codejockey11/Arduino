#include <Stepper.h>

Stepper motor(200, 10,11,12,13);

#include <MyArduinoLibrary.h>

int speed = 100;
int activate = 2;
int direction = 3;
int enable = 8;

void setup()
{
	pinMode(activate, INPUT_PULLUP);
	pinMode(direction, INPUT_PULLUP);

	pinMode(enable, OUTPUT);

}

void loop()
{
	int d = digitalRead(direction);

	int a = digitalRead(activate);

	digitalWrite(enable, LOW);

	if (a == 0)
	{
		if (d == 0)
		{
			digitalWrite(enable, HIGH);

			motor.step(10);
			motor.setSpeed(speed);
		}

		if (d == 1)
		{
			digitalWrite(enable, HIGH);

			motor.step(-10);
			motor.setSpeed(speed);
		}
	}

}
