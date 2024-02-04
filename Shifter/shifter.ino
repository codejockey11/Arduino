#include <LiquidCrystal_AIP31068_I2C.h>

LiquidCrystal_AIP31068_I2C lcd(0x3E, 16, 2);

#include <MyArduinoLibrary.h>

void WriteLcdLine(const char* b)
{
  lcd.print(b);

  if (strlen(b) < 16)
  {
    for(int i = strlen(b);i < 16;i++)
    {
      lcd.print(" ");
    }
  }
}

FrameTime frameTime;

Timer neutralTimer(&frameTime, 1500);

char gear = 'N';

AnalogSensor shifterPosition(A3, 5.0, 0.0, 0.0);

void setup()
{
	Serial.begin(115200);

	lcd.init();

	frameTime.Init();
}

void loop()
{
	frameTime.Update();

	shifterPosition.ReadState();

	Serial.println(shifterPosition.voltage);

	switch (shifterPosition.rawIn)
	{
		case 146:
		{
			gear = '1';

			neutralTimer.Reset();

			break;
		}

		case 78:
		{
			gear = '2';

			neutralTimer.Reset();

			break;
		}

		case 53:
		{
			gear = '3';

			neutralTimer.Reset();

			break;
		}

		case 40:
		{
			gear = '4';

			neutralTimer.Reset();

			break;
		}

		case 32:
		{
			gear = '5';

			neutralTimer.Reset();

			break;
		}

		case 27:
		{
			gear = 'R';

			neutralTimer.Reset();

			break;
		}

		default:
		{
			if (neutralTimer.Update())
			{
				gear = 'N';

				neutralTimer.Reset();
			}

			break;
		}
	}

	lcd.setCursor(0, 0);
	lcd.print(gear);
}
