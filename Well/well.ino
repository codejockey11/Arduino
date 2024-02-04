#include <LiquidCrystal_AIP31068_I2C.h>

LiquidCrystal_AIP31068_I2C lcd(0x3E, 16, 2);

#include "MyArduinoLibrary.h"

class DeviceChain
{
	public:

	enum
	{
		sprinkler = 0,
		wellPump,
		generator
	};

	uint8_t currentDevice;

	DeviceChain(){currentDevice = 0;};
};

FrameTime frameTime;

Timer daylightTimer(&frameTime, 5000);

Device sprinkler(&frameTime, 2, (uint32_t)5000);
Device wellPump(&frameTime, 3, 0);
Device generator(&frameTime, 4, 0);

DeviceChain deviceChain;

Button page(&frameTime, 9, 500);
Button up(&frameTime, 10, 125);
Button down(&frameTime, 11, 125);
Button home(&frameTime, 12, 0);

AnalogSensor lightSensor(A0, 5.0, 1000.0, 0.0);
AnalogSensor tankSensor(A1, 5.0, 1000.0, 0.0);
AnalogSensor voltMeter(A2, 5.0, 1000.0, 13.4);

short pageNbr = 0;

double threshold = 2.50;

double tankLevel;

char s1[8];
char s2[8];

char lcdLine[17];

void Page();

void WriteLcdLine(const char* b);

void setup()
{
	//Serial.begin(115200);

	lcd.init();

	// read settings from EPROM

	frameTime.Init();

	deviceChain.currentDevice = DeviceChain::sprinkler;
}

void loop()
{
	frameTime.Update();

	lightSensor.ReadState();

	tankSensor.ReadState();

	tankLevel = map(tankSensor.R2, 0, 248.78, 0, 10);

	voltMeter.ReadState();

	if (lightSensor.voltage >= threshold)
	{
		// timer in case of lightning at night time
		if (daylightTimer.Update())
		{
			// enough daylight to start days events
			// starting with the sprinkler
			// assuming enough battery power
			if (sprinkler.state == Device::idle)
			{
				// device state is now running
				sprinkler.Start();
			}
		}
	}
	else
	{
		// sun down events
		daylightTimer.Reset();
			
		// idle sprinkler
		if (sprinkler.state != Device::running)
		{
			sprinkler.Idle();
		}
	}

	// device state is set to ended when timer completes
	// this update only happens when device is running
	switch (deviceChain.currentDevice)
	{
		case DeviceChain::sprinkler:
		{
			if (sprinkler.Update())
			{
				deviceChain.currentDevice++;
			}

			break;
		}

		case DeviceChain::wellPump:
		{
			if (tankLevel < 10)
			{
				wellPump.Start();
			}
			else
			{
				wellPump.Stop();

				deviceChain.currentDevice++;
			}

			break;
		}

		case DeviceChain::generator:
		{
			if (voltMeter.voltage < 4.99)
			{
				generator.Start();
			}
			else
			{
				generator.Stop();

				deviceChain.currentDevice = 0;
			}

			break;
		}
	}

	Page();
}

void Page()
{
	if (home.ReadState())
	{
		lcd.clear();

		pageNbr = 0;
		// save settings to EPROM
	}

	if (page.ReadState())
	{
		lcd.clear();

		pageNbr++;

		if (pageNbr > 8)
		{
			pageNbr = 0;
		}
	}

	switch (pageNbr)
	{
		case 0:
		{
			lcd.setCursor(0, 0);

			sprintf(lcdLine, "%s", frameTime.FormattedClock());
			WriteLcdLine(lcdLine);

			if (sprinkler.state == Device::ended)
			{
				lcd.setCursor(0, 1);

				sprintf(lcdLine, "%s", sprinkler.startTime);
				WriteLcdLine(lcdLine);
			}

			break;
		}

		case 1:
		{
			if (up.ReadState())
			{
				frameTime.IncrementHours();
			}

			if (down.ReadState())
			{
				frameTime.DecrementHours();
			}

			lcd.setCursor(0, 0);

			sprintf(lcdLine, "Hours %u", frameTime.hours);
			WriteLcdLine(lcdLine);

			break;
		}

		case 2:
		{
			if (up.ReadState())
			{
				frameTime.IncrementMinutes();
			}

			if (down.ReadState())
			{
				frameTime.DecrementMinutes();
			}

			lcd.setCursor(0, 0);

			sprintf(lcdLine, "Minutes %u", frameTime.minutes);
			WriteLcdLine(lcdLine);

			break;
		}

		case 3:
		{
			if (up.ReadState())
			{
				sprinkler.duration += 60000;

				if (sprinkler.duration > 3600000)
				{
					sprinkler.duration = 0;
				}
			}

			if (down.ReadState())
			{
				sprinkler.duration -= 60000;

				if (sprinkler.duration > 3600000)
				{
					sprinkler.duration = 3600000;
				}
			}

			sprinkler.runTimer.reloadTime = sprinkler.duration;

			lcd.setCursor(0, 0);

			sprintf(lcdLine, "Duration %lu", sprinkler.duration / 60000);
			WriteLcdLine(lcdLine);

			break;
		}

		case 4:
		{
			lcd.setCursor(0, 0);

			WriteLcdLine("Sprinkler On");

			// %lu for uint32_t
			lcd.setCursor(0, 1);

			sprintf(lcdLine, "%lu %lu", sprinkler.runTimer.totalTime, sprinkler.runTimer.reloadTime);
			WriteLcdLine(lcdLine);

			break;
		}

		case 5:
		{
			if (up.ReadState())
			{
				threshold += 0.01;

				if (threshold > 3.57)
				{
					threshold = 1.0;
				}
			}

			if (down.ReadState())
			{
				threshold -= 0.01;

				if (threshold < 1.0)
				{
					threshold = 3.57;
				}
			}

			lcd.setCursor(0, 0);

			WriteLcdLine("Dawn");

			lcd.setCursor(0, 1);

			dtostrf(lightSensor.voltage, 3, 2, s1);
			dtostrf(threshold, 3, 2, s2);

			sprintf(lcdLine, "%s >= %s", s1, s2);
			WriteLcdLine(lcdLine);

			break;
		}

		case 6:
		{
			lcd.setCursor(0, 0);
			WriteLcdLine("Tank");

			lcd.setCursor(0, 1);

			sprintf(lcdLine, "%i%%", int(100.0 * (tankLevel / 10.0)));
			WriteLcdLine(lcdLine);

			break;
		}

		case 7:
		{
			lcd.setCursor(0, 0);
			WriteLcdLine("Battery");

			lcd.setCursor(0, 1);

			sprintf(lcdLine, "%i%%", int(100.0 * (voltMeter.voltage / 5) + 0.10));
			WriteLcdLine(lcdLine);

			break;
		}

		case 8:
		{
			lcd.setCursor(0, 0);
			WriteLcdLine("Current Device");

			lcd.setCursor(0, 1);
			
			switch(deviceChain.currentDevice)
			{
				case 0:
				{
					sprintf(lcdLine, "Sprinkler:%s", sprinkler.GetStateName());
					WriteLcdLine(lcdLine);
					
					break;
				}
				
				case 1:
				{
					sprintf(lcdLine, "Pump:%s", wellPump.GetStateName());
					WriteLcdLine(lcdLine);
									
					break;
				}
				
				case 2:
				{
					sprintf(lcdLine, "Generator:%s", generator.GetStateName());
					WriteLcdLine(lcdLine);
									
					break;
				}
			}

			break;
		}
	}
}

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
