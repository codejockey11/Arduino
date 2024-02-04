// https://ohmslawcalculator.com/555-astable-calculator
// C1 8nf
// R1 1k
// R2 100k
// cycle 897.389
// lcd pump at 0.5 seconds yields 224 pulses per pump

#include <LiquidCrystal_AIP31068_I2C.h>

LiquidCrystal_AIP31068_I2C lcd(0x3E, 16, 2);

#include <MyArduinoLibrary.h>

FrameTime frameTime;

Timer lcdTimer(&frameTime, 1000);

bool needCount;

float voltage;
float tireCircumference = 90.16f;
float differential = 3.25f;
float teethCount = 40.0f;
float tireRotation;
float rate;
float mph;
float timeAdjust;

uint32_t count;

char buffer[17];
char str[5];

void setup()
{
	Serial.begin(115200);
	
	lcd.init();

	count = 0;

	frameTime.Init();

	tireRotation = (tireCircumference / 12) / differential;

	timeAdjust = 1000.0 / lcdTimer.reloadTime;

}

double prevv;

void loop()
{
	frameTime.Update();

	voltage = analogRead(A0) * (1.0 / 1023.0 * 5.0);
	if (prevv != voltage)
	{
	Serial.print(frameTime.currentTime);
	Serial.print("\t");
	Serial.println(voltage);
	prevv = voltage;
	}

	if (voltage > 0)
	{
		needCount = true;
	}

	if ((voltage < 1) && (needCount == true))
	{
		needCount = false;

		count++;
	}

	if (lcdTimer.Update())
	{
		// distance = rate * time
		//
		// tireRotation is the total rate in inches
		// count is counted teeth on reluctor wheel
		// divide by the number of teeth
		// time adjusted for reset interval of count
		// gives overall rate in feet per cycle
		rate = tireRotation * ((count / teethCount) * timeAdjust);

		// convert rate to mph
		// time is 3600 which is one hour in seconds
		// 5280 feet in a mile
		mph = (rate * 3600) / 5280;

		dtostrf(mph, 3, 2, str);
		
		sprintf(buffer, "%s %u", str, count);
		
		lcd.setCursor(0, 0);
		
		PrintBuffer(buffer);
			
		count = 0;
	}
}

void PrintBuffer(char* b)
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
