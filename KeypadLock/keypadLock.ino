#include <LiquidCrystal_AIP31068_I2C.h>

LiquidCrystal_AIP31068_I2C lcd(0x3E, 16, 2);

#include <MyArduinoLibrary.h>

const byte numRows = 4; //number of rows on the keypad
const byte numCols = 3; //number of columns on the keypad

//keymap defines the key pressed according to the row and columns just as appears on the keypad
char keymap[numRows][numCols] =
{
  {'1', '2', '3'},//, 'A'},
  {'4', '5', '6'},//, 'B'},
  {'7', '8', '9'},//, 'C'},
  {'*', '0', '#'},//, 'D'}
};

char NOKEY = 0x00;
//Code that shows the the keypad connections to the arduino terminals
byte rowPins[numRows] = { 13,12,11,10 }; //Rows 0 to 3
byte colPins[numCols] = { 9,8,7 }; //Columns 0 to 2

char holdKey;

char key;
char kp[2];
char code[5];
char currentCode[5];

int keyCount;

FrameTime frameTime;

Timer entryTimer(&frameTime, 4 * 1000);
Timer flasherTimer(&frameTime, 250);
Timer codeInvalidTimer(&frameTime, 2 * 1000);

Device masterRelay(&frameTime, 4, 0);

LED greenLed(&frameTime, 5, 250);
LED redLed(&frameTime, 6, 0);

boolean codeInvalid;
boolean codeAccepted;
boolean flasherHIGH;

short state = 0;

void setup()
{
	lcd.init();

	for (int i = 0; i < numRows; i++)
	{
		pinMode(rowPins[i], OUTPUT);
	}

	for (int i = 0; i < numCols; i++)
	{
		pinMode(colPins[i], INPUT_PULLUP);
	}

	flasherHIGH = false;

	memset(code, 0x00, 5);
	memset(kp, 0x00, 2);

	// read EPROM current code
	strcpy(currentCode, "1234");

}

void loop()
{
	frameTime.Update();

	while (codeAccepted)
	{
		if (flasherTimer.Update())
		{
			if (flasherHIGH)
			{
				flasherHIGH = false;
				redLed.Off();
				greenLed.On();
			}
			else
			{
				flasherHIGH = true;
				redLed.On();
				greenLed.Off();
			}
		}

		return;
	}

	key = GetKey();

	if (key != NOKEY)
	{
		if (key == '*')
		{
			state = 1;
			memset(code, 0x00, 5);
		}
		else if (key == '#')
		{
			state = 3;
		}
		else if (strlen(code) < 4)
		{
			kp[0] = key;
			strcat(code, kp);
		}
	}

	lcd.setCursor(0, 0);
	lcd.print(code);

	if (codeInvalid)
	{
		memset(code, 0x00, 5);
		lcd.clear();

		redLed.On();
		if (codeInvalidTimer.Update())
		{
			codeInvalid = false;
			redLed.Off();
		}
	}

	switch (state)
	{
		// awaiting code
	case 0:
	{
		entryTimer.Reset();

		greenLed.Blink();

		break;
	}

	// updating code
	case 1:
	{
		greenLed.Off();

		redLed.Blink();

		if (strlen(code) == 4)
		{
			if (strcmp(code, currentCode) == 0)
			{
				state = 2;

				memset(code, 0x00, 5);
				lcd.clear();

				greenLed.Off();
				redLed.Off();

				Twinkle(4, 50);
			}
			else
			{
				codeInvalid = true;
				state = 3;
			}
		}

		break;
	}

	// awaiting new code
	case 2:
	{
		greenLed.Off();

		redLed.Blink();

		if (strlen(code) == 4)
		{
			// write EPROM current code
			strcpy(currentCode, code);

			memset(code, 0x00, 5);
			lcd.clear();

			greenLed.Off();
			redLed.Off();

			state = 0;

			Twinkle(4, 250);
		}

		break;
	}

	// awaiting code
	case 3:
	{
		flasherTimer.Reset();

		greenLed.Off();

		if (strcmp(code, currentCode) == 0)
		{
			codeAccepted = true;
			state = 4;

			lcd.setCursor(0, 0);
			lcd.print("Enabled");

			masterRelay.Start();
		}
		else
		{
			codeInvalid = true;
			state = 0;
		}

		break;
	}

	// this should never happen
	default:
	{
		break;
	}
	}

}

char GetKey()
{
	boolean noKey = true;
	holdKey = NOKEY;

	for (int r = 0; r < numRows; r++)
	{
		digitalWrite(rowPins[r], LOW);

		for (int c = 0; c < numCols; c++)
		{
			if (digitalRead(colPins[c]) == LOW)
			{
				holdKey = keymap[r][c];

				keyCount++;
				noKey = false;

				digitalWrite(colPins[c], HIGH);
				digitalWrite(rowPins[r], HIGH);

				break;
			}
		}

		digitalWrite(rowPins[r], HIGH);
	}

	if (noKey)
	{
		noKey = false;
		keyCount = 0;
	}

	if (keyCount > 1)
	{
		return NOKEY;
	}

	return holdKey;

}

void Twinkle(int count, int time)
{
	for (int i = 0; i < count; i++)
	{
		greenLed.On();
		delay(time);
		greenLed.Off();
		delay(time);
		redLed.On();
		delay(time);
		redLed.Off();
		delay(time);
	}

}
