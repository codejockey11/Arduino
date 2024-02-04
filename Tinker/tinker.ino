unsigned long currentTime;
unsigned long lastTime;
unsigned long frameTime;

bool isHigh;

void setup() {
	// initialize digital pin LED_BUILTIN as an output.
	pinMode(LED_BUILTIN, OUTPUT);

	currentTime = millis();
	lastTime = currentTime;
	
	isHigh = false;
}

// the loop function runs over and over again forever
void loop()
{
	lastTime = currentTime;
	currentTime = millis();
	frameTime += (currentTime - lastTime);

	if (frameTime >= 500)
	{
		if (isHigh)
		{
			isHigh = false;			
			digitalWrite(LED_BUILTIN, LOW);		
		}
		else
		{
			isHigh = true;	
			digitalWrite(LED_BUILTIN, HIGH);
		}
		
		frameTime = 0;
	}
}
