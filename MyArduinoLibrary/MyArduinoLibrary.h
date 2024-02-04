#ifndef MYARDUINOLIBRARY
#define MYARDUINOLIBRARY

#include "Arduino.h"

/*
From pins.arduino.h

On the Arduino board, digital pins are also used
for the analog output (software PWM).  Analog input
pins are a separate set.

ATMEL ATMEGA8 & 168 / ARDUINO

+-\/-+
PC6             1|    |28  PC5 (AI 5)
(D 0) PD0       2|    |27  PC4 (AI 4)
(D 1) PD1       3|    |26  PC3 (AI 3)
(D 2) PD2       4|    |25  PC2 (AI 2)
PWM+ (D 3) PD3  5|    |24  PC1 (AI 1)
(D 4) PD4       6|    |23  PC0 (AI 0)
VCC             7|    |22  GND
GND             8|    |21  AREF
PB6             9|    |20  AVCC
PB7            10|    |19  PB5 (D 13)
PWM+ (D 5) PD5 11|    |18  PB4 (D 12)
PWM+ (D 6) PD6 12|    |17  PB3 (D 11) PWM
(D 7) PD7      13|    |16  PB2 (D 10) PWM
(D 8) PB0      14|    |15  PB1 (D 9)  PWM
+----+

(PWM+ indicates the additional PWM pins on the ATmega168.)
*/

class FrameTime
{
	public:

	uint32_t currentTime;
	uint32_t previousTime;
	uint32_t frameTime;

	uint32_t clockTotalSeconds;
	uint32_t clockFrameTime;

	uint8_t hours;
	uint8_t minutes;
	uint8_t seconds;

	char strTime[9];

	void Init();

	void Update();

	void Clock();

	void IncrementHours();
	void DecrementHours();

	void IncrementMinutes();
	void DecrementMinutes();

	char* FormattedClock();
};

class Timer
{
	public:

	FrameTime* frameTime;

	uint32_t reloadTime;
	uint32_t totalTime;

	Timer();
	Timer(FrameTime* ft, uint32_t r);

	int Update();

	void Reset();
};

class Button
{
	public:

	uint8_t number;
	uint8_t count;

	Timer holdTimer;

	FrameTime* frameTime;

	Button(FrameTime* ft, uint8_t n, uint32_t r);

	int ReadState();
};

class Device
{
	public:

	enum
	{
		idle = 0,
		start,
		running,
		ended,
		stop
	};

	Timer runTimer;

	FrameTime* frameTime;

	char startTime[9];

	uint8_t state;

	int number;

	uint32_t duration;

	Device(FrameTime* ft, int n, uint32_t d);

	void Start();
	void Idle();
	void Stop();

	int Update();
	
	char* GetStateName();
	
	private:
	
	char strState[6];
};

class AnalogSensor
{
	public:

	int pin;
	int rawIn;

	double voltageIn;
	double voltage;

	double R1;
	double R2;

	double senderSpec;

	AnalogSensor(int p, double Vin, double r1, double ss);

	void ReadState();

	private:

	double multiplier;
};

class LED
{
	public:

	Timer blinkTimer;

	uint8_t number;

	boolean isOn;

	LED(FrameTime* ft, uint8_t n, uint32_t t);

	void On();
	void Off();
	void Blink();
};

class Storage
{
	public:

	uint8_t* settings;

	Storage() {maxSize = 512;}

	Storage(uint8_t* s, uint16_t l, uint16_t sz) {this->settings = s; this->length = l, this->maxSize = sz;}

	void ClearSettings(void);

	void StoreSettings(void);

	void ReadSettings(void);

	private:

	uint8_t* tempAddr;

	uint16_t maxSize;

	uint16_t length;
};

class AnalogPin
{
	public:

	int pin;

	double value;
	double voltage;

	AnalogPin() {}

	AnalogPin(int n, double v) {this->pin = n; this->voltage = v; this->multiplier = (1.0 / 1023.0) * this->voltage;}

	double ReadState();

	int ReadStateNoMultiplier();

	private:

	double multiplier;
};

class DigitalPin
{
	public:

	int pin;
	int value;

	DigitalPin() {}

	DigitalPin(int n, int t) {this->pin = n; pinMode(pin, t);}

	void Init (int n, int t) {this->pin = n; pinMode(pin, t);}

	int ReadState() {this->value = digitalRead(pin); return this->value;}

	void SetLow() {this->value = LOW; digitalWrite(this->pin, this->value);}

	void SetHigh() {this->value = HIGH; digitalWrite(this->pin, this->value);}

	void SetOutput() {this->value = OUTPUT; digitalWrite(this->pin, this->value);}

	void SetPullup() {this->value = INPUT_PULLUP; digitalWrite(this->pin, this->value);}
};

class Resistance
{
	public:

	double voltageIn;
	double resistorIn;

	double resistance;

	Resistance() {}

	Resistance(double vi, double ri) {this->resistance = 0, this->voltageIn = vi; this->resistorIn = ri;}

	double GetOhms(double sv);
};

// TWI Interface Object
// Two-Wire Serial Interface
//
// Usage is:
//
// I2CNetwork i2cNetwork;
//
// ISR(TWI_vect)
// {
// 		i2cNetwork.CheckISR();
// }
//

class I2CNetwork
{
	public:

	enum
	{
		// Physical pin 27
		PINSDA = A4,
		// Physical pin 28
		PINSCL = A5,

		READY = 0,

		RX = 1,
		TX = 2,

		WRITE = 0,
		READ = 1,

		SDAFREQ = 100000L,

		BUFFERLENGTH = 32,

		// Microseconds
		TIMEOUT = 25000
	};

	uint8_t state;
	uint8_t slarw;

	boolean inRepStart;

	boolean sendStop;

	uint8_t err;

	uint8_t txBuffer[I2CNetwork::BUFFERLENGTH];
	uint8_t txBufferIndex;
	uint8_t txBufferLength;

	uint8_t rxBuffer[I2CNetwork::BUFFERLENGTH];
	uint8_t rxBufferIndex;

	DigitalPin SDA;
	DigitalPin SCL;

	I2CNetwork() {SDA.Init(PINSDA, OUTPUT); SCL.Init(PINSCL, OUTPUT);}

	void Init();

	uint8_t RxRead(uint8_t address, uint8_t* data, uint8_t length, uint8_t sendStop);

	uint8_t TxWrite(uint8_t address, uint8_t* data, uint8_t length, uint8_t wait, uint8_t ss);

	void TimeoutReset();

	void Disable();

	void Reply(uint8_t ack);

	void Stop();

	void ReleaseBus();

	void CheckISR();

	class TWICODE
	{
		public:

		// do not change these values
		enum
		{
			BUS_ERROR				= 0x00,

			START					= 0x08,
			REP_START				= 0x10,

			MT_SLA_ACK				= 0x18,
			MT_SLA_NACK				= 0x20,
			MT_DATA_ACK				= 0x28,
			MT_DATA_NACK			= 0x30,
			MT_ARB_LOST				= 0x38,

			MR_ARB_LOST				= 0x38,
			MR_SLA_ACK				= 0x40,
			MR_SLA_NACK				= 0x48,
			MR_DATA_ACK				= 0x50,
			MR_DATA_NACK			= 0x58,

			ST_SLA_ACK				= 0xA8,
			ST_ARB_LOST_SLA_ACK		= 0xB0,
			ST_DATA_ACK				= 0xB8,
			ST_DATA_NACK			= 0xC0,
			ST_LAST_DATA			= 0xC8,

			SR_SLA_ACK				= 0x60,
			SR_ARB_LOST_SLA_ACK		= 0x68,
			SR_GCALL_ACK			= 0x70,
			SR_ARB_LOST_GCALL_ACK	= 0x78,
			SR_DATA_ACK				= 0x80,
			SR_DATA_NACK			= 0x88,
			SR_GCALL_DATA_ACK		= 0x90,
			SR_GCALL_DATA_NACK		= 0x98,
			SR_STOP					= 0xA0,

			NO_INFO					= 0xF8,
			NO_ERROR				= 0xFF
		};
	};
};

class NumPad
{
	public:

	// Keymap array
	char keymap[4][4];

	DigitalPin rowPins[4];
	DigitalPin colPins[4];

	uint8_t numRows;
	uint8_t numCols;

	char key;

	char NOKEY;

	boolean noKey;

	uint16_t keyCount;

	NumPad();

	NumPad(uint8_t r, uint8_t c, char* km, int* rp, int* cp);

	char GetKey();
};

class MyString
{
	public:

	enum
	{
		MAXSIZE = 255
	};

	char value[MyString::MAXSIZE];

	uint8_t size;

	MyString() {memset(this, 0x00, sizeof(String));}

	MyString(int sz) {memset(this, 0x00, sizeof(String)); this->size = sz;}

	MyString(char* c);

	void DoubleFloatToStr(double d, uint8_t precision);

	double Double() {return atof(this->value);}

	void Clear() {memset(this->value, 0x00, this->size);}

	void Append(char* c)
	{
		if (this->size < (MAXSIZE - 2))
		{
			strcat(value, c);

			this->size++;
		}
	}
};
#endif
