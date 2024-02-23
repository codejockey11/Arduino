#include "MyArduinoLibrary.h"

void FrameTime::Init()
{
	this->currentTime = millis();
}

void FrameTime::Update()
{
	this->previousTime = this->currentTime;
	this->currentTime  = millis();
	this->frameTime    = this->currentTime - this->previousTime;

	this->Clock();
}

void FrameTime::Clock()
{
	this->clockFrameTime += this->frameTime;

	if (this->clockFrameTime >= 1000)
	{
		this->seconds++;

		this->clockFrameTime = 0;
	}

	if (this->seconds > 59)
	{
		this->seconds = 0;

		this->minutes++;
	}

	if (this->minutes > 59)
	{
		this->minutes = 0;

		this->hours++;
	}

	if (this->hours > 24)
	{
		this->hours = 0;
	}

	this->clockTotalSeconds  = this->hours * 60 * 60;
	this->clockTotalSeconds += this->minutes * 60;
	this->clockTotalSeconds += this->seconds;
}

void FrameTime::IncrementHours()
{
	this->hours++;

	if (this->hours > 24)
	{
		this->hours = 0;
	}
}

void FrameTime::DecrementHours()
{
	this->hours--;

	if (this->hours > 24)
	{
		this->hours = 24;
	}
}

void FrameTime::IncrementMinutes()
{
	this->minutes++;

	if (this->minutes > 59)
	{
		this->minutes = 0;
	}
}

void FrameTime::DecrementMinutes()
{
	this->minutes--;

	if (this->minutes > 59)
	{
		this->minutes = 59;
	}
}

char* FrameTime::FormattedClock()
{
	sprintf(this->strTime, "%02i:%02i:%02i", this->hours, this->minutes, this->seconds);

	return this->strTime;
}

Timer::Timer()
{
	this->totalTime = 0;
}

Timer::Timer(FrameTime* ft, uint32_t r)
{
	this->frameTime = ft;

	this->reloadTime = r;

	this->totalTime = 0;
}

int Timer::Update()
{
	this->totalTime += this->frameTime->frameTime;

	if (this->totalTime >= this->reloadTime)
	{
		this->totalTime = 0;

		return 1;
	}

	return 0;
}

void Timer::Reset()
{
	this->totalTime = 0;
}

Button::Button(FrameTime* ft, uint8_t n, uint32_t r)
{
	this->frameTime = ft;

	this->number = n;

	this->holdTimer.frameTime  = ft;
	this->holdTimer.reloadTime = r;
	this->holdTimer.totalTime  = 0;

	pinMode(this->number, INPUT_PULLUP);

	this->count = 0;
}

int Button::ReadState()
{
	int value = digitalRead(this->number);

	if (value == 0)
	{
		if (this->count == 0)
		{
			this->count++;

			return 1;
		}

		if (this->holdTimer.reloadTime > 0)
		{
			if (this->holdTimer.Update())
			{
				return 1;
			}

			return 0;
		}

		return 0;
	}

	this->count = 0;

	this->holdTimer.Reset();

	return 0;
}

Device::Device(FrameTime* ft, int n, uint32_t d)
{
	this->frameTime = ft;
	this->number    = n;
	this->duration  = d;

	this->runTimer.frameTime  = ft;
	this->runTimer.reloadTime = this->duration;
	this->runTimer.totalTime  = 0;

	this->state = Device::idle;	

	pinMode(this->number, OUTPUT);

	digitalWrite(this->number, LOW);
}

void Device::Start()
{
	if (this->state == Device::running)
	{
		return;
	}

	this->state = Device::running;

	this->runTimer.Reset();

	strcpy(this->startTime, this->frameTime->FormattedClock());

	digitalWrite(this->number, HIGH);
}

void Device::Idle()
{
	if (this->state == Device::idle)
	{
		return;
	}

	this->state = Device::idle;

	digitalWrite(this->number, LOW);
}

void Device::Stop()
{
	if (this->state == Device::stop)
	{
		return;
	}

	this->state = Device::stop;

	digitalWrite(this->number, LOW);
}

int Device::Update()
{
	if (this->state != Device::running)
	{
		return 0;
	}

	if (this->runTimer.Update())
	{
		this->state = Device::ended;

		digitalWrite(this->number, LOW);

		return 1;
	}

	return 0;
}

char* Device::GetStateName()
{
	switch (this->state)
	{
		case  Device::idle:
		{	
			strcpy(strState, "Idle");

			break;
		}

		case  Device::start:
		{
			strcpy(strState, "Start");

			break;
		}

		case  Device::running:
		{
			strcpy(strState, "Run");

			break;
		}

		case Device::ended:
		{
			strcpy(strState, "Ended");

			break;
		}

		case  Device::stop:
		{
			strcpy(strState, "Stop");

			break;
		}
	}

	return strState;
}

AnalogSensor::AnalogSensor(int p, double Vin, double r1, double ss)
{
	// 1 / 1023 * 5
	// 0.004887585532746823069403714565
	this->multiplier = (1.0 / 1023.0) * Vin;

	this->pin = p;

	this->voltageIn = Vin;

	this->R1 = r1;

	this->senderSpec = ss / Vin;
}

void AnalogSensor::ReadState()
{
	this->rawIn = analogRead(this->pin);

	this->voltage = this->rawIn * this->multiplier;

	// R2 = (VOUT * R1) / (VIN - VOUT)
	this->R2 = (this->voltage * this->R1) / (this->voltageIn - this->voltage);
}

LED::LED(FrameTime* ft, uint8_t n, uint32_t t)
{
	this->blinkTimer.frameTime  = ft;
	this->blinkTimer.reloadTime = t;

	this->number = n;

	pinMode(this->number, OUTPUT);

	digitalWrite(this->number, LOW);
}

void LED::On()
{
	digitalWrite(this->number, HIGH);

	this->isOn = true;
}

void LED::Off()
{
	digitalWrite(this->number, LOW);

	this->isOn = false;
}

void LED::Blink()
{
	if (this->blinkTimer.Update())
	{
		if (this->isOn)
		{
			this->Off();
		}
		else
		{
			this->On();
		}
	}
}

void Storage::ClearSettings(void)
{
	eeprom_busy_wait();

	for (int i = 0; i < (int)this->maxSize; i++)
	{
		eeprom_update_byte((uint8_t*)i, 0x00);
	}

	eeprom_busy_wait();
}

void Storage::StoreSettings(void)
{
	this->tempAddr = this->settings;

	eeprom_busy_wait();

	for (int i = 0; i < (int)this->length; i++)
	{
		eeprom_update_byte((uint8_t*)i, *this->tempAddr);

		this->tempAddr++;
	}

	eeprom_busy_wait();
}

void Storage::ReadSettings(void)
{
	this->tempAddr = this->settings;

	eeprom_busy_wait();

	for (int i = 0; i < (int)this->length; i++)
	{
		*this->tempAddr = eeprom_read_byte((uint8_t*)i);

		this->tempAddr++;
	}

	eeprom_busy_wait();
}

double AnalogPin::ReadState()
{
	this->value = (double)analogRead(this->pin) * this->multiplier;

	return this->value;
}

int AnalogPin::ReadStateNoMultiplier()
{
	this->value = (double)analogRead(this->pin);

	return (int)this->value;
}

// Calculate resistance
// R2 = (VOUT * R1) / (VIN - VOUT)
// resistorIn and voltageIn are from the product's datasheet
double Resistance::GetOhms(double senderVoltage)
{
	this->resistance = (senderVoltage * this->resistorIn) / (this->voltageIn - senderVoltage);

	return this->resistance;
}

void I2CNetwork::Init()
{
	this->state = I2CNetwork::READY;

	// TWI Status Register
	_SFR_BYTE(TWSR) &= ~_BV(0);
	_SFR_BYTE(TWSR) &= ~_BV(1);

	// Set I2C Serial pins as output
	SDA.SetOutput();
	SCL.SetOutput();

	// TWI Bit Rate Register
	// twi bit rate formula from atmega128 manual pg 204
	// SCL Frequency = CPU Clock Frequency / (16 + (2 * TWBR))
	// note: TWBR should be 10 or higher for master mode
	// It is 72 for a 16mhz Wiring board with 100kHz TWI
	TWBR = ((F_CPU / I2CNetwork::SDAFREQ) - 16) / 2;

	// enable twi module, ack, and twi interrupt
	TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWEA);
}

uint8_t I2CNetwork::RxRead(uint8_t address, uint8_t* data, uint8_t length, uint8_t ss)
{
	// Ensure data will fit into buffer
	if (I2CNetwork::BUFFERLENGTH < length)
	{
		return 0;
	}

	// Master receiver
	// Wait until twi is ready
	uint32_t startMicros = micros();

	while(this->state != I2CNetwork::READY)
	{
		if ((I2CNetwork::TIMEOUT > 0ul) && ((micros() - startMicros) > I2CNetwork::TIMEOUT))
		{
			I2CNetwork::TimeoutReset();

			return 0;
		}
	}

	this->state = I2CNetwork::RX;

	this->sendStop = ss;

	this->err = I2CNetwork::TWICODE::NO_ERROR;

	// Build sla + w
	// Slave device address + w bit
	this->slarw = I2CNetwork::READ;

	this->slarw |= address << 1;

	// if we're in the repeated start state, then we've already sent the start,
	// (@@@ we hope), and the TWI state machine is just waiting for the address byte.
	// We need to remove ourselves from the repeated start state before we enable interrupts,
	// since the ISR is ASYNC, and we could get confused if we hit the ISR before cleaning
	// up. Also, don't enable the START interrupt. There may be one pending from the
	// repeated start that we sent ourselves, and that would really confuse things.
	if (this->inRepStart)
	{
		// remember, we're dealing with an ASYNC ISR
		this->inRepStart = false;

		startMicros = micros();

		do
		{
			TWDR = slarw;

			if ((I2CNetwork::TIMEOUT > 0ul) && ((micros() - startMicros) > I2CNetwork::TIMEOUT))
			{
				I2CNetwork::TimeoutReset();

				return 0;
			}

		} while(TWCR & _BV(TWWC));

		// enable INTs, but not START
		TWCR = _BV(TWINT) | _BV(TWEA) | _BV(TWEN) | _BV(TWIE);
	}
	else
	{
		// send start condition
		TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWEA) | _BV(TWINT) | _BV(TWSTA);
	}

	// wait for read operation to complete
	startMicros = micros();

	while(state == I2CNetwork::RX)
	{
		if ((I2CNetwork::TIMEOUT > 0ul) && ((micros() - startMicros) > I2CNetwork::TIMEOUT))
		{
			I2CNetwork::TimeoutReset();

			return 0;
		}
	}

	if (rxBufferIndex < length)
	{
		length = this->rxBufferIndex;
	}

	// copy twi buffer to data
	for(uint8_t i = 0; i < length; ++i)
	{
		data[i] = this->rxBuffer[i];
	}

	return 1;
}

uint8_t I2CNetwork::TxWrite(uint8_t address, uint8_t* data, uint8_t length, uint8_t wait, uint8_t ss)
{
	uint8_t i;

	// ensure data will fit into buffer
	if (length < I2CNetwork::BUFFERLENGTH)
	{
		return 1;
	}

	// wait until twi is ready
	uint32_t startMicros = micros();

	while(this->state != I2CNetwork::READY)
	{
		if ((I2CNetwork::TIMEOUT > 0ul) && ((micros() - startMicros) > I2CNetwork::TIMEOUT))
		{
			I2CNetwork::TimeoutReset();

			return 0;
		}
	}

	this->state = I2CNetwork::TX;

	this->sendStop = ss;

	this->err = I2CNetwork::TWICODE::NO_ERROR;

	// initialize buffer iteration vars
	this->txBufferIndex = 0;

	this->txBufferLength = length;

	// copy data to twi buffer
	for(i = 0; i < length; ++i)
	{
		this->txBuffer[i] = data[i];
	}

	// build sla + w, slave device address + w bit
	this->slarw = I2CNetwork::WRITE;

	this->slarw |= address << 1;

	// if we're in the repeated start state, then we've already sent the start,
	// (@@@ we hope), and the TWI state machine is just waiting for the address byte.
	// We need to remove ourselves from the repeated start state before we enable interrupts,
	// since the ISR is ASYNC, and we could get confused if we hit the ISR before cleaning
	// up. Also, don't enable the START interrupt. There may be one pending from the
	// repeated start that we sent ourselves, and that would really confuse things.

	// if we're in a repeated start, then we've already sent the START
	// in the ISR. Don't do it again.
	if (this->inRepStart)
	{
		this->inRepStart = false;			// remember, we're dealing with an ASYNC ISR

		startMicros = micros();

		do
		{
			TWDR = slarw;

			if ((I2CNetwork::TIMEOUT > 0ul) && ((micros() - startMicros) > I2CNetwork::TIMEOUT))
			{
				I2CNetwork::TimeoutReset();

				return 0;
			}
		} while(TWCR & _BV(TWWC));

		TWCR = _BV(TWINT) | _BV(TWEA) | _BV(TWEN) | _BV(TWIE);	// enable INTs, but not START
	}
	else
	{
		// send start condition
		TWCR = _BV(TWINT) | _BV(TWEA) | _BV(TWEN) | _BV(TWIE) | _BV(TWSTA);	// enable INTs
	}

	// wait for write operation to complete
	startMicros = micros();

	while(wait && (state == I2CNetwork::TX))
	{
		if ((I2CNetwork::TIMEOUT > 0ul) && ((micros() - startMicros) > I2CNetwork::TIMEOUT))
		{
			I2CNetwork::TimeoutReset();

			return 0;
		}
	}

	return 1;
}

void I2CNetwork::TimeoutReset()
{
	// remember bit rate and address settings
	uint8_t previous_TWBR = TWBR;
	uint8_t previous_TWAR = TWAR;

	// reset the interface
	I2CNetwork::Disable();

	I2CNetwork::Init();

	// reapply the previous register values
	TWAR = previous_TWAR;
	TWBR = previous_TWBR;
}

// disable twi module, ack, and twi interrupt
void I2CNetwork::Disable()
{
	TWCR &= ~(_BV(TWEN) | _BV(TWIE) | _BV(TWEA));

	// deactivate
	SDA.SetLow();
	SCL.SetLow();
}

// Transmit master read ready signal, with or without ack
void I2CNetwork::Reply(uint8_t ack)
{
	if (ack)
	{
		TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWINT) | _BV(TWEA);
	}
	else
	{
		TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWINT);
	}
}

// Send stop condition
void I2CNetwork::Stop()
{
	TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWEA) | _BV(TWINT) | _BV(TWSTO);

	// wait for stop condition to be executed on bus
	// TWINT is not set after a stop condition!
	// We cannot use micros() from an ISR, so approximate the timeout with cycle-counted delays
	const uint8_t us_per_loop = 8;

	uint32_t counter = (I2CNetwork::TIMEOUT + us_per_loop - 1) / us_per_loop; // Round up

	while(TWCR & _BV(TWSTO))
	{
		if (I2CNetwork::TIMEOUT > 0ul)
		{
			if (counter > 0ul)
			{
				_delay_us(10);

				counter--;

			}
			else
			{
				I2CNetwork::TimeoutReset();

				return;
			}
		}
	}

	// update twi state
	this->state = I2CNetwork::READY;
}

void I2CNetwork::ReleaseBus()
{
	// release bus
	TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWEA) | _BV(TWINT);
}

// The lower 3 bits of TWSR are reserved on the ATmega163.
// The 2 LSB carry the prescaler bits on the newer ATmegas.
#define STATUS_MASK	(_BV(TWS7) | _BV(TWS6) | _BV(TWS5) | _BV(TWS4) | _BV(TWS3))
#define STATUS		(TWSR & STATUS_MASK)

void I2CNetwork::CheckISR()
{
	// Evaluate the status register
	switch(STATUS)
	{
		// All Master
		case I2CNetwork::TWICODE::START:     // sent start condition
		case I2CNetwork::TWICODE::REP_START: // sent repeated start condition
		{
			// copy device address and r/w bit to output register
			TWDR = this->slarw;

			// ack
			I2CNetwork::Reply(1);

			break;
		}

		// ==========================================================================================================================
		// Master Transmitter
		case I2CNetwork::TWICODE::MT_SLA_ACK:  // slave receiver ack address
		case I2CNetwork::TWICODE::MT_DATA_ACK: // slave receiver ack data
		{
			// if there is data to send, send it, otherwise stop
			if (this->txBufferIndex < I2CNetwork::BUFFERLENGTH)
			{
				// copy data to output register
				TWDR = this->txBuffer[this->txBufferIndex++];

				// ack
				I2CNetwork::Reply(1);
			}
			else
			{
				if (sendStop)
				{
					I2CNetwork::Stop();
				}
				else
				{
					// Need to perform a repeated start
					this->inRepStart = true;

					// don't enable the interrupt. We'll generate the start, but we
					// avoid handling the interrupt until we're in the next transaction,
					// at the point where we would normally issue the start.
					TWCR = _BV(TWINT) | _BV(TWSTA) | _BV(TWEN);

					this->state = I2CNetwork::READY;
				}
			}

			break;

		}

		// address sent, nack received
		case I2CNetwork::TWICODE::MT_SLA_NACK:
		{
			this->err = I2CNetwork::TWICODE::MT_SLA_NACK;

			I2CNetwork::Stop();

			break;
		}

		// data sent, nack received
		case I2CNetwork::TWICODE::MT_DATA_NACK:
		{
			this->err = I2CNetwork::TWICODE::MT_DATA_NACK;

			I2CNetwork::Stop();

			break;
		}

		// lost bus arbitration
		case I2CNetwork::TWICODE::MT_ARB_LOST:
		{
			this->err = I2CNetwork::TWICODE::MT_ARB_LOST;

			I2CNetwork::ReleaseBus();

			// bus ready for next transaction
			this->state = I2CNetwork::READY;

			break;

		}

		// ==========================================================================================================================
		// Master Receiver
		// data received, ack sent
		case I2CNetwork::TWICODE::MR_DATA_ACK:
		{
			// put byte into buffer
			this->rxBuffer[this->rxBufferIndex++] = TWDR;

			/* fall through */
			__attribute__ ((fallthrough));
		}

		// address sent, ack received
		case I2CNetwork::TWICODE::MR_SLA_ACK:
		{
			// ack if more bytes are expected, otherwise nack
			if (this->rxBufferIndex < I2CNetwork::BUFFERLENGTH)
			{
				I2CNetwork::Reply(1);
			}
			else
			{
				I2CNetwork::Reply(0);
			}

			break;
		}

		// data received, nack sent
		case I2CNetwork::TWICODE::MR_DATA_NACK:
		{
			// put final byte into buffer
			this->rxBuffer[rxBufferIndex++] = TWDR;

			if (this->sendStop)
			{
				I2CNetwork::Stop();
			}
			else
			{
				// Need to perform a repeated start
				this->inRepStart = true;

				// don't enable the interrupt. We'll generate the start, but we
				// avoid handling the interrupt until we're in the next transaction,
				// at the point where we would normally issue the start.
				TWCR = _BV(TWINT) | _BV(TWSTA) | _BV(TWEN);

				this->state = I2CNetwork::READY;
			}

			break;
		}

		// address sent, nack received
		case I2CNetwork::TWICODE::MR_SLA_NACK:
		{
			I2CNetwork::Stop();

			break;
		}

		// ==========================================================================================================================
		// I2CNetwork::TWICODE::MR_ARB_LOST handled by I2CNetwork::TWICODE::MT_ARB_LOST case

		// Slave Receiver
		case I2CNetwork::TWICODE::SR_SLA_ACK:				// addressed, returned ack
		case I2CNetwork::TWICODE::SR_GCALL_ACK:				// addressed generally, returned ack
		case I2CNetwork::TWICODE::SR_ARB_LOST_SLA_ACK:		// lost arbitration, returned ack
		case I2CNetwork::TWICODE::SR_ARB_LOST_GCALL_ACK:	// lost arbitration, returned ack
		{
			// receive buffer can be overwritten
			this->rxBufferIndex = 0;

			// ack ready to receive
			I2CNetwork::Reply(1);

			break;
		}

		case I2CNetwork::TWICODE::SR_DATA_ACK:       // data received, returned ack
		case I2CNetwork::TWICODE::SR_GCALL_DATA_ACK: // data received generally, returned ack
		{
			// if there is still room in the receive buffer
			if (this->rxBufferIndex < I2CNetwork::BUFFERLENGTH)
			{
				// put byte in buffer
				this->rxBuffer[this->rxBufferIndex++] = TWDR;

				// ack to receive next byte
				I2CNetwork::Reply(1);
			}
			else
			{
				// otherwise nack
				I2CNetwork::Reply(0);
			}

			break;
		}

		// stop or repeated start condition received
		case I2CNetwork::TWICODE::SR_STOP:
		{
			I2CNetwork::ReleaseBus();

			// put a null after data if there's room
			if (this->rxBufferIndex < I2CNetwork::BUFFERLENGTH)
			{
				this->rxBuffer[this->rxBufferIndex] = 0x00;
			}

			// bus ready for next transaction
			this->state = I2CNetwork::READY;

			break;
		}

		case I2CNetwork::TWICODE::SR_DATA_NACK:       // data received, returned nack
		case I2CNetwork::TWICODE::SR_GCALL_DATA_NACK: // data received generally, returned nack
		{
			// nack back at master
			I2CNetwork::Reply(0);

			break;
		}

		// ==========================================================================================================================
		// Slave Transmitter
		case I2CNetwork::TWICODE::ST_SLA_ACK:          // addressed, returned ack
		case I2CNetwork::TWICODE::ST_ARB_LOST_SLA_ACK: // arbitration lost, returned ack
		{
			// ready the transmit buffer index for iteration
			this->txBufferIndex = 0;

			/* fall through */
			__attribute__ ((fallthrough));
		}

		// transmit byte from buffer
		case I2CNetwork::TWICODE::ST_DATA_ACK: // byte sent, ack returned
		{
			// copy data to output register
			TWDR = this->txBuffer[this->txBufferIndex++];

			// if there is more to send, ack, otherwise nack
			if (this->txBufferIndex < I2CNetwork::BUFFERLENGTH)
			{
				I2CNetwork::Reply(1);
			}
			else
			{
				I2CNetwork::Reply(0);
			}

			break;
		}

		// ==========================================================================================================================
		case I2CNetwork::TWICODE::ST_DATA_NACK: // received nack, we are done
		case I2CNetwork::TWICODE::ST_LAST_DATA: // received ack, but we are done already!
		{
			// ack future responses
			I2CNetwork::Reply(1);

			// bus ready for next transaction
			this->state = I2CNetwork::READY;

			break;
		}

		// ==========================================================================================================================
		// All
		case I2CNetwork::TWICODE::NO_INFO:   // no state information
		{
			break;
		}

		case I2CNetwork::TWICODE::BUS_ERROR: // bus error, illegal stop/start
		{
			this->err = I2CNetwork::TWICODE::BUS_ERROR;

			I2CNetwork::Stop();

			break;
		}
	}
}

// Default 4x4 Numeric Keypad
NumPad::NumPad()
{
	memset(this, 0x00, sizeof(NumPad));

	// Keymap
	memcpy((void*)this->keymap, (void*)"123A456B789C*0#D", 16);

	this->numRows = 4;
	this->numCols = 4;

	// Terminal connections
	// Arduino pin 13 is physical pin 19
	for (uint8_t r = 13; r <= 10;r++)
	{
		this->rowPins[r].Init(r, OUTPUT);
	}

	for (uint8_t c = 9;c <= 6;c++)
	{
		this->colPins[c].Init(c, OUTPUT);
	}
}

NumPad::NumPad(uint8_t r, uint8_t c, char* km, int* rp, int* cp)
{
	memset(this, 0x00, sizeof(NumPad));

	// Keymap
	memcpy((void*)this->keymap, (void*)km, strlen(km));

	this->numRows = r;
	this->numCols = c;

	// Terminal connections
	for (uint8_t row = 0; row < numRows;row++)
	{
		this->rowPins[row].Init(rp[row], OUTPUT);
	}

	for (uint8_t col = 0;col < numCols;col++)
	{
		this->colPins[col].Init(cp[col], OUTPUT);
	}
}

char NumPad::GetKey()
{
	this->noKey = true;

	this->key = this->NOKEY;

	for(uint8_t r = 0;r < numRows;r++)
	{
		this->rowPins[r].SetLow();

		for(uint8_t c = 0;c < numCols;c++)
		{
			if (colPins[c].ReadState() == LOW)
			{
				this->key = this->keymap[r][c];

				this->keyCount++;

				this->noKey = false;

				this->colPins[c].SetHigh();
				this->rowPins[r].SetHigh();

				break;
			}
		}

		this->rowPins[r].SetHigh();
	}

	if (this->noKey)
	{
		this->noKey = false;

		this->keyCount = 0;
	}

	if (this->keyCount > 1)
	{
		return this->NOKEY;
	}

	return this->key;
}

MyString::MyString(char* c)
{
	memset(this, 0x00, sizeof(MyString));

	this->size = sizeof(c);

	if (this->size >= MyString::MAXSIZE)
	{
		this->size = MyString::MAXSIZE - 1;
	}

	memcpy(this->value, c, this->size);
}

void MyString::DoubleFloatToStr(double d, uint8_t precision)
{
	memset(this, 0x00, sizeof(MyString));

	dtostrf(d, this->size, precision, this->value);
}
