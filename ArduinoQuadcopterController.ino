#include <Wire.h>
#include "MPU6050.h"

#define RECEIVER_CHANNEL_0_MASK _BV(PB0)//B00000001
#define RECEIVER_CHANNEL_1_MASK _BV(PB1)//B00000010
#define RECEIVER_CHANNEL_2_MASK _BV(PB2)//B00000100
#define RECEIVER_CHANNEL_3_MASK _BV(PB3)//B00001000
#define RECEIVER_CHANNEL_4_MASK _BV(PJ1)

#define ESC1_REGISTER OCR3A
#define ESC2_REGISTER OCR4A
#define ESC3_REGISTER OCR4B
#define ESC4_REGISTER OCR4C
#define SERVO1_REGISTER OCR5A
#define SERVO2_REGISTER OCR5B
#define SERVO3_REGISTER OCR5C

#define _SETBIT(port, bit) (port |= _BV(bit))
#define _CLEARBIT(port, bit) (port &= ~_BV(bit))
#define _TOGGLEBIT(port, bit) ( bit_is_set(port, bit) ? _CLEARBIT(port, bit) : _SETBIT(port, bit) )

typedef struct PID {
	double pGain;
	double iGain;
	double dGain;
	int maxValue;

	double iState;
	double dState;
};

//typedef struct PIDValue
//{
//	double pValue;
//	double iValue;
//	double dValue;
//	double lastError;
//};

typedef struct Attitude
{
	double pitch;
	double roll;
	double yaw;
};

typedef struct ISREvent
{
	unsigned long clock;
	uint8_t state;
};

enum State { disarmed, armed, flying };
enum Mode { acro, attitude };

State state = disarmed;
Mode mode = acro;

int receiverInput[5] = { 0, 0, 0, 0, 0 };
int reverseInputChannel[5] = { -1, 1, 1, -1, 1 };
int escOutput[4] = { 0, 0, 0, 0 };

Attitude gyroInput = { 0, 0, 0 };
Attitude setPoint = { 0, 0, 0 };
Attitude correction = { 0, 0, 0 };
Attitude currentAttitude = { 0, 0, 0 };

int attMaxRoll = 60;
int attMaxRollRate = 150;
double attHoldGain = 3;

//double gyroRollInput, gyroPitchInput, gyroYawInput;
//double pitchSetPoint, rollSetPoint, yawSetPoint;
//double pitch, roll, yaw;
int throttle;
int idleSpeed = 1150;

//bool isArmed = false;
bool isArming = false;
int ledBlinkDuration = 0;

unsigned long currentTime = 0;
unsigned long lastLedBlinkUpdateTime = 0;

PID rollPID = { 2.0, 0.04, 15.0, 400, 0, 0 };
PID pitchPID = rollPID;//{ 1.4, 0.01, 15.0, 400 };
PID yawPID = { 3.0, 0.02, 0.0, 400, 0, 0 };

//PIDValue pitchPIDValue = { 0, 0, 0, 0 };
//PIDValue rollPIDValue = { 0, 0, 0, 0 };
//PIDValue yawPIDValue = { 0, 0, 0, 0 };

unsigned long isr_currentTime = 0;
unsigned long isr_currentClock = 0;
byte isr_receiverLastPinValue[5] = { 0, 0, 0, 0, 0 };
unsigned long isr_receiverPulseStartTime[5] = { 0, 0, 0, 0, 0 };
unsigned long isr_receiverPulseEndTime[5] = { 0, 0, 0, 0, 0 };

volatile uint16_t isr_eventClock[16];
volatile uint8_t isr_eventState[16];
volatile uint8_t isr_currentEventIndex;
uint8_t isr_processingEventIndex;

//double att_gyro_roll = 0;
//double att_gyro_pitch = 0;
double att_acc_roll = 0;
double att_acc_pitch = 0;
unsigned long att_currentTime = 0;
unsigned long att_lastUpdate = 0;

double mpu_gyroOffset_x = -2.15;
double mpu_gyroOffset_y = -0.14;
double mpu_gyroOffset_z = -0.18;

double mpu_accel_x, mpu_accel_y, mpu_accel_z, mpu_temprature;
double mpu_rot_x, mpu_rot_y, mpu_rot_z;
unsigned long mpu_lastUpdate = 0;
bool mpu_calibrated = false;

void setupInterrupts()
{
	//cli();

	//_CLEARBIT(TIMSK0, TOIE0);



	// configure timer 3 and timer 4
	// timer 3 controls pins 2, 3, 5
	// timer 4 controls pins 6, 7, 8
	// timer 5 controls pins 46, 45, 44

	// set fastPWM mode, prescaler 1/8
	// set the TOP value for timer counter to ICR3 register value
	// this results in timer counting from 0 to ICR3 value with (tick_length = 1 / (16MHz / 8) = 0.5 microseconds
	TCCR3A = (_BV(COM3A1) | _BV(COM3B1) | _BV(COM3C1) | _BV(WGM31));
	TCCR3B = (_BV(WGM33) | _BV(WGM32) | _BV(CS31));

	TCCR4A = (_BV(COM4A1) | _BV(COM4B1) | _BV(COM4C1) | _BV(WGM41));
	TCCR4B = (_BV(WGM43) | _BV(WGM42) | _BV(CS41));

	TCCR5A = (_BV(COM5A1) | _BV(COM5B1) | _BV(COM5C1) | _BV(WGM51));
	TCCR5B = (_BV(WGM53) | _BV(WGM52) | _BV(CS51));

	Serial.print(TCCR0A);
	Serial.print("\t");
	Serial.print(TCCR1A);
	Serial.print("\t");
	Serial.print(TCCR2A);
	Serial.print("\t");
	Serial.print(TCCR3A);
	Serial.print("\t");
	Serial.print(TCCR4A);
	Serial.print("\t");
	Serial.print(TCCR5A);
	Serial.print("\n");
	Serial.print(TCCR0B);
	Serial.print("\t");
	Serial.print(TCCR1B);
	Serial.print("\t");
	Serial.print(TCCR2B);
	Serial.print("\t");
	Serial.print(TCCR3B);
	Serial.print("\t");
	Serial.print(TCCR4B);
	Serial.print("\t");
	Serial.print(TCCR5B);
	Serial.print("\n");

	// set the ICR3 register value which is used as the timer counter TOP value
	// with tick_length = 0.5 microseconds and ICR3 = 7999 this gives us 4 millisecond period or 250Hz PWM frequency
	ICR3 = 7999;
	ICR4 = 7999;

	// set timer 5 PWM frequency to 50Hz to drive servos
	ICR5 = 39999;

	// enable timer 3 overflow interrupt
	//_SETBIT(TIMSK3, TOIE3);

	// configure interrupt for receiver input on pins 50, 51, 52, 53
	_SETBIT(PCICR, PCIE0);
	_SETBIT(PCICR, PCIE1);
	_SETBIT(PCMSK0, PCINT0);
	_SETBIT(PCMSK0, PCINT1);
	_SETBIT(PCMSK0, PCINT2);
	_SETBIT(PCMSK0, PCINT3);
	_SETBIT(PCMSK1, PCINT10);

	//sei();
}


void setup()
{
	pinMode(13, OUTPUT);
	pinMode(10, OUTPUT);

	Serial.begin(57600);
	Wire.begin();

	Serial.print("Roll PID:\t");
	Serial.print(rollPID.pGain);
	Serial.print("\t");
	Serial.print(rollPID.iGain);
	Serial.print("\t");
	Serial.print(rollPID.dGain);
	Serial.print("\n");
	Serial.print("Pitch PID:\t");
	Serial.print(pitchPID.pGain);
	Serial.print("\t");
	Serial.print(pitchPID.iGain);
	Serial.print("\t");
	Serial.print(pitchPID.dGain);
	Serial.print("\n");
	Serial.print("Yaw PID:\t");
	Serial.print(yawPID.pGain);
	Serial.print("\t");
	Serial.print(yawPID.iGain);
	Serial.print("\t");
	Serial.print(yawPID.dGain);
	Serial.print("\n");

	// use pins 5, 6, 7, 8 for PWM output
	pinMode(5, OUTPUT);
	pinMode(6, OUTPUT);
	pinMode(7, OUTPUT);
	pinMode(8, OUTPUT);
	pinMode(44, OUTPUT);
	pinMode(45, OUTPUT);
	pinMode(46, OUTPUT);

	pinMode(41, OUTPUT);

	MpuInit();
	MPU6050_Test_I2C();

	Serial.print("MPU init successfull\n");

	setupInterrupts();

	// set initial PWM pulse length to ESCs
	//setESC(1, 1000);
	//setESC(2, 1000);
	//setESC(3, 1000);
	//setESC(4, 1000);

	setPWM(ESC1_REGISTER, 1000);
	setPWM(ESC2_REGISTER, 1000);
	setPWM(ESC3_REGISTER, 1000);
	setPWM(ESC4_REGISTER, 1000);

	setPWM(SERVO1_REGISTER, 1520);
	setPWM(SERVO2_REGISTER, 1500);
	setPWM(SERVO3_REGISTER, 1530);

	calibrateGyro();


	// wait for correct receiver inputs
	Serial.print("Waiting for the receiver signal\n");
	while (receiverInput[0] < 1000 ||
		receiverInput[1] < 1000 ||
		receiverInput[2] < 1000 ||
		receiverInput[3] < 1000)
	{
		//printReceiverInput();
		processInterruptEvents();
		delayMicroseconds(1000);
	}
	Serial.print("Receiver signal aquired\n");

	currentTime = micros();
}

void loop()
{
	// force the loop to run at exactly 250Hz to get a steady sampling rate
	while (micros() < currentTime + 4000) 
	{
	}


	//Serial.print(isr_lastEventIndex);
	//Serial.print("\n");

	currentTime = micros();

	_TOGGLEBIT(PORTG, PG0);


	
	processInterruptEvents();
	updateState();
	blinkLed();
	updateMPUReadings();

	updateAttitude();

	// use 80/20 complementary filter on gyro input

	gyroInput.roll = gyroInput.roll * 0.8 + mpu_rot_x * 0.2;
	gyroInput.pitch = gyroInput.pitch * 0.8 + mpu_rot_y * 0.2;
	gyroInput.yaw = gyroInput.yaw * 0.8 + mpu_rot_z * -0.2;

	//gyroInput.roll = mpu_rot_x;
	//gyroInput.pitch = mpu_rot_y;
	//gyroInput.yaw = mpu_rot_z * -1;

	throttle = receiverInput[2];

	// leave a bit of headroom for stabilization
	if (throttle > 1800)
		throttle = 1800;

	//Serial.print(toAngle(receiverInput[0], reverseInputChannel[0]));
	//Serial.print("\t");
	//Serial.print(toAngle(receiverInput[1], reverseInputChannel[1]));
	//Serial.print("\n");

	if (state == flying)
	{
		//rollSetPoint = toAngularVelocity(receiverInput[0], reverseInputChannel[0]);
		//pitchSetPoint = toAngularVelocity(receiverInput[1], reverseInputChannel[1]);
		//yawSetPoint = toAngularVelocity(receiverInput[3], reverseInputChannel[3]);

		mode = (receiverInput[4] < 1500) ? acro : attitude;

		//Serial.print(mode);
		//Serial.print("\n");

		switch (mode)
		{
			case acro:
				setPoint.roll = toAngularVelocity(receiverInput[0], reverseInputChannel[0]);
				setPoint.pitch = toAngularVelocity(receiverInput[1], reverseInputChannel[1]);
				setPoint.yaw = toAngularVelocity(receiverInput[3], reverseInputChannel[3]);
				break;
			case attitude:
				setPoint.roll = (toAngle(receiverInput[0], reverseInputChannel[0]) - currentAttitude.roll) * attHoldGain;
				setPoint.pitch = (toAngle(receiverInput[1], reverseInputChannel[1]) - currentAttitude.pitch) * attHoldGain;
				setPoint.yaw = toAngularVelocity(receiverInput[3], reverseInputChannel[3]);

				if (setPoint.roll > attMaxRollRate)
					setPoint.roll = attMaxRollRate;
				else if (setPoint.roll < attMaxRollRate * -1)
					setPoint.roll = attMaxRollRate * -1;

				if (setPoint.pitch > attMaxRollRate)
					setPoint.pitch = attMaxRollRate;
				else if (setPoint.pitch < attMaxRollRate * -1)
					setPoint.pitch = attMaxRollRate * -1;

				break;
		}
		//pitch = calculatePID(pitchPID, gyroPitchInput, pitchSetPoint, pitchPIDValue);
		//roll = calculatePID(rollPID, gyroRollInput, rollSetPoint, rollPIDValue);
		//yaw = calculatePID(yawPID, gyroYawInput, yawSetPoint, yawPIDValue);

		correction.pitch = calculatePID(&pitchPID, gyroInput.pitch, setPoint.pitch);
		correction.roll = calculatePID(&rollPID, gyroInput.roll, setPoint.roll);
		correction.yaw = calculatePID(&yawPID, gyroInput.yaw, setPoint.yaw);

		escOutput[0] = throttle + correction.pitch + correction.roll - correction.yaw;
		escOutput[1] = throttle + correction.pitch - correction.roll + correction.yaw;
		escOutput[2] = throttle - correction.pitch - correction.roll - correction.yaw;
		escOutput[3] = throttle - correction.pitch + correction.roll + correction.yaw;

		// limit esc outputs to correct values
		for (int i = 0; i < 4; i++)
		{
			if (escOutput[i] < idleSpeed)
				escOutput[i] = idleSpeed;
			else if (escOutput[i] > 2000)
				escOutput[i] = 2000;
		}

		
	}
	else if (state == armed)
	{
		escOutput[0] =	idleSpeed;
		escOutput[1] =	idleSpeed;
		escOutput[2] =	idleSpeed;
		escOutput[3] =	idleSpeed;
	}
	else
	{
		escOutput[0] = 1000;
		escOutput[1] = 1000;
		escOutput[2] = 1000;
		escOutput[3] = 1000;
	}



	
	//setESC(1, escOutput[0]);
	//setESC(2, escOutput[1]);
	//setESC(3, escOutput[2]);
	//setESC(4, escOutput[3]);
	
	setPWM(ESC1_REGISTER, escOutput[0]);
	setPWM(ESC2_REGISTER, escOutput[1]);
	setPWM(ESC3_REGISTER, escOutput[2]);
	setPWM(ESC4_REGISTER, escOutput[3]);

	//setPWM(SERVO2_REGISTER, servoDegreesToPulseLength(currentAttitude.pitch * -1));

	//setPWM(SERVO1_REGISTER, receiverInput[0]);
	//setPWM(SERVO2_REGISTER, receiverInput[1]);
	//setPWM(SERVO3_REGISTER, receiverInput[3]);

	//printSetPoints();
	//printAttitude();
	//printReceiverInput();
	//printGyroInputs();
	//printError();
	//printControllerOutput();
	//printESCOutputs();
	//printMPUData();
	
	//Serial.print(micros() - currentTime);
	
	//Serial.print("\n");

	if (micros() > currentTime + 4000)
	{
		_SETBIT(PORTB, PB4);
		//Serial.print("Warning! Main loop takes more than 4 microseconds!\n");
	}
	else
	{
		_CLEARBIT(PORTB, PB4);
	}

	
}

void updateAttitude()
{
	unsigned long currentTime = micros();
	// get the time passed since the last attutude update
	double time =(double)(currentTime - att_lastUpdate) / 1000000;

	att_lastUpdate = currentTime;

	double change_x = time * mpu_rot_x;
	double change_y = time * mpu_rot_y;
	double change_z = time * mpu_rot_z;
	double totalAccel = sqrt(mpu_accel_x*mpu_accel_x + mpu_accel_y*mpu_accel_y + mpu_accel_z*mpu_accel_z);
	
	currentAttitude.roll += change_x;
	currentAttitude.pitch += change_y;
	
	currentAttitude.roll += currentAttitude.pitch * sin(change_z * PI / 180);
	currentAttitude.pitch -= currentAttitude.roll * sin(change_z * PI / 180);

	att_acc_roll = asin(mpu_accel_y / totalAccel) * 180 / PI;
	att_acc_pitch = asin(mpu_accel_x / totalAccel) * 180 / PI;

	currentAttitude.roll = (0.996 * currentAttitude.roll + 0.004 * att_acc_roll);
	currentAttitude.pitch = (0.996 * currentAttitude.pitch + 0.004 * att_acc_pitch);

	//Serial.print(att_gyro_roll);
	//Serial.print("\t");
	//Serial.print(att_acc_roll);
	//Serial.print("\t");
	//Serial.println(currentAttitude.roll);
	//Serial.print("\n");

	//currentAttitude.roll = att_gyro_roll;
	//currentAttitude.pitch = att_gyro_pitch;

	//currentAttitude.roll = att_acc_roll;
	//currentAttitude.pitch = att_acc_pitch;

	//Serial.print(currentTime - lastAttitudeUpdateTime);
}

void updateState()
{

	// if all inputs are put in the lowest position, start arming/disarming procedure
	if ((state == disarmed || state == armed) &&
		receiverInput[0] < 1050 &&
		receiverInput[1] < 1050 &&
		receiverInput[2] < 1050 &&
		receiverInput[3] < 1050)
	{
		isArming = true;
	}

	// if throttle is in the lowest position and all other inputs somwhere else
	if (isArming &&
		receiverInput[0] > 1050 &&
		receiverInput[1] > 1050 &&
		receiverInput[2] < 1050 &&
		receiverInput[3] > 1050)
	{
		isArming = false;
		state = (state == disarmed) ? armed : disarmed;
		Serial.print((state == armed) ? "ARMED\n" : "DISARMED\n");
	}

	if (state == armed &&
		receiverInput[2] > 1050)
	{
		// clear PID memory before takeoff
		pitchPID.iState = 0;
		pitchPID.dState = 0;
		rollPID.iState = 0;
		rollPID.dState = 0;
		yawPID.iState = 0;
		yawPID.dState = 0;

		//pitchPIDValue = { 0, 0, 0, 0 };
		//rollPIDValue = { 0, 0, 0, 0 };
		//yawPIDValue = { 0, 0, 0, 0 };

		state = flying;
	}

	if (state == flying &&
		receiverInput[2] < 1050)
	{
		state = armed;
	}

	if (isArming)
		ledBlinkDuration = 50;
	else switch (state)
	{
		case disarmed:
			ledBlinkDuration = 300;
			break;
		default: 
			ledBlinkDuration = 0;
			break;
	}
}

void updateMPUReadings()
{
	mpu_lastUpdate = micros();
	MpuGetMeasurments(&mpu_rot_x, &mpu_rot_y, &mpu_rot_z, &mpu_accel_x, &mpu_accel_y, &mpu_accel_z, &mpu_temprature);
	// adjust gyro measurments
	mpu_rot_x -= mpu_gyroOffset_x;
	mpu_rot_y -= mpu_gyroOffset_y;
	mpu_rot_z -= mpu_gyroOffset_z;

	mpu_accel_x *= -1;
}

double calculatePID(PID * pid, double gyroInput, double setPoint)
{
	double error = gyroInput - setPoint;
	double pTerm, iTerm, dTerm;
	//value.iValue += pid.iGain * error;
	pid->iState += pid->iGain * error;

	//if (value.iValue > pid.maxValue)
	//	value.iValue = pid.maxValue;
	//else if (value.iValue < pid.maxValue * -1)
	//	value.iValue = pid.maxValue * -1;

	if (pid->iState > pid->maxValue)
		pid->iState = pid->maxValue;
	else if (pid->iState < pid->maxValue * -1)
		pid->iState = pid->maxValue * -1;

	pTerm = pid->pGain * error;
	iTerm = pid->iState;
	dTerm = pid->dGain * (gyroInput - pid->dState);

	pid->dState = gyroInput;

	//value.dValue = pid.dGain * (error - value.lastError);
	//value.lastError = error;

	//double result = value.pValue + value.iValue + value.dValue;

	//if (result > pid.maxValue)
	//	result = pid.maxValue;
	//else if (result < pid.maxValue * -1)
	//	result = pid.maxValue * -1;

	double result = pTerm + iTerm + dTerm;

	if (result > pid->maxValue)
		result = pid->maxValue;
	else if (result < pid->maxValue * -1)
		result = pid->maxValue * -1;

	return result;
}

double toAngularVelocity(int receiverInput, int reverceChannel)
{
	if (receiverInput > 1508)
		return ((receiverInput - 1508) / 2.5) * reverceChannel;
	else if (receiverInput < 1492)
		return ((receiverInput - 1492) / 2.5) * reverceChannel;
}

int toAngle(int receiverInput, int reverceChannel)
{
	if (receiverInput > 1508)
		return ((receiverInput - 1508) * ((double)attMaxRoll / 492)) * reverceChannel;
	else if (receiverInput < 1492)
		return ((receiverInput - 1492) * ((double)attMaxRoll / 492)) * reverceChannel;
	else
		return 0;
}

int servoDegreesToPulseLength(double degrees)
{
	double maxAngle = 30;
	double offset = -3;
	double gain = 0.8;
	int oneDegreePulse = (500 / (maxAngle)) * gain;

	if (degrees > maxAngle)
		degrees = maxAngle;
	else if (degrees < maxAngle * -1)
		degrees = maxAngle * -1;

	return 1500 + (degrees - offset) * oneDegreePulse;
}

void blinkLed()
{
	unsigned long blink_currentTime = millis();
	if (ledBlinkDuration == 0)
	{
		_SETBIT(PORTB, PB7);
		return;
	}

	if (blink_currentTime > lastLedBlinkUpdateTime + ledBlinkDuration / 2)
	{
		lastLedBlinkUpdateTime = blink_currentTime;
		//digitalWrite(13, !digitalRead(13));
		bit_is_set(PORTB, PB7) ? _CLEARBIT(PORTB, PB7) : _SETBIT(PORTB, PB7);
	}
}

void setESC(int escNumber, int pulseLength)
{
	if (pulseLength < 1000)
		pulseLength = 1000;

	if (pulseLength > 2000)
		pulseLength = 2000;

	switch (escNumber)
	{
		case 1:
			ESC1_REGISTER = pulseLength * 2;
			break;
		case 2:
			ESC2_REGISTER = pulseLength * 2;
			break;
		case 3:
			ESC3_REGISTER = pulseLength * 2;
			break;
		case 4:
			ESC4_REGISTER = pulseLength * 2;
			break;
	}
}

void setPWM(volatile uint16_t &reg, int pulseLength)
{
	if (pulseLength < 1000)
		pulseLength = 1000;

	if (pulseLength > 2000)
		pulseLength = 2000;

	reg = pulseLength * 2;
}

__attribute__((always_inline)) void updateReceiverChannelInput(int const &port, int const &pinRegister, int const &channel, unsigned long const &currentClock)
{
	// if pin is HIGH
	if (port & pinRegister)
	{
		if (isr_receiverLastPinValue[channel] == 0)
		{
			// save current pin value
			isr_receiverLastPinValue[channel] = 1;

			// save the pulse start time
			isr_receiverPulseStartTime[channel] = currentClock; //currentTime;
		}
	}
	// if pin is LOW
	else
	{
		// if pin value changed from 1 to 0
		if (isr_receiverLastPinValue[channel] == 1)
		{
			// save current pin value
			isr_receiverLastPinValue[channel] = 0;

			// save the pulse duration
			receiverInput[channel] = currentClock > isr_receiverPulseStartTime[channel]
				? (currentClock - isr_receiverPulseStartTime[channel]) / 2 // note: 1 timer tick = 0.5 microseconds
				: ((ICR5 - isr_receiverPulseStartTime[channel]) + currentClock) / 2; // in case counter overflow happens
				//currentTime - isr_receiverPulseStartTime[channel];
		}
	}
}

void processInterruptEvents()
{
	while (isr_processingEventIndex != isr_currentEventIndex)
	{
		uint8_t state = isr_eventState[isr_processingEventIndex];
		unsigned long clock = isr_eventClock[isr_processingEventIndex];

		updateReceiverChannelInput(state, RECEIVER_CHANNEL_0_MASK, 0, clock);
		updateReceiverChannelInput(state, RECEIVER_CHANNEL_1_MASK, 1, clock);
		updateReceiverChannelInput(state, RECEIVER_CHANNEL_2_MASK, 2, clock);
		updateReceiverChannelInput(state, RECEIVER_CHANNEL_3_MASK, 3, clock);

		isr_processingEventIndex = (isr_processingEventIndex + 1) & B1111;
	}
}

// pin change interrupt subroutine
// this subroutine is called each time pin 50, 51, 52 or 53 changes value
ISR(PCINT0_vect)
{
	//isr_currentTime = micros();
	// get the current clock from timer 5 that runs with 1/8 prescaler
	// that lets us measure time with accuracy of 0.5 microseconds
	//isr_currentClock = TCNT5;

	//isr_events[isr_lastEventIndex].clock = TCNT5;
	//isr_events[isr_lastEventIndex].state = PINB;
	isr_eventClock[isr_currentEventIndex] = TCNT5;
	isr_eventState[isr_currentEventIndex] = PINB;
	isr_currentEventIndex = (isr_currentEventIndex + 1) & B1111;

	//updateReceiverChannelInput(PINB, RECEIVER_CHANNEL_0_MASK, 0, isr_currentClock);
	//updateReceiverChannelInput(PINB, RECEIVER_CHANNEL_1_MASK, 1, isr_currentClock);
	//updateReceiverChannelInput(PINB, RECEIVER_CHANNEL_2_MASK, 2, isr_currentClock);
	//updateReceiverChannelInput(PINB, RECEIVER_CHANNEL_3_MASK, 3, isr_currentClock);

	//if (PINB & RECEIVER_CHANNEL_0_MASK)
	//{
	//	if (isr_receiverLastPinValue[0] == 0)
	//	{
	//		// save current pin value
	//		isr_receiverLastPinValue[0] = 1;
	//
	//		// save the pulse start time
	//		isr_receiverPulseStartTime[0] = isr_currentClock; //currentTime;
	//	}
	//}
	//else
	//{
	//	// if pin value changed from 1 to 0
	//	if (isr_receiverLastPinValue[0] == 1)
	//	{
	//		// save current pin value
	//		isr_receiverLastPinValue[0] = 0;
	//
	//		// save the pulse duration
	//		receiverInput[0] = isr_currentClock > isr_receiverPulseStartTime[0]
	//			? (isr_currentClock - isr_receiverPulseStartTime[0]) / 2 // note: 1 timer tick = 0.5 microseconds
	//			: ((ICR5 - isr_receiverPulseStartTime[0]) + isr_currentClock) / 2; // in case counter overflow happens
	//																				 //currentTime - isr_receiverPulseStartTime[channel];
	//	}
	//}

	//unsigned long t = TCNT5;
	//Serial.print(t - isr_currentClock);
	//Serial.print("\n");
}

ISR(PCINT1_vect) 
{
	isr_currentTime = micros();

	updateReceiverChannelInput(PINJ, RECEIVER_CHANNEL_4_MASK, 4, isr_currentTime);
}

void calibrateGyro()
{
	_SETBIT(PORTB, PB4);
	//Serial.print("\nCalibrating gyro...");
	//Serial.print("\n");

	int sampleSize = 100;
	double tolerance = 0.1;
	double sum_x = 0, sum_y = 0, sum_z = 0;
	double error_x = 1000.0, error_y = 1000.0, error_z = 1000.0;
	double offset_x = 0, offset_y = 0, offset_z = 0;
	double best_error_x = 1000, best_error_y = 1000, best_error_z = 1000;

	while (abs(best_error_x) > tolerance || abs(best_error_y) > tolerance || abs(best_error_z) > tolerance)
	{

		sum_x = 0;
		sum_y = 0;
		sum_z = 0;
		// measure the mean offset
		for (int i = 0; i < sampleSize; i++)
		{
			MpuGetMeasurments(&mpu_rot_x, &mpu_rot_y, &mpu_rot_z, &mpu_accel_x, &mpu_accel_y, &mpu_accel_z, &mpu_temprature);
			sum_x += mpu_rot_x;
			sum_y += mpu_rot_y;
			sum_z += mpu_rot_z;
		}

		offset_x = sum_x / sampleSize;
		offset_y = sum_y / sampleSize;
		offset_z = sum_z / sampleSize;


		error_x = 0;
		error_y = 0;
		error_z = 0;
		// measure the error
		for (int i = 0; i < sampleSize * 0.2; i++)
		{
			MpuGetMeasurments(&mpu_rot_x, &mpu_rot_y, &mpu_rot_z, &mpu_accel_x, &mpu_accel_y, &mpu_accel_z, &mpu_temprature);
			error_x += (mpu_rot_x - offset_x);
			error_y += (mpu_rot_y - offset_y);
			error_z += (mpu_rot_z - offset_z);
		}

		if (abs(best_error_x) > abs(error_x))
		{
			mpu_gyroOffset_x = offset_x;
			best_error_x = error_x;
		}
		if (abs(best_error_y) > abs(error_y))
		{
			mpu_gyroOffset_y = offset_y;
			best_error_y = error_y;
		}
		if (abs(best_error_z) > abs(error_z))
		{
			mpu_gyroOffset_z = offset_z;
			best_error_z = error_z;
		}
	}

	//Serial.print("Calibration complete\n");
	//Serial.print("Offset X: ");
	//Serial.print(mpu_gyroOffset_x);
	//Serial.print("\t");
	//
	//Serial.print("Offset Y: ");
	//Serial.print(mpu_gyroOffset_y);
	//Serial.print("\t");
	//
	//Serial.print("Offset Z: ");
	//Serial.print(mpu_gyroOffset_z);
	//Serial.print("\t");
	//
	//Serial.print("\n");

	mpu_calibrated = true;

	_CLEARBIT(PORTB, PB4);
	
}

void printReceiverInput()
{
	Serial.print("Roll: ");

	//if (receiverInput[0] - 1480 < 0) Serial.print("<<<");
	//else if (receiverInput[0] - 1520 > 0) Serial.print(">>>");
	//else Serial.print("-+-");

	Serial.print("\t");
	Serial.print(receiverInput[0]);
	Serial.print("\t");

	///////////////////////////

	Serial.print("Pitch: ");

	//if (receiverInput[1] - 1480 < 0) Serial.print("^^^");
	//else if (receiverInput[1] - 1520 > 0) Serial.print("vvv");
	//else Serial.print("-+-");

	Serial.print("\t");
	Serial.print(receiverInput[1]);
	Serial.print("\t");

	///////////////////////////

	Serial.print("Throttle: ");

	//if (receiverInput[2] - 1480 < 0) Serial.print("^^^");
	//else if (receiverInput[2] - 1520 > 0) Serial.print("vvv");
	//else Serial.print("-+-");

	Serial.print("\t");
	Serial.print(receiverInput[2]);
	Serial.print("\t");

	///////////////////////////

	Serial.print("Yaw: ");

	//if (receiverInput[3] - 1480 < 0) Serial.print("<<<");
	//else if (receiverInput[3] - 1520 > 0) Serial.print(">>>");
	//else Serial.print("-+-");

	Serial.print("\t");
	Serial.print(receiverInput[3]);
	Serial.print("\t");

	///////////////////////////

	Serial.print("Aux1: ");


	Serial.print("\t");
	Serial.print(receiverInput[4]);
	Serial.print("\n");
}

void printMPUData()
{
	Serial.print("\nRotation X:\t");
	Serial.print(mpu_rot_x);
	Serial.print("\t");
	Serial.print(" Y:\t");
	Serial.print(mpu_rot_y);
	Serial.print("\t");
	Serial.print(" Z:\t");
	Serial.print(mpu_rot_z);
	Serial.print("\t");

	Serial.print(" Temprature: ");
	Serial.print(mpu_temprature);
	Serial.print("\t");

	Serial.print(" Acceleration X: ");
	Serial.print(mpu_accel_x);
	Serial.print("\t");
	Serial.print(" Y: ");
	Serial.print(mpu_accel_y);
	Serial.print("\t");
	Serial.print(" Z: ");
	Serial.print(mpu_accel_z);
	Serial.print("\n");
}

void printGyroInputs()
{
	Serial.print("Roll:\t");
	Serial.print(gyroInput.roll);
	Serial.print("\t");

	Serial.print("Pitch:\t");
	Serial.print(gyroInput.pitch);
	Serial.print("\t");

	Serial.print("Yaw:\t");
	Serial.print(gyroInput.yaw);
	Serial.print("\n");
}

void printESCOutputs() 
{
	Serial.print(escOutput[0]);
	Serial.print("\t");
	Serial.print(escOutput[1]);
	Serial.print("\t");
	Serial.print(escOutput[2]);
	Serial.print("\t");
	Serial.print(escOutput[3]);
	Serial.print("\n");
}

void printSetPoints() 
{
	Serial.print("Set points: Roll:\t");
	Serial.print(setPoint.roll);
	Serial.print("\t");

	Serial.print("Pitch:\t");
	Serial.print(setPoint.pitch);
	Serial.print("\t");

	Serial.print("Yaw:\t");
	Serial.print(setPoint.yaw);
	Serial.print("\n");
}

void printError()
{
	Serial.print("Error: Roll:\t");
	Serial.print(gyroInput.roll - setPoint.roll);
	Serial.print("\t");

	Serial.print("Pitch:\t");
	Serial.print(gyroInput.pitch - setPoint.pitch);
	Serial.print("\t");

	Serial.print("Yaw:\t");
	Serial.print(gyroInput.yaw - setPoint.yaw);
	Serial.print("\n");
}

void printControllerOutput()
{
	Serial.print("Output: Roll:\t");
	Serial.print(correction.roll);
	Serial.print("\t");

	Serial.print("Pitch:\t");
	Serial.print(correction.pitch);
	Serial.print("\t");

	Serial.print("Yaw:\t");
	Serial.print(correction.yaw);
	Serial.print("\n");
}

void printAttitude() 
{
	Serial.print("Pitch:\t");
	Serial.print(currentAttitude.pitch);
	Serial.print("\t");
	Serial.print("Roll\t");
	Serial.print(currentAttitude.roll);
	Serial.print("\n");


}
