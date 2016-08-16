
#include <arduino.h>
#include <Wire.h>

#include "MPU6050.h"

int MPU6050_Test_I2C()
{
	byte data = 0x00;

	data = MpuRegRead(MPU_REG_WHO_AM_I);
	
	if (data == MPU6050_ADDRESS)
	{
		//Serial.print("\nI2C Read Test Passed, MPU6050 Address: ");
		//Serial.print(data);
		return 0;
	}
	else
	{
		//Serial.print("\nERROR: I2C Read Test failed. Actual data: ");
		//Serial.print(data);
		return 1;
	}
}

void MpuTestRegisters()
{
	byte sampleRate = MpuRegRead(MPU_REG_SMPLRT_DIV);
	bool error = false;

	if (sampleRate != MPU_SAMPLE_RATE_DIV)
	{
		//Serial.print("\nERROR: MPU sample rate register has incorrect value: ");
		//Serial.print(sampleRate);
	}

	//if (!error)
	//	Serial.print("\nRegister tests passed");
}

int MpuInit()
{
	// set sampling rate
	MpuRegWrite(MPU_REG_SMPLRT_DIV, MPU_SAMPLE_RATE_DIV);
	
	// FSYNC and digital low pass filter settings
	MpuRegWrite(MPU_REG_CONFIG,(MPU_EXT_SYNC | MPU_DLP_BW) );
	
	// set gyroscope full scale range
	MpuRegWrite(MPU_REG_GYRO_CONFIG, MPU_GYRO_FS_RANGE);
	
	// set accelerometer full scale range
	MpuRegWrite(MPU_REG_ACCEL_CONFIG, MPU_ACC_FS_RANGE);
	
	// MPU control functions
	// set interrupt clear option, FSYNC logic level, aux bus access
	MpuRegWrite(MPU_REG_INT_PIN_CFG,(
		MPU_INT_STAT_CLEAR | MPU_FSYNC_LOGIC_LEVEL | MPU_I2C_BYPASS));
	
	// enable interrupts
	MpuRegWrite(MPU_REG_INT_ENABLE, MPU_INT_ENABLE);
	
	// configure MPU hardware FIFO
	MpuRegWrite(MPU_REG_USER_CTRL, 0x40);
	
	// reset the analog and digital signal paths of all on-chip sensors
	MpuRegWrite(MPU_REG_SIGNAL_PATH_RESET, 0x07);
	
	// CLKSEL is a 3-bit unsigned value specifying
	// the clock source of the device
	MpuRegWrite(MPU_REG_PWR_MGMT_1, MPU_CLK_SEL);
	
	//Serial.print("\nMPU6050 Setup Complete");	

	return 0;
}

byte MpuRegRead(byte regAddress)
{
	byte data = 0;

	MpuRegRead(regAddress, &data, 1);

	return data;
}

void MpuRegRead(byte regAddress, byte* data, size_t length)
{
	Wire.beginTransmission(MPU6050_ADDRESS);
	Wire.write(regAddress);
	Wire.endTransmission();
	Wire.requestFrom(MPU6050_ADDRESS, length);

	while (Wire.available() < length);

	for (int i = 0; i < length; i++)
	{
		data[i] = Wire.read();
	}
}

void MpuRegWrite(byte regAddress, byte data)
{
	Wire.beginTransmission(MPU6050_ADDRESS);
	Wire.write(regAddress);
	Wire.write(data);
	Wire.endTransmission();
}

void MpuGetMeasurments(
	double* gyroX, double* gyroY, double* gyroZ, 
	double* accelX, double*accelY, double* accelZ, 
	double* temprature)
{
	byte data[14] = { 0 };
	long temp;

	MpuRegRead(MPU_REG_ACCEL_XOUT_H, data, 14);
	
	// calculate acceleration values
	temp = ((data[0] << 8) | data[1]);
	*accelX = (double)temp * MPU_ACC_SCALE_FACTOR;
	temp = ((data[2] << 8) | data[3]);
	*accelY = (double)temp * MPU_ACC_SCALE_FACTOR;
	temp = ((data[4] << 8) | data[5]);
	*accelZ = (double)temp * MPU_ACC_SCALE_FACTOR;

	temp = ((data[6] << 8) | data[7]);
	//calculate temperature in °C
	*temprature = ((double)temp / 340.0) + 36.53;

	// calculate axis rotation values in °/sec
	temp = ((data[8] << 8) | data[9]);
	*gyroX = (double)temp * MPU_GYRO_SCALE_FACTOR;
	temp = ((data[10] << 8) | data[11]);
	*gyroY = (double)temp * MPU_GYRO_SCALE_FACTOR;
	temp = ((data[12] << 8) | data[13]);
	*gyroZ = (double)temp * MPU_GYRO_SCALE_FACTOR;
}