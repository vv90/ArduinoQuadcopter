

#define MPU6050_ADDRESS 0x68 // MPU6050 address

#define MPU_REG_SMPLRT_DIV 0x19
#define MPU_REG_CONFIG 0x1A
#define MPU_REG_GYRO_CONFIG 0x1B
#define MPU_REG_ACCEL_CONFIG 0x1C
#define MPU_REG_INT_PIN_CFG 0x37
#define MPU_REG_INT_ENABLE 0x38
#define MPU_REG_USER_CTRL 0x6A
#define MPU_REG_SIGNAL_PATH_RESET 0x68
#define MPU_REG_PWR_MGMT_1 0x6B
#define MPU_REG_ACCEL_XOUT_H 0x3B
#define MPU_REG_GYRO_XOUT_H 0x43
#define MPU_REG_TEMP_OUT_H 0x41
#define MPU_REG_WHO_AM_I 0x75

// MPU6050 configuration parameters

// sample rate divider, to set the sample rate of the sensor
#define MPU_SAMPLE_RATE_DIV 0x07 // to generate the desired Sample Rate for MPU

// external FSYNC pin sampling
#define MPU_EXT_SYNC 0

// digital low pass filter bandwidth
#define MPU_DLP_BW 0

// gyroscope full scale range
#define MPU_GYRO_FS_RANGE 0x8 // full scale range = ± 500 °/s

// accelerometer full scale range
#define MPU_ACC_FS_RANGE 0x18 // full Scale Range = ± 16g

// interrupt status bit clear by read operation
#define MPU_INT_STAT_CLEAR 0x10 // enable

// set FSYNC pin active logic level
#define MPU_FSYNC_LOGIC_LEVEL 0x80 // active low

// set aux I2C bus access for host
#define MPU_I2C_BYPASS 0x20 // enable

// enable interrupts
#define MPU_INT_ENABLE 0x59 // enabled interrupts: motion detection,
// FIFO overflow,
// I2C master,
// data ready
// clock selection
#define MPU_CLK_SEL 0 // internal 8MHz oscillator

// gyroscope scaling factor. This depends on MPU_GYRO_FS_RANGE
#define MPU_GYRO_SCALE_FACTOR 0.0152587890625

// accelerometer scaling factor. This depends on MPU_ACC_FS_RANGE
#define MPU_ACC_SCALE_FACTOR 0.00048828125

int MpuInit();
byte MpuRegRead(byte regAddress);
void MpuRegRead(byte regAddress, byte *data, size_t length);
void MpuRegWrite(byte regAddress, byte data);
int MPU6050_Test_I2C();
void MpuTestRegisters();
void MpuGetMeasurments(
	double* gyroX, double* gyroY, double* gyroZ,
	double* accelX, double*accelY, double* accelZ,
	double* temprature);