#include <math.h>
#include "mpu6050.h"
#include "mpu6050defs.h"

#define RAD_TO_DEG 57.295779513082320876798154814105

#define WHO_AM_I_REG 0x75
#define PWR_MGMT_1_REG 0x6B
#define SMPLRT_DIV_REG 0x19
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG 0x41
#define GYRO_CONFIG_REG 0x1B
#define GYRO_XOUT_H_REG 0x43

// Setup MPU6050
#define MPU6050_ADDR 0xD0
const uint16_t i2c_timeout = 100;
const double Accel_Z_corrector = 14418.0;


#define I2C_TIMEOUT 1000
#define MPU_ERR_SAMPLING_COUNTER	10000// max: 65536
#define CONF_SAMPLE_FREQ 0.001F
const float mpuScale[] = {2048.34f, 131.072f}; // acc, gyro
#define acc     0
#define gyro    1
#define X       0
#define Y       1
#define Z       2

uint8_t mpu_buffer[14];
float mpuDataScaled[2][3];
I2C_HandleTypeDef *i2c;


uint32_t timer;

Kalman_t KalmanX = {
    .Q_angle = 0.001f,
    .Q_bias = 0.003f,
    .R_measure = 0.03f};

Kalman_t KalmanY = {
    .Q_angle = 0.001f,
    .Q_bias = 0.003f,
    .R_measure = 0.03f,
};

void MPU6050Set_Calibrate_Gyro(uint8_t *data)
{
	HAL_I2C_Mem_Write(i2c, MPU6050_ADDR, MPU6050_RA_XG_OFFS_USRH , 1, data, 6, I2C_TIMEOUT);
}

void GetGyroscopeRAW(int16_t *gyroRaw)//int16_t *x, int16_t *y, int16_t *z)
{
	uint8_t tmp[6];
	HAL_I2C_Mem_Read(i2c, MPU6050_ADDR, MPU6050_RA_GYRO_XOUT_H, 1, tmp, 6, I2C_TIMEOUT);

	gyroRaw[X] = (((int16_t)tmp[0]) << 8) | tmp[1];
	gyroRaw[Y] = (((int16_t)tmp[2]) << 8) | tmp[3];
	gyroRaw[Z] = (((int16_t)tmp[4]) << 8) | tmp[5];
}

uint8_t* MPU6050_Calibrate_Gyro(void)
{
	static uint8_t data[6] = {0, 0, 0, 0, 0, 0};
	int32_t gyroBias[3] = {0, 0, 0};
	int16_t gyroRaw[3];

	for(uint16_t c = 0; c < MPU_ERR_SAMPLING_COUNTER; c++)
	{
		GetGyroscopeRAW(gyroRaw);
		gyroBias[X] += gyroRaw[X];
		gyroBias[Y] += gyroRaw[Y];
		gyroBias[Z] += gyroRaw[Z];
		HAL_Delay(5);
	}
	
	gyroBias[X] /= MPU_ERR_SAMPLING_COUNTER;
	gyroBias[Y] /= MPU_ERR_SAMPLING_COUNTER;
	gyroBias[Z] /= MPU_ERR_SAMPLING_COUNTER;

	data[0] = (-gyroBias[X]/4  >> 8) & 0xFF;
	data[1] = (-gyroBias[X]/4)       & 0xFF;
	data[2] = (-gyroBias[Y]/4  >> 8) & 0xFF;
	data[3] = (-gyroBias[Y]/4)       & 0xFF;
	data[4] = (-gyroBias[Z]/4  >> 8) & 0xFF;
	data[5] = (-gyroBias[Z]/4)       & 0xFF;

	MPU6050Set_Calibrate_Gyro(data);

	return data;
}

void MPU6050_DeviceReset(uint8_t Reset)
{
	uint8_t tmp;
	HAL_I2C_Mem_Read(i2c, MPU6050_ADDR, MPU6050_RA_PWR_MGMT_1, 1, &tmp, 1, I2C_TIMEOUT);
	tmp &= ~(1<<MPU6050_PWR1_DEVICE_RESET_BIT);
	tmp |= ((Reset & 0x1) << MPU6050_PWR1_DEVICE_RESET_BIT);
	HAL_I2C_Mem_Write(i2c, MPU6050_ADDR, MPU6050_RA_PWR_MGMT_1, 1, &tmp, 1, I2C_TIMEOUT);
}

uint8_t MPU6050_Init(I2C_HandleTypeDef *I2Cx)
{
    uint8_t check = 0;
    uint8_t Data;

	i2c = I2Cx;
    // check device ID WHO_AM_I

    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, WHO_AM_I_REG, 1, &check, 1, i2c_timeout);

    if (check == 104) // 0x68 will be returned by the sensor if everything goes well
    {
        // power management register 0X6B we should write all 0's to wake the sensor up
        Data = 0;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &Data, 1, i2c_timeout);

        // Set DATA RATE of 0x07 - 1KH  || 0x04 - 200 z by writing SMPLRT_DIV register
        Data = 0x07;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &Data, 1, i2c_timeout);

        // Set accelerometer configuration in ACCEL_CONFIG Register
        // XA_ST=0,YA_ST=0,ZA_ST=0, FS_SEL=0 -> � 2g
        uint8_t tmp;
        HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &tmp, 1, i2c_timeout);
        tmp &= 0xE7;
        tmp |= ((0x3 & 0x7) << 3);
        //Data = 0x00;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &tmp, 1, i2c_timeout);

        // Set Gyroscopic configuration in GYRO_CONFIG Register
        // XG_ST=0,YG_ST=0,ZG_ST=0, FS_SEL=0 -> � 250 �/s
        Data = 0x00;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &Data, 1, i2c_timeout);


        tmp = 0x30;
        HAL_I2C_Mem_Write(i2c, MPU6050_ADDR, MPU6050_RA_INT_PIN_CFG, 1, &tmp, 1, I2C_TIMEOUT);


        // uint8_t gyroCalibValues[6] = {0, 23, 0, 63, 0, 27};
        // HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, 0x13 , 1, gyroCalibValues, 6, i2c_timeout);

        return 0;
    }
    return 1;
}

void MPU6050_Start_IRQ(void) //Enable Int
{
	uint8_t tmp = 0x01;
	HAL_I2C_Mem_Write(i2c, MPU6050_ADDR, MPU6050_RA_INT_ENABLE, 1, &tmp, 1, I2C_TIMEOUT);

	HAL_NVIC_SetPriority(IRQ_GPIO_LINE, 0, 0);
  	HAL_NVIC_EnableIRQ(IRQ_GPIO_LINE);
}

void MPU6050_Read_DMA(void)
{
	HAL_I2C_Mem_Read_DMA(i2c, MPU6050_ADDR, MPU6050_RA_ACCEL_XOUT_H, 1, mpu_buffer, 14);
}

void MPU6050_Read_Accel(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct)
{
    uint8_t Rec_Data[6];

    // Read 6 BYTES of data starting from ACCEL_XOUT_H register

    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 6, i2c_timeout);

    DataStruct->Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
    DataStruct->Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);

    /*** convert the RAW values into acceleration in 'g'
         we have to divide according to the Full scale value set in FS_SEL
         I have configured FS_SEL = 0. So I am dividing by 16384.0
         for more details check ACCEL_CONFIG Register              ****/

    DataStruct->Ax = DataStruct->Accel_X_RAW *0.0004882;
    DataStruct->Ay = DataStruct->Accel_Y_RAW *0.0004882;
    DataStruct->Az = DataStruct->Accel_Z_RAW *0.0004882;
}

void MPU6050_Read_Gyro(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct)
{
    uint8_t Rec_Data[6];

    // Read 6 BYTES of data starting from GYRO_XOUT_H register

    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, GYRO_XOUT_H_REG, 1, Rec_Data, 6, i2c_timeout);

    DataStruct->Gyro_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Gyro_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
    DataStruct->Gyro_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);

    /*** convert the RAW values into dps (�/s)
         we have to divide according to the Full scale value set in FS_SEL
         I have configured FS_SEL = 0. So I am dividing by 131.0
         for more details check GYRO_CONFIG Register              ****/

    DataStruct->Gx = DataStruct->Gyro_X_RAW / 131.0;
    DataStruct->Gy = DataStruct->Gyro_Y_RAW / 131.0;
    DataStruct->Gz = DataStruct->Gyro_Z_RAW / 131.0;
}

void MPU6050_Read_Temp(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct)
{
    uint8_t Rec_Data[2];
    int16_t temp;

    // Read 2 BYTES of data starting from TEMP_OUT_H_REG register

    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, TEMP_OUT_H_REG, 1, Rec_Data, 2, i2c_timeout);

    temp = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Temperature = (float)((int16_t)temp / (float)340.0 + (float)36.53);
}

void MPU6050_Read_All(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct)
{
    uint8_t Rec_Data[14];
    int16_t temp;

    // Read 14 BYTES of data starting from ACCEL_XOUT_H register

    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, 0x3B, 1, Rec_Data, 14, i2c_timeout);

    DataStruct->Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
    DataStruct->Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);
    temp = (int16_t)(Rec_Data[6] << 8 | Rec_Data[7]);
    DataStruct->Gyro_X_RAW = (int16_t)(Rec_Data[8] << 8 | Rec_Data[9]);
    DataStruct->Gyro_Y_RAW = (int16_t)(Rec_Data[10] << 8 | Rec_Data[11]);
    DataStruct->Gyro_Z_RAW = (int16_t)(Rec_Data[12] << 8 | Rec_Data[13]);

    DataStruct->Ax = DataStruct->Accel_X_RAW *0.0004882;
    DataStruct->Ay = DataStruct->Accel_Y_RAW *0.0004882;
    DataStruct->Az = DataStruct->Accel_Z_RAW *0.0004882;
    DataStruct->Temperature = (float)((int16_t)temp / (float)340.0 + (float)36.53);
    DataStruct->Gx = DataStruct->Gyro_X_RAW / 131.0;
    DataStruct->Gy = DataStruct->Gyro_Y_RAW / 131.0;
    DataStruct->Gz = DataStruct->Gyro_Z_RAW / 131.0;

    // Kalman angle solve
    double dt = (double)(HAL_GetTick() - timer) / 1000;
    timer = HAL_GetTick();
    double roll;
    double roll_sqrt = sqrt(
        DataStruct->Accel_X_RAW * DataStruct->Accel_X_RAW + DataStruct->Accel_Z_RAW * DataStruct->Accel_Z_RAW);
    if (roll_sqrt != 0.0)
    {
        roll = atan(DataStruct->Accel_Y_RAW / roll_sqrt) * RAD_TO_DEG;
    }
    else
    {
        roll = 0.0;
    }
    double pitch = atan2(-DataStruct->Accel_X_RAW, DataStruct->Accel_Z_RAW) * RAD_TO_DEG;
    if ((pitch < -90 && DataStruct->KalmanAngleY > 90) || (pitch > 90 && DataStruct->KalmanAngleY < -90))
    {
        KalmanY.angle = pitch;
        DataStruct->KalmanAngleY = pitch;
    }
    else
    {
        DataStruct->KalmanAngleY = Kalman_getAngle(&KalmanY, pitch, DataStruct->Gy, dt);
    }
    if (fabs(DataStruct->KalmanAngleY) > 90)
        DataStruct->Gx = -DataStruct->Gx;
    DataStruct->KalmanAngleX = Kalman_getAngle(&KalmanX, roll, DataStruct->Gx, dt);
}

double Kalman_getAngle(Kalman_t *Kalman, double newAngle, double newRate, double dt)
{
    double rate = newRate - Kalman->bias;
    Kalman->angle += dt * rate;

    Kalman->P[0][0] += dt * (dt * Kalman->P[1][1] - Kalman->P[0][1] - Kalman->P[1][0] + Kalman->Q_angle);
    Kalman->P[0][1] -= dt * Kalman->P[1][1];
    Kalman->P[1][0] -= dt * Kalman->P[1][1];
    Kalman->P[1][1] += Kalman->Q_bias * dt;

    double S = Kalman->P[0][0] + Kalman->R_measure;
    double K[2];
    K[0] = Kalman->P[0][0] / S;
    K[1] = Kalman->P[1][0] / S;

    double y = newAngle - Kalman->angle;
    Kalman->angle += K[0] * y;
    Kalman->bias += K[1] * y;

    double P00_temp = Kalman->P[0][0];
    double P01_temp = Kalman->P[0][1];

    Kalman->P[0][0] -= K[0] * P00_temp;
    Kalman->P[0][1] -= K[0] * P01_temp;
    Kalman->P[1][0] -= K[1] * P00_temp;
    Kalman->P[1][1] -= K[1] * P01_temp;

    return Kalman->angle;
};


/*!
 ************************************************************************************************
 * \brief ScaleReceivedData_DMA
 * \details Function used to scale received data
 * \param 
 * \param out 
 * \param out 
 *
 * */
void ScaleReceivedData_DMA()
{
	for(uint8_t coordinat = 0; coordinat < 2; coordinat++)
	{
		for(uint8_t axis = 0; axis < 3; axis++)
		{
		mpuDataScaled[coordinat][axis] = ((float)((int16_t)((((int16_t)mpu_buffer[(coordinat*8)+(axis*2)]) << 8) | mpu_buffer[(coordinat*8)+(axis*2)+1])) / mpuScale[coordinat]);// - mpuErr[coordinat][axis];
		}
	}	
}


//Fast inverse sqrt for madgwick filter
static float invSqrt(float x) {
  unsigned int i = 0x5F1F1412 - (*(unsigned int*)&x >> 1);
  float tmp = *(float*)&i;
  float y = tmp * (1.69000231f - 0.714158168f * x * tmp * tmp);
  return y;
}

/*!
 ************************************************************************************************
 * \brief GetEulerAngles_MadgwickFilter
 * \details 
 * \param in
 * \param out
 * \param out
 * \param out
 *
 * */
static void GetEulerAngles_MadgwickFilter(float *roll, float *pitch, float *yaw, const float invSampleFreq)//float gx, float gy, float gz, float ax, float ay, float az, float invSampleFreq, float* roll_IMU, float* pitch_IMU, float* yaw_IMU) {
{
	static float q0 = 1.0f; //initialize quaternion for madgwick filter
	static float q1 = 0.0f;
	static float q2 = 0.0f;
	static float q3 = 0.0f;
	static float B_madgwick = 0.04;  //Madgwick filter parameter

	//DESCRIPTION: Attitude estimation through sensor fusion - 6DOF
	/*
	* See description of Madgwick() for more information. This is a 6DOF implimentation for when magnetometer data is not
	* available (for example when using the recommended MPU6050 IMU for the default setup).
	*/
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

	//Convert gyroscope degrees/sec to radians/sec
	mpuDataScaled[gyro][X] *= 0.0174533f;
	mpuDataScaled[gyro][Y] *= 0.0174533f;
	mpuDataScaled[gyro][Z] *= 0.0174533f;

	//Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-q1 * mpuDataScaled[gyro][X] - q2 * mpuDataScaled[gyro][Y] - q3 * mpuDataScaled[gyro][Z]);
	qDot2 = 0.5f * (q0 * mpuDataScaled[gyro][X] + q2 * mpuDataScaled[gyro][Z] - q3 * mpuDataScaled[gyro][Y]);
	qDot3 = 0.5f * (q0 * mpuDataScaled[gyro][Y] - q1 * mpuDataScaled[gyro][Z] + q3 * mpuDataScaled[gyro][X]);
	qDot4 = 0.5f * (q0 * mpuDataScaled[gyro][Z] + q1 * mpuDataScaled[gyro][Y] - q2 * mpuDataScaled[gyro][X]);

	//Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((mpuDataScaled[acc][X] == 0.0f) && (mpuDataScaled[acc][Y] == 0.0f) && (mpuDataScaled[acc][Z] == 0.0f))) {
		//Normalise accelerometer measurement
		recipNorm = invSqrt(mpuDataScaled[acc][X] * mpuDataScaled[acc][X] + mpuDataScaled[acc][Y] * mpuDataScaled[acc][Y] + mpuDataScaled[acc][Z] * mpuDataScaled[acc][Z]);
		mpuDataScaled[acc][X] *= recipNorm;
		mpuDataScaled[acc][Y] *= recipNorm;
		mpuDataScaled[acc][Z] *= recipNorm;

		//Auxiliary variables to avoid repeated arithmetic
		_2q0 = 2.0f * q0;
		_2q1 = 2.0f * q1;
		_2q2 = 2.0f * q2;
		_2q3 = 2.0f * q3;
		_4q0 = 4.0f * q0;
		_4q1 = 4.0f * q1;
		_4q2 = 4.0f * q2;
		_8q1 = 8.0f * q1;
		_8q2 = 8.0f * q2;
		q0q0 = q0 * q0;
		q1q1 = q1 * q1;
		q2q2 = q2 * q2;
		q3q3 = q3 * q3;

		//Gradient decent algorithm corrective step
		s0 = _4q0 * q2q2 + _2q2 * mpuDataScaled[acc][X] + _4q0 * q1q1 - _2q1 * mpuDataScaled[acc][Y];
		s1 = _4q1 * q3q3 - _2q3 * mpuDataScaled[acc][X] + 4.0f * q0q0 * q1 - _2q0 * mpuDataScaled[acc][Y] - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * mpuDataScaled[acc][Z];
		s2 = 4.0f * q0q0 * q2 + _2q0 * mpuDataScaled[acc][X] + _4q2 * q3q3 - _2q3 * mpuDataScaled[acc][Y] - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * mpuDataScaled[acc][Z];
		s3 = 4.0f * q1q1 * q3 - _2q1 * mpuDataScaled[acc][X] + 4.0f * q2q2 * q3 - _2q2 * mpuDataScaled[acc][Y];
		recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); //normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		//Apply feedback step
		qDot1 -= B_madgwick * s0;
		qDot2 -= B_madgwick * s1;
		qDot3 -= B_madgwick * s2;
		qDot4 -= B_madgwick * s3;
	}

	//Integrate rate of change of quaternion to yield quaternion
	q0 += qDot1 * invSampleFreq;
	q1 += qDot2 * invSampleFreq;
	q2 += qDot3 * invSampleFreq;
	q3 += qDot4 * invSampleFreq;

	//Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;

	//compute angles
	*roll = atan2(q0*q1 + q2*q3, 0.5f - q1*q1 - q2*q2);
	*pitch = -asin(-2.0f * (q1*q3 - q0*q2));
	*yaw = -atan2(q1*q2 + q0*q3, 0.5f - q2*q2 - q3*q3);
}

void minusGravity(MpuData_t *RecMpuData)
{
float inYawRad = RecMpuData->yaw;
float inPitchRad = RecMpuData->pitch; 
float inRollRad = RecMpuData->roll; 

float x = RecMpuData->accX;
float y = RecMpuData->accY;
float z = RecMpuData->accZ;
double alpha=  inYawRad    ;// some aribitar values for testing //pitch
double beta=   inPitchRad    ;// some aribitar values for testing //yaw
double theta = inRollRad ;// some aribitar values for testing //roll

double accel[3]=   {x,    y,      z};// data will come from the accelerometers
double gravity[3]= {0.0,    0.0,      1.0};// always vertically downwards at g = 1.0
double rG[3],rA[3];
double mA[3];

double R[3][3] =
{
  { cos(alpha)*cos(beta) , cos(alpha)*sin(beta)*sin(theta) - sin(alpha)*cos(theta) , cos(alpha)*sin(beta)*cos(theta) + sin(alpha)*sin(theta)},
  { sin(alpha)*cos(beta) , sin(alpha)*sin(beta)*sin(theta) + cos(alpha)*cos(theta) , sin(alpha)*sin(beta)*cos(theta) - cos(alpha)*sin(theta)},
  {     -1* sin(beta)    ,                  cos(beta) * sin(theta)                 ,               cos(beta) * cos(theta)                   }
};

rG[0]= gravity[0]*R[0][0] + gravity[1]*R[0][1] + gravity[2]*R[0][2] ;
rG[1]= gravity[0]*R[1][0] + gravity[1]*R[1][1] + gravity[2]*R[1][2] ;
rG[2]= gravity[0]*R[2][0] + gravity[1]*R[2][1] + gravity[2]*R[2][2] ;

rA[0]= accel[0]*R[0][0]   + accel[1]*R[0][1]   + accel[2]*R[0][2] ;
rA[1]= accel[0]*R[1][0]   + accel[1]*R[1][1]   + accel[2]*R[1][2] ;
rA[2]= accel[0]*R[2][0]   + accel[1]*R[2][1]   + accel[2]*R[2][2] ;

mA[0]= accel[0]-rG[0];
mA[1]= rG[1] + accel[1];
mA[2]= accel[2]-rG[2];

RecMpuData->normalizedAccX = mA[0];
RecMpuData->normalizedAccY = mA[1];
RecMpuData->normalizedAccZ = mA[2];

}

void MPU6050_ReadDmaDataEndCallBack(MpuData_t *RecMpuData)
{
	ScaleReceivedData_DMA();
	// Madgwick Orientation Filter with 6 degrees of freedom
	GetEulerAngles_MadgwickFilter(&RecMpuData->roll, &RecMpuData->pitch, &RecMpuData->yaw, CONF_SAMPLE_FREQ);
    // minusGravity(RecMpuData);

	RecMpuData->gyroX = mpuDataScaled[gyro][X];
	RecMpuData->gyroY = mpuDataScaled[gyro][Y];
	RecMpuData->gyroZ = mpuDataScaled[gyro][Z];
	RecMpuData->accX  = mpuDataScaled[acc][X];
	RecMpuData->accY  = mpuDataScaled[acc][Y];
	RecMpuData->accZ  = mpuDataScaled[acc][Z];

	RecMpuData->flagUpdated = true;
}