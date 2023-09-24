/******************************************************************************************
 * @file mpu6050.c
 * @author teodor rosolowski, trteodor@gmail.com
 * 
 * @date 23.09.2023
 * @brief Source file contatning functionality related to IMU MPU6050 unit.
 * This file is distributed under MIT license
 *
 *******************************************************************************************/


/******************************************************************************************/
/*                                 INCLUDES                                               */
/******************************************************************************************/
#include <math.h>
#include "mpu6050.h"
#include "mpu6050defs.h"


/******************************************************************************************/
/*                                 DEFINES                                                */
/******************************************************************************************/
#define I2C_TIMEOUT 1000

#define MPU_ERR_SAMPLING_COUNTER	10000// max: 65536

#define CONF_SAMPLE_FREQ 0.001F

#define acc     0
#define gyro    1
#define X       0
#define Y       1
#define Z       2

/******************************************************************************************/
/*                                 DATA TYPES                                             */
/******************************************************************************************/


/******************************************************************************************/
/*                                 MACROS                                                 */
/******************************************************************************************/


/******************************************************************************************/
/*                                 PRIVATE STATIC VARIABLES                               */
/******************************************************************************************/

const float mpuScale[] = {2048.34f, 131.072f}; // acc, gyro
uint8_t mpuRawDataBuffer[14];
float mpuDataScaled[2][3];

I2C_HandleTypeDef *i2c;

/******************************************************************************************/
/*                         PRIVATE FUNCTION PROTOTYPES                                    */
/******************************************************************************************/

static void scaleReceivedDataByDMA(void);

static void computeEulerAnglesMadgwickFilter(float *roll, float *pitch, float *yaw, const float invSampleFreq);

static float invSqrt(float x);

static void minus3dGravityVector(MpuData_t *RecMpuData);

static void getGyroscopeRAW(int16_t *gyroRaw);

/******************************************************************************************/
/*                              FUNCTION DEFINITION                                       */
/******************************************************************************************/

static void getGyroscopeRAW(int16_t *gyroRaw)
{
	uint8_t tmp[6];
	HAL_I2C_Mem_Read(i2c, MPU6050_ADDR, MPU6050_RA_GYRO_XOUT_H, 1, tmp, 6, I2C_TIMEOUT);

	gyroRaw[X] = (((int16_t)tmp[0]) << 8) | tmp[1];
	gyroRaw[Y] = (((int16_t)tmp[2]) << 8) | tmp[3];
	gyroRaw[Z] = (((int16_t)tmp[4]) << 8) | tmp[5];
}

static void scaleReceivedDataByDMA(void)
{
	for(uint8_t coordinat = 0; coordinat < 2; coordinat++)
	{
		for(uint8_t axis = 0; axis < 3; axis++)
		{
		mpuDataScaled[coordinat][axis] = ((float)((int16_t)((((int16_t)mpuRawDataBuffer[(coordinat*8)+(axis*2)]) << 8) | mpuRawDataBuffer[(coordinat*8)+(axis*2)+1])) / mpuScale[coordinat]);// - mpuErr[coordinat][axis];
		}
	}	
}

static float invSqrt(float x) {
  unsigned int i = 0x5F1F1412 - (*(unsigned int*)&x >> 1);
  float tmp = *(float*)&i;
  float y = tmp * (1.69000231f - 0.714158168f * x * tmp * tmp);
  return y;
}

static void computeEulerAnglesMadgwickFilter(float *roll, float *pitch, float *yaw, const float invSampleFreq)//float gx, float gy, float gz, float ax, float ay, float az, float invSampleFreq, float* roll_IMU, float* pitch_IMU, float* yaw_IMU) {
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

static void minus3dGravityVector(MpuData_t *RecMpuData)
{
double alpha=  RecMpuData->yaw;    ;// some aribitar values for testing //yaw
double beta=   RecMpuData->pitch    ;// some aribitar values for testing //pitch
double theta = RecMpuData->roll ;// some aribitar values for testing //roll

double accel[3]=   {RecMpuData->accX,    RecMpuData->accY,      RecMpuData->accZ};
double gravity[3]= {0.0,    0.0,      1.0};// always vertically downwards at g = 1.0
double rG[3];
// double rA[3];
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

// rA[0]= accel[0]*R[0][0]   + accel[1]*R[0][1]   + accel[2]*R[0][2] ;
// rA[1]= accel[0]*R[1][0]   + accel[1]*R[1][1]   + accel[2]*R[1][2] ;
// rA[2]= accel[0]*R[2][0]   + accel[1]*R[2][1]   + accel[2]*R[2][2] ;

mA[0]= accel[0]-rG[0];
mA[1]= rG[1] + accel[1];
mA[2]= accel[2]-rG[2];

RecMpuData->normalizedAccX = mA[0];
RecMpuData->normalizedAccY = mA[1];
RecMpuData->normalizedAccZ = mA[2];

}

static float lowPassAlfaFiltering(float FilteredState, float newRawVal,float alfa)
{
    return(newRawVal * alfa + FilteredState * (1.0F-alfa) );
}

static void computeObjectVelocity(MpuData_t *RecMpuData)
{
    static uint32_t timerTest1 = 0;
    static bool oneTimeFlag = 0;

    RecMpuData->jerkX = RecMpuData->fildAccX - RecMpuData->normalizedAccX;
    RecMpuData->jerkY = RecMpuData->fildAccY - RecMpuData->normalizedAccY;
    RecMpuData->jerkZ = RecMpuData->fildAccZ - RecMpuData->normalizedAccZ;

    RecMpuData->velX = RecMpuData->velX + (RecMpuData->jerkX * (9.81F) * CONF_SAMPLE_FREQ);
    RecMpuData->velY = RecMpuData->velY + (RecMpuData->jerkY * (9.81F) * CONF_SAMPLE_FREQ);
    RecMpuData->velZ = RecMpuData->velZ + (RecMpuData->jerkZ * (9.81F) * CONF_SAMPLE_FREQ);

    RecMpuData->posX = RecMpuData->posX + (RecMpuData->velX * CONF_SAMPLE_FREQ);
    RecMpuData->posY = RecMpuData->posY + (RecMpuData->velY * CONF_SAMPLE_FREQ);
    RecMpuData->posZ = RecMpuData->posZ + (RecMpuData->velZ * CONF_SAMPLE_FREQ);

    RecMpuData->fildAccX = lowPassAlfaFiltering(RecMpuData->fildAccX, RecMpuData->normalizedAccX, 0.01);
    RecMpuData->fildAccY = lowPassAlfaFiltering(RecMpuData->fildAccY, RecMpuData->normalizedAccY, 0.01);
    RecMpuData->fildAccZ = lowPassAlfaFiltering(RecMpuData->fildAccZ, RecMpuData->normalizedAccZ, 0.01);


    if(HAL_GetTick() - timerTest1 > 5000 && oneTimeFlag == 0)
    {
        oneTimeFlag = 1;

        RecMpuData->velX = 0;
        RecMpuData->velY = 0;
        RecMpuData->velZ = 0;
        RecMpuData->posX = 0;
        RecMpuData->posY = 0;
        RecMpuData->posZ = 0;
    }

}

/******************************************************************************************/
/*                         PUBLIC FUNCTION DECLARATIONS                                   */
/******************************************************************************************/
extern void MPU6050_ReadDmaDataEndCallBack(MpuData_t *RecMpuData)
{
	scaleReceivedDataByDMA();
	computeEulerAnglesMadgwickFilter(&RecMpuData->roll, &RecMpuData->pitch, &RecMpuData->yaw, CONF_SAMPLE_FREQ);
    minus3dGravityVector(RecMpuData);
    computeObjectVelocity(RecMpuData);

	RecMpuData->gyroX = mpuDataScaled[gyro][X];
	RecMpuData->gyroY = mpuDataScaled[gyro][Y];
	RecMpuData->gyroZ = mpuDataScaled[gyro][Z];
	RecMpuData->accX  = mpuDataScaled[acc][X];
	RecMpuData->accY  = mpuDataScaled[acc][Y];
	RecMpuData->accZ  = mpuDataScaled[acc][Z];

	RecMpuData->flagUpdated = true;
}

extern void MPU6050_DeviceReset(uint8_t Reset)
{
	uint8_t tmp;
	HAL_I2C_Mem_Read(i2c, MPU6050_ADDR, MPU6050_RA_PWR_MGMT_1, 1, &tmp, 1, I2C_TIMEOUT);
	tmp &= ~(1<<MPU6050_PWR1_DEVICE_RESET_BIT);
	tmp |= ((Reset & 0x1) << MPU6050_PWR1_DEVICE_RESET_BIT);
	HAL_I2C_Mem_Write(i2c, MPU6050_ADDR, MPU6050_RA_PWR_MGMT_1, 1, &tmp, 1, I2C_TIMEOUT);
}

extern uint8_t MPU6050_Init(I2C_HandleTypeDef *I2Cx)
{
    uint8_t check = 0;
    uint8_t Data;

	i2c = I2Cx;
    // check device ID WHO_AM_I

    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, MPU6050_RA_WHO_AM_I, 1, &check, 1, I2C_TIMEOUT);

    if (check == 104) // 0x68 will be returned by the sensor if everything goes well
    {
        // power management register 0X6B we should write all 0's to wake the sensor up
        Data = 0;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, MPU6050_RA_PWR_MGMT_1, 1, &Data, 1, I2C_TIMEOUT);

        // Set DATA RATE of 0x07 - 1KH  || 0x04 - 200 z by writing SMPLRT_DIV register
        Data = 0x07;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, MPU6050_RA_SMPLRT_DIV, 1, &Data, 1, I2C_TIMEOUT);

        // Set accelerometer configuration in ACCEL_CONFIG Register
        // XA_ST=0,YA_ST=0,ZA_ST=0, FS_SEL=0 -> � 2g
        uint8_t tmp;
        HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, MPU6050_RA_ACCEL_CONFIG, 1, &tmp, 1, I2C_TIMEOUT);
        tmp &= 0xE7;
        tmp |= ((0x3 & 0x7) << 3);
        //Data = 0x00;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, MPU6050_RA_ACCEL_CONFIG, 1, &tmp, 1, I2C_TIMEOUT);

        // Set Gyroscopic configuration in GYRO_CONFIG Register
        // XG_ST=0,YG_ST=0,ZG_ST=0, FS_SEL=0 -> � 250 �/s
        Data = 0x00;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, MPU6050_RA_GYRO_CONFIG, 1, &Data, 1, I2C_TIMEOUT);


        tmp = 0x30;
        HAL_I2C_Mem_Write(i2c, MPU6050_ADDR, MPU6050_RA_INT_PIN_CFG, 1, &tmp, 1, I2C_TIMEOUT);

        return 0;
    }
    return 1;
}

extern void MPU6050Set_Calibrate_Gyro(uint8_t *data)
{
	HAL_I2C_Mem_Write(i2c, MPU6050_ADDR, MPU6050_RA_XG_OFFS_USRH , 1, data, 6, I2C_TIMEOUT);
}

extern uint8_t* MPU6050_Calibrate_Gyro(void)
{
	static uint8_t data[6] = {0, 0, 0, 0, 0, 0};
	int32_t gyroBias[3] = {0, 0, 0};
	int16_t gyroRaw[3];

	for(uint16_t c = 0; c < MPU_ERR_SAMPLING_COUNTER; c++)
	{
		getGyroscopeRAW(gyroRaw);
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

extern void MPU6050_Start_IRQ(void) //Enable Int
{
	uint8_t tmp = 0x01;
	HAL_I2C_Mem_Write(i2c, MPU6050_ADDR, MPU6050_RA_INT_ENABLE, 1, &tmp, 1, I2C_TIMEOUT);

	HAL_NVIC_SetPriority(IRQ_GPIO_LINE, 0, 0);
  	HAL_NVIC_EnableIRQ(IRQ_GPIO_LINE);
}

extern void MPU6050_Read_DMA(void)
{
	HAL_I2C_Mem_Read_DMA(i2c, MPU6050_ADDR, MPU6050_RA_ACCEL_XOUT_H, 1, mpuRawDataBuffer, 14);
}

