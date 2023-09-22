/*
 * mpu6050.h
 *
 *  Created on: Nov 13, 2019
 *      Author: Bulanov Konstantin
 */

#ifndef INC_GY521_H_
#define INC_GY521_H_

#endif /* INC_GY521_H_ */

#include <stdint.h>
#include "i2c.h"
#include "stdbool.h"

#define IRQ_GPIO_LINE EXTI9_5_IRQn

typedef struct
{
    float gyroX;
    float gyroY;
    float gyroZ;
    float accX;
    float accY;
    float accZ;
    float normalizedAccX;
    float normalizedAccY;
    float normalizedAccZ;
    float checkDet;
    float roll;
    float pitch; 
    float yaw;
    float posX;
    float posY;
    bool flagUpdated;
}MpuData_t;

// MPU6050 structure
typedef struct
{

    int16_t Accel_X_RAW;
    int16_t Accel_Y_RAW;
    int16_t Accel_Z_RAW;
    double Ax;
    double Ay;
    double Az;

    int16_t Gyro_X_RAW;
    int16_t Gyro_Y_RAW;
    int16_t Gyro_Z_RAW;
    double Gx;
    double Gy;
    double Gz;

    float Temperature;

    double KalmanAngleX;
    double KalmanAngleY;
} MPU6050_t;

// Kalman structure
typedef struct
{
    double Q_angle;
    double Q_bias;
    double R_measure;
    double angle;
    double bias;
    double P[2][2];
} Kalman_t;

uint8_t MPU6050_Init(I2C_HandleTypeDef *I2Cx);
uint8_t* MPU6050_Calibrate_Gyro(void);
void MPU6050Set_Calibrate_Gyro(uint8_t *data);
void MPU6050_Start_IRQ(void);
void MPU6050_Read_DMA(void);
void MPU6050_ReadDmaDataEndCallBack(MpuData_t *RecMpuData);

void minusGravity(MpuData_t *RecMpuData);

void MPU6050_DeviceReset(uint8_t Reset);


void MPU6050_Read_Accel(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct);

void MPU6050_Read_Gyro(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct);

void MPU6050_Read_Temp(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct);

void MPU6050_Read_All(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct);

double Kalman_getAngle(Kalman_t *Kalman, double newAngle, double newRate, double dt);
