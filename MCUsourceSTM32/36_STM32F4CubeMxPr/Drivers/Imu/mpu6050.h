/******************************************************************************************
 * @file mpu6050.h
 * @author teodor rosolowski, trteodor@gmail.com
 * 
 * @date 23.09.2023
 * @brief Source file contatning functionality related to IMU MPU6050 unit.
 * This file is distributed under MIT license
 *
 *******************************************************************************************/

#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_


/******************************************************************************************/
/*                                 INCLUDES                                               */
/******************************************************************************************/
#include <stdint.h>
#include "i2c.h"
#include "stdbool.h"

/******************************************************************************************/
/*                                 DEFINES                                                */
/******************************************************************************************/


#define IRQ_GPIO_LINE EXTI9_5_IRQn

#define MPU6050_ADDR 0xD0

/******************************************************************************************/
/*                                 DATA TYPES                                             */
/******************************************************************************************/

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
    float fildAccX;
    float fildAccY;
    float fildAccZ;
    float jerkX;
    float jerkY;
    float jerkZ;
    float checkDet;
    float roll;
    float pitch; 
    float yaw;
    float velX;
    float velY;
    float velZ;
    float posX;
    float posY;
    float posZ;
    bool flagUpdated;
}MpuData_t;


/******************************************************************************************/
/*                         PUBLIC FUNCTION PROTOTYPES                                     */
/******************************************************************************************/

uint8_t MPU6050_Init(I2C_HandleTypeDef *I2Cx);
uint8_t* MPU6050_Calibrate_Gyro(void);

void MPU6050Set_Calibrate_Gyro(uint8_t *data);
void MPU6050_Start_IRQ(void);
void MPU6050_Read_DMA(void);
void MPU6050_ReadDmaDataEndCallBack(MpuData_t *RecMpuData);
void MPU6050_DeviceReset(uint8_t Reset);



/******************************************************************************************/
/*                              END OF FILE                                               */
/******************************************************************************************/

#endif /* INC_MPU6050_H_ */