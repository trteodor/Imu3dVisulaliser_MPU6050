/**
 * @file BluetoothClassicComm.h
 * @author teodor

 * @brief This file is a part of Line follower application
 * This is Bluetooth module to communicate wireless with Line Follower
 * I used module HC-06 - if you are interested about information of the modules please find data sheets in internet.
 */





#ifndef _BLE_ServiceModule_H
#define _BLE_ServiceModule_H

#include "stdint.h"
#include "stdbool.h"

#include "stdio.h"
#include "stdint.h"

#include "stdbool.h"
#include <stdarg.h>
#include <string.h>


/*
 * Defines configuration start
 * */

#define BLU_TRANSMIT_RING_BUFFER_SIZE     100
#define BLU_RECEIVE_RING_BUFFER_SIZE     100



/*
 * Defines configuration end
 * */

/*
 * Type defs
 * */
typedef enum
{
	BLU_Ok,
	BLU_Error
}BLU_CallStatus_t;

/*
 * Type definition of Common Header or message ID for Embedded software and desktop application
 * */
typedef enum
{
    BLU_None = 0,
    BLU_ConfirmationTag,
	BLU_CommunicationStats,
    BLU_DebugMessage,

	BLU_LoggingStart,
	BLU_LoggingStop,

	BLU_ImuData,
}BLU_MessageID_t;

/////////////////////////////////////////////////////////////////////////////////
typedef struct
{
	uint8_t SyncId;
	uint32_t ucTimeStamp;
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
    float roll;
    float pitch; 
    float yaw;
    float velX;
    float velY;
    float velZ;
    float posX;
    float posY;
    float posZ;
}__attribute__((__packed__)) BLU_ImuBaseDataReport_t; 


/*
 *
 * Exported functions prototypes:
 * */

void BLU_Init(void); /*Initialize the Communication module*/
void BLU_Task(void); /*Runnable of BLE communication module (Call as often as possible*/
					/*.. don't blocked contest, very short*/


void BLU_ReportImuData(BLU_ImuBaseDataReport_t *ImuData);


/* brief BLU_DbgMsgTransmit
* A simple function to send debug message through BLE (to QT Application)
* String length is limited to ~100chars
* Input format is the same as in "printf" function.. 
* To keep good performence i suggest don't sent many debug messages..
*/
void BLU_DbgMsgTransmit(char *DbgString, ...);

// #define BLU_LOG(str, ...)  BLU_DbgMsgTransmit((char *) str, ##__VA_ARGS__)

#endif //_BLE_ServiceModule_H
