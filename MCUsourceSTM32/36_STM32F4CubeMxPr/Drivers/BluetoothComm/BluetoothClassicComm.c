/**
 * @file BluetoothClassicComm.c
 * @author teodor

 * @brief This file is a part of Line follower application
 * This is Bluetooth module to communicate wireless with Line Follower
 * I used module HC-06 - if you are interested about information of the modules please find data sheets in internet.
 */


#include "BluetoothClassicComm.h"

#include "main.h"

#include "stdbool.h"
#include "string.h"
#include <stdio.h>
#include <stdlib.h>
#include "dma.h" /*DMA driver part of STM32 HAL Library*/
#include "usart.h"  /*usart driver part of STM32 HAL Library*/
#include "stddef.h"
#include "math.h"

#define SIMULATOR_PROBES_COUNT             300
#define BLU_SINGLE_MESSAGE_SIZE            120
#define BLU_SINGLE_REC_MESSAGE_SIZE        120
#define BLU_STATISTICS_PERIOD              5000
#define BLU_DATA_REPORTING_TIME            20
#define BLU_MINIMUM_MESS_BREAK_TIM         10

/**!
 * \brief BluRingBufferStatus_t
 * \details ---
 * */
typedef enum BluRingBufferStatus_t
{
	RB_OK       = 0,
	RB_ERROR	= 1
} BluRingBufferStatus_t;

/**!
 * \brief BluRingBufferTransmit_t
 * \details ---
 * */
typedef struct
{
	uint16_t Head; // Pointer to write
	uint16_t Tail; // Pointer to read
	uint8_t  MessageSize[BLU_TRANSMIT_RING_BUFFER_SIZE]; // Array to store messages size
} BluRingBufferTransmit_t;

/**!
 * \brief BluRingBufferReceive_t
 * \details ---
 * */
typedef struct
{

	uint16_t Head; // Pointer to write
	uint16_t Tail; // Pointer to read
	uint8_t  MessageSize[BLU_RECEIVE_RING_BUFFER_SIZE]; // Array to store messages size
	bool ReadyToRead[BLU_RECEIVE_RING_BUFFER_SIZE];
} BluRingBufferReceive_t;


typedef enum InternalRobotState_t
{
	Standstill,
	Driving,
}InternalRobotState_t;

typedef enum LoggingState_t
{
	Suspended,
	TrueDataLogging,
	SimulatorDataLogging,
}LoggingState_t;

/**
 * *******************************************************************************************
 * Static variables
 * *******************************************************************************************
 * */

#define BLU_NVM_UPDATE_MAX_CALL_BACKS_COUNT 15

static void (*NvmUpdateCallBacks[BLU_NVM_UPDATE_MAX_CALL_BACKS_COUNT])(void) = {0};


static LoggingState_t LoggingState = Suspended;

static bool LogDroppedFlag =false;
static uint16_t RetransmissionCounter = 0U;
static uint16_t TransmisstedMessagesCounter = 0;

static uint16_t volatile UartBusyCounter =0u;

static BluRingBufferTransmit_t BluMainTransmitRingBuffer;
static uint8_t BluMainTransmitMessagesTab[BLU_TRANSMIT_RING_BUFFER_SIZE][BLU_SINGLE_MESSAGE_SIZE];

static BluRingBufferReceive_t BleMainReceiveRingBuffer;
static uint8_t BluMainReceiveMessagesTab[BLU_RECEIVE_RING_BUFFER_SIZE][BLU_SINGLE_REC_MESSAGE_SIZE];


static BLU_ImuBaseDataReport_t NewestImuDataReport = {0};


/*
 *********************************************************************************************
 * Static function prototypes section
 ********************************************************************************************
 */

static BluRingBufferStatus_t RB_Transmit_Read(BluRingBufferTransmit_t *Buf, uint8_t *MessageSize, uint8_t **MessagePointer);
static BluRingBufferStatus_t RB_Transmit_Write(BluRingBufferTransmit_t *Buf,uint8_t *DataToWrite, uint8_t MessageSize);
static BluRingBufferStatus_t RB_Receive_GetNextMessageAddress(BluRingBufferReceive_t *Buf, uint8_t **WriteAddress);
static BluRingBufferStatus_t RB_Receive_Read(BluRingBufferReceive_t *Buf, uint8_t *MessageSize, uint8_t **MessagePointer);


/************************************************************************************************/
//  void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
//  	if(huart->Instance == USART2)
//  	{
//  		uint8_t *MessageReceiveBufferAddress;

//  		/*Start listen again as fast as possible*/
//  		RB_Receive_GetNextMessageAddress(&BleMainReceiveRingBuffer,&MessageReceiveBufferAddress);
//  		// Start listening again
//  		// HAL_UARTEx_ReceiveToIdle_DMA(&huart1, MessageReceiveBufferAddress, BLU_SINGLE_REC_MESSAGE_SIZE);
//  		HAL_UART_Receive_DMA(&huart1, MessageReceiveBufferAddress, BLU_SINGLE_REC_MESSAGE_SIZE);
//  		// HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
//  	}
//  }

static bool Received50BytesAndHitIdleFlag;

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	static uint8_t *MessageReceiveBufferAddress;

 	if(huart->Instance == USART1)
 	{
		if( (Size == BLU_SINGLE_REC_MESSAGE_SIZE) || (true == Received50BytesAndHitIdleFlag) )
		{
			Received50BytesAndHitIdleFlag = false; /*Clear the flag..*/
			RB_Receive_GetNextMessageAddress(&BleMainReceiveRingBuffer,&MessageReceiveBufferAddress);
			HAL_UARTEx_ReceiveToIdle_DMA(&huart1, MessageReceiveBufferAddress, BLU_SINGLE_REC_MESSAGE_SIZE);
		}
		else if(Size == (BLU_SINGLE_REC_MESSAGE_SIZE/2))
		{
			Received50BytesAndHitIdleFlag=true;
			HAL_UARTEx_ReceiveToIdle_DMA(&huart1, &MessageReceiveBufferAddress[BLU_SINGLE_REC_MESSAGE_SIZE/2], BLU_SINGLE_REC_MESSAGE_SIZE/2);
		}else{
			RB_Receive_GetNextMessageAddress(&BleMainReceiveRingBuffer,&MessageReceiveBufferAddress);
			HAL_UARTEx_ReceiveToIdle_DMA(&huart1, MessageReceiveBufferAddress, BLU_SINGLE_REC_MESSAGE_SIZE);
		}



 	}
}


/*!
 ************************************************************************************************
 * \brief RB_Transmit_Read Function to read data from ring buffer
 * \details --
 * \param RingBuffer_t *Buf - pointer to Ring Buffer structure
 * \param out MessageSize - size of the "BleLogData" (return value)
 * \param out MessagePointer - pointer to the message stored in RingBuffer (return value)
 *
 * */
static BluRingBufferStatus_t RB_Transmit_Read(BluRingBufferTransmit_t *Buf, uint8_t *MessageSize, uint8_t **MessagePointer)
{
	// Check if Tail hit Head
	if(Buf->Head == Buf->Tail)
	{
		// If yes - there is nothing to read
		return RB_ERROR;
	}

	// Write current value from buffer to pointer from argument
	*MessageSize = Buf->MessageSize[Buf->Tail];
	*MessagePointer = &BluMainTransmitMessagesTab[Buf->Tail][0];

	// Calculate new Tail pointer
	Buf->Tail = (Buf->Tail + 1) % BLU_TRANSMIT_RING_BUFFER_SIZE;

	// Everything is ok - return OK status
	return RB_OK;
}


/*!
 ************************************************************************************************
 * \brief RB_Transmit_Write
 * \details --
 * \param in RingBuffer_t *Buf - pointer to Ring Buffer structure
 * \param in BleLogData - pointer to the data stored in RingBuffer
 * \param in MessageSize - size of the "BleLogData"
 ************************************************************************************************/
static BluRingBufferStatus_t RB_Transmit_Write(BluRingBufferTransmit_t *Buf,uint8_t *DataToWrite, uint8_t MessageSize)
{
	uint16_t ActualWriteIndex;

	// Calculate new Head pointer value
	uint16_t HeadTmp = (Buf->Head + 1) % BLU_TRANSMIT_RING_BUFFER_SIZE;

	// Check if there is one free space ahead the Head buffer
	if(HeadTmp == Buf->Tail)
	{
		// There is no space in the buffer - return an error
		return RB_ERROR;
	}

	ActualWriteIndex = Buf->Head;
	Buf->Head = HeadTmp;
	// Store a value into the buffer
	Buf->MessageSize[ActualWriteIndex] = MessageSize;

	/*Copy the values to new buffer*/
	for(int i=0; i<MessageSize; i++)
	{
		BluMainTransmitMessagesTab[ActualWriteIndex][i] = DataToWrite[i];
	}
	// Everything is ok - return OK status
	return RB_OK;
}

/*!
 ************************************************************************************************
 * \brief RB_Receive_GetNextMessageAddress
 * \details Function used to work with DMA - direct write to ring buffer by DMA
 * \param in WriteIndex - Current Write index in Ring Buffer where the 
 *                          value should be transmitted for example by DMA
 ************************************************************************************************/
static BluRingBufferStatus_t RB_Receive_GetNextMessageAddress(BluRingBufferReceive_t *Buf, uint8_t **WriteAddress)
{
	static uint8_t DefaultBlindBuffer[20];



	/*Mark previous message as ready to read*/
	Buf->ReadyToRead[Buf->Head] = true;

	// Calculate new Head pointer value
	uint16_t HeadTmp = (Buf->Head + 1) % BLU_RECEIVE_RING_BUFFER_SIZE;

	// Check if there is one free space ahead the Head buffer
	if(HeadTmp == Buf->Tail)
	{
		/*Even if buffer is full data must be received somewhere to don't crush application/ dma*/
		*WriteAddress = DefaultBlindBuffer;
		// There is no space in the buffer - return an error
		return RB_ERROR;
	}

	Buf->ReadyToRead[HeadTmp] = false;
	Buf->MessageSize[HeadTmp] = BLU_SINGLE_REC_MESSAGE_SIZE;
	Buf->Head = HeadTmp;

	*WriteAddress = &BluMainReceiveMessagesTab[HeadTmp][0];

	// Everything is ok - return OK status
	return RB_OK;
}


/*!
 ************************************************************************************************
 * \brief RB_Transmit_Read Function to read data from ring buffer
 * \details --
 * \param RingBuffer_t *Buf - pointer to Ring Buffer structure
 * \param out MessageSize - size of the "BleLogData" (return value)
 * \param out MessagePointer - pointer to the message stored in RingBuffer (return value)
 *
 * */
static BluRingBufferStatus_t RB_Receive_Read(BluRingBufferReceive_t *Buf, uint8_t *MessageSize, uint8_t **MessagePointer)
{

	if(Buf->ReadyToRead[Buf->Tail] == false)
	{
		/*Any message in ring buffer isn't ready to read*/
		return RB_ERROR;
	}
	/*Mark again as not ready to read*/
	Buf->ReadyToRead[Buf->Tail] = false;

	// Check if Tail hit Head
	if(Buf->Head == Buf->Tail)
	{
		// If yes - there is nothing to read
		return RB_ERROR;
	}



	// Write current value from buffer to pointer from argument
	*MessageSize = Buf->MessageSize[Buf->Tail];
	*MessagePointer = &BluMainReceiveMessagesTab[Buf->Tail][0];

	// Calculate new Tail pointer
	Buf->Tail = (Buf->Tail + 1) % BLU_RECEIVE_RING_BUFFER_SIZE;

	// Everything is ok - return OK status
	return RB_OK;
}

static BLU_CallStatus_t TransmitImuBaseDataReport(void)
{
	uint8_t DataBuffer[BLU_SINGLE_MESSAGE_SIZE] = {'B'};
	static uint8_t _SyncID = 0;

	BLU_CallStatus_t retval = BLU_Ok;

	NewestImuDataReport.SyncId = _SyncID;
	NewestImuDataReport.ucTimeStamp = HAL_GetTick();

	DataBuffer[0] = BLU_ImuData;
	DataBuffer[1] = NewestImuDataReport.SyncId;

	memcpy(&DataBuffer[2], &NewestImuDataReport.ucTimeStamp,sizeof(BLU_ImuBaseDataReport_t) -2 );

	if(RB_Transmit_Write(&BluMainTransmitRingBuffer, (uint8_t *)DataBuffer, BLU_SINGLE_MESSAGE_SIZE) != RB_OK)
	{
		LogDroppedFlag = true;
		retval = BLU_Error;
	}

	_SyncID++;
	return retval;
}


static void Statistics_CreateAndTransmitCommunicationStatistics(void)
{
	static uint8_t SyncIdStat = 0U;

	static uint8_t RingBufferStatisticsHolder[BLU_SINGLE_MESSAGE_SIZE] = {0};

	static uint16_t MainRingBufferOverFlowCounter = 0U;
	uint16_t RingBufferRemaninzingSpace = 0U;

	for(int i =0; i<BLU_SINGLE_MESSAGE_SIZE; i++)
	{
		RingBufferStatisticsHolder[i] = 0U; /*Sanitize the buffer*/
	}

	if(true == LogDroppedFlag)
	{
		LogDroppedFlag = false;
		MainRingBufferOverFlowCounter++;
	}

	RingBufferStatisticsHolder[0] = BLU_CommunicationStats;
	RingBufferStatisticsHolder[1] = SyncIdStat; /*SyncId*/

	uint32_t ucHelperTime = HAL_GetTick();
	RingBufferStatisticsHolder[2] = ((uint8_t *)&ucHelperTime)[0];
	RingBufferStatisticsHolder[3] = ((uint8_t *)&ucHelperTime)[1];
	RingBufferStatisticsHolder[4] = ((uint8_t *)&ucHelperTime)[2];
	RingBufferStatisticsHolder[5] = ((uint8_t *)&ucHelperTime)[3];

	RingBufferRemaninzingSpace = BLU_TRANSMIT_RING_BUFFER_SIZE - (BluMainTransmitRingBuffer.Head - BluMainTransmitRingBuffer.Tail);

	RingBufferStatisticsHolder[6] = ((uint8_t *)&RingBufferRemaninzingSpace)[0];
	RingBufferStatisticsHolder[7] = ((uint8_t *)&RingBufferRemaninzingSpace)[1];

	RingBufferStatisticsHolder[8] = ((uint8_t *)&MainRingBufferOverFlowCounter)[0];
	RingBufferStatisticsHolder[9] = ((uint8_t *)&MainRingBufferOverFlowCounter)[1];

	RingBufferStatisticsHolder[10] =  ((uint8_t *)&TransmisstedMessagesCounter)[0];
	RingBufferStatisticsHolder[11] =  ((uint8_t *)&TransmisstedMessagesCounter)[1];

	RingBufferStatisticsHolder[12]  =  ((uint8_t *)&RetransmissionCounter)[0];
	RingBufferStatisticsHolder[13] =  ((uint8_t *)&RetransmissionCounter)[1];

	HAL_UART_Transmit_DMA(&huart1, RingBufferStatisticsHolder, BLU_SINGLE_MESSAGE_SIZE);

	SyncIdStat++;
}


/*
 ****************************************************************************************************
 * exported functions declarations section START
 * exported functions description is added in header file
 *****************************************************************************************************
 */

void BaseDataReporterTask(void)
{
	static uint32_t PreviousReportTransmitTime = 0;

	if( (HAL_GetTick() - PreviousReportTransmitTime >= BLU_DATA_REPORTING_TIME)
			&& (TrueDataLogging == LoggingState))
	{
		PreviousReportTransmitTime = HAL_GetTick();
		TransmitImuBaseDataReport(); 
	}
	else if(  (HAL_GetTick() - PreviousReportTransmitTime >= BLU_DATA_REPORTING_TIME )
						&& (SimulatorDataLogging == LoggingState) )
	{
		PreviousReportTransmitTime = HAL_GetTick();
		//Sim_FakeBaseDataReportTask();
		TransmitImuBaseDataReport();
	}
	else//Suspended == LoggingState
	{
		/*Nothing to do*/
	}
}

static void ReceiveDataHandler(void)
{
	static uint32_t LastReadRingBufferTime = 0U;
	static bool NvmDataUpdatedFlag = false;

	uint8_t *MessageToRead_p = NULL;
	uint8_t MessageToReadSize= 0U;

	{

		if(RB_Receive_Read(&BleMainReceiveRingBuffer, &MessageToReadSize,&MessageToRead_p) == RB_OK)
		{

			uint8_t *ReceivedMessageBuff = MessageToRead_p;
			BLU_MessageID_t ReceivedMessageId = ReceivedMessageBuff[0];

			LastReadRingBufferTime = HAL_GetTick();

			switch(ReceivedMessageId)
			{
				case BLU_LoggingStart:
				{
					LoggingState = TrueDataLogging;
					break;
				}
				case BLU_LoggingStop:
				{
					LoggingState = Suspended;
					break;
				}

				default:
				{
					if(ReceivedMessageId != 0)
					{
						BLU_DbgMsgTransmit("BLU RecHandlrErr-why defult?? |FrId:%d",ReceivedMessageId);
					}
					break;
				}

			}

			ReceivedMessageId = BLU_None;
		}

		if( (NvmDataUpdatedFlag == true) && (HAL_GetTick() - LastReadRingBufferTime > 50) )
		{
			NvmDataUpdatedFlag = false;

			for(int i=0; i<BLU_NVM_UPDATE_MAX_CALL_BACKS_COUNT; i++)
			{
				if(NvmUpdateCallBacks[i] != 0)
				{
					NvmUpdateCallBacks[i]();
				}
			}

			BLU_DbgMsgTransmit("Nvm Data updated and commited");
		}

	}


}



static void TransmitDataHandler(void)
{
	static uint32_t BLU_StatisticTimer = 0;
	static uint32_t PreviousDataTransmitTime = 0;

	if( HAL_UART_STATE_READY == huart1.gState)
	{
		if(HAL_GetTick() - BLU_StatisticTimer >= BLU_STATISTICS_PERIOD)
		{
				BLU_StatisticTimer = HAL_GetTick();
				// /*Transmit the statistics with higher prio than normal messages*/
				TransmisstedMessagesCounter++;
				Statistics_CreateAndTransmitCommunicationStatistics();
		}
		else
		{
			if( (HAL_GetTick() - PreviousDataTransmitTime) > BLU_MINIMUM_MESS_BREAK_TIM)
			{
				PreviousDataTransmitTime = HAL_GetTick();

				uint8_t *MessageToTransmit_p = NULL;
				uint8_t MessageToSize_p = 0U;
				if(RB_Transmit_Read(&BluMainTransmitRingBuffer, &MessageToSize_p, &MessageToTransmit_p) == RB_OK)
				{
					TransmisstedMessagesCounter++;
					HAL_UART_Transmit_DMA(&huart1, MessageToTransmit_p, BLU_SINGLE_MESSAGE_SIZE);
				}
			}
		}
	}
	else
	{
		UartBusyCounter++;
	}
}


/*
*********************************************************************************************
*Interface functions implementation
*********************************************************************************************
*/
#define API_FUNCTIONS_BluetoothClassicComm

void BLU_Init(void)
{
	// HAL_UART1_CostumUserInit(9600); /*Configure default HC-06 uart speed for few operations*/
	// HAL_Delay(200); /* While startup after power on
	// 	* and after experimental test it was observed that short delay is required to stabilize bluetooth module
	// 	* It is required for AT Commands like AT+NAME...
	// 	* */

	// HAL_UART_Transmit(&huart1, (uint8_t *)"AT+BAUDA",8,1000);
	// HAL_Delay(500);
	//HAL_UART1_CostumUserInit(460800); /*We are almost sure that bluetooth
										//* BaudRate is now configured as expected*/
	// HAL_Delay(500);

	// HAL_UART_Receive_DMA(&huart1, &BluMainReceiveMessagesTab[0][0], BLU_SINGLE_REC_MESSAGE_SIZE);
	HAL_UARTEx_ReceiveToIdle_DMA(&huart1, &BluMainReceiveMessagesTab[0][0], BLU_SINGLE_REC_MESSAGE_SIZE);
}

void BLU_Task(void)
{
	BaseDataReporterTask();
	ReceiveDataHandler();
	TransmitDataHandler();
}



void BLU_ReportImuData(BLU_ImuBaseDataReport_t *ImuData)
{
	NewestImuDataReport = *ImuData;
}


void BLU_DbgMsgTransmit(char *DbgString, ...)
{
	static uint8_t SyncId = 0;
	uint32_t ucTimeStamp = HAL_GetTick();
	char DebugMessageBufferHelper[255] = {'B'};

	va_list ap;
	va_start(ap, DbgString);
	uint16_t Size = vsprintf(&DebugMessageBufferHelper[6],DbgString,ap);
	va_end(ap);

	Size = Size + 6;
	DebugMessageBufferHelper[0] = BLU_DebugMessage;
	DebugMessageBufferHelper[1] = SyncId;
	memcpy(&DebugMessageBufferHelper[2],(uint8_t *)&ucTimeStamp,4);
	

	if(RB_Transmit_Write(&BluMainTransmitRingBuffer, (uint8_t *)DebugMessageBufferHelper, BLU_SINGLE_MESSAGE_SIZE) != RB_OK)
	{
		LogDroppedFlag = true;
	}

	SyncId++;;
}













