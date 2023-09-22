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
#define BLU_SINGLE_MESSAGE_SIZE            100
#define BLU_SINGLE_REC_MESSAGE_SIZE        100
#define BLU_STATISTICS_PERIOD              5000
#define BLU_DATA_REPORTING_TIME            20
#define BLU_MINIMUM_MESS_BREAK_TIM         30

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

typedef struct
{
	float X[SIMULATOR_PROBES_COUNT];
	float Y[SIMULATOR_PROBES_COUNT];
	uint16_t T[SIMULATOR_PROBES_COUNT];
}Sim_PositionOnTruck_t;

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

static void (*ManualCtrlRequestCallBackPointer)(float vecV_X, float vecV_Y);

static LoggingState_t LoggingState = Suspended;
static InternalRobotState_t InternalRobotState = Standstill;

Sim_PositionOnTruck_t  SimFakeXY_MapDat;

static bool LogDroppedFlag =false;
static uint16_t RetransmissionCounter = 0U;
static uint16_t TransmisstedMessagesCounter = 0;

static uint16_t volatile UartBusyCounter =0u;

static BluRingBufferTransmit_t BluMainTransmitRingBuffer;
static uint8_t BluMainTransmitMessagesTab[BLU_TRANSMIT_RING_BUFFER_SIZE][BLU_SINGLE_MESSAGE_SIZE];

static BluRingBufferReceive_t BleMainReceiveRingBuffer;
static uint8_t BluMainReceiveMessagesTab[BLU_RECEIVE_RING_BUFFER_SIZE][BLU_SINGLE_REC_MESSAGE_SIZE];


static BLU_LfDataReport_t NewestLfDataReport = {0};


/*
 *********************************************************************************************
 * Static function prototypes section
 ********************************************************************************************
 */

static void Sim_Create_XY_FakeMap(void);
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

static BLU_CallStatus_t TransmitLfBaseDataReport(void)
{
	uint8_t DataBuffer[BLU_SINGLE_MESSAGE_SIZE] = {'B'};
	static uint8_t _SyncID = 0;

	BLU_CallStatus_t retval = BLU_Ok;

	NewestLfDataReport.SyncId = _SyncID;
	NewestLfDataReport.ucTimeStamp = HAL_GetTick();

	DataBuffer[0] = BLU_BaseDataReport;
	DataBuffer[1] = NewestLfDataReport.SyncId;

	memcpy(&DataBuffer[2], &NewestLfDataReport.ucTimeStamp,54);

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

static void Sim_Create_XY_FakeMap(void)
{
	static uint32_t ExtraShifter = 0u;

	for(int i=0;  i<100; i++)
	{
		SimFakeXY_MapDat.T[i]=    HAL_GetTick();
		SimFakeXY_MapDat.X[i]=  (float)i + (float)ExtraShifter;
		SimFakeXY_MapDat.Y[i] = 0.0F + (float)ExtraShifter;
	}
	for(int i=0;  i<100; i++)
	{
		SimFakeXY_MapDat.T[i+100]= HAL_GetTick();
		SimFakeXY_MapDat.X[i+100]= 100.0F + (float)ExtraShifter;
		SimFakeXY_MapDat.Y[i+100] = (float)i  + (float)ExtraShifter;
	}
	for(int i=0;  i<100; i++)
	{
		SimFakeXY_MapDat.T[i+200]= HAL_GetTick();
		SimFakeXY_MapDat.X[i+200]= 100.0F - ((float)i*1.0F) + (float)ExtraShifter;
		SimFakeXY_MapDat.Y[i+200] =100.0F     + (float)ExtraShifter;
	}

	ExtraShifter = ExtraShifter + 5;
}

void Sim_FakeBaseDataReportTask(void)
{
	static uint16_t ProbeIterator = 0u;
	static uint8_t SyncIdIter = 1;
	static float WaveHelper = 0.0F;

	for(int i =0; i<1; i++)
	{
		if(ProbeIterator == SIMULATOR_PROBES_COUNT)
		{
			Sim_Create_XY_FakeMap();
			ProbeIterator = 1U;
		}
		NewestLfDataReport.ucTimeStamp = HAL_GetTick();
		NewestLfDataReport.CurrMapData.PosX = SimFakeXY_MapDat.X[ProbeIterator];
		NewestLfDataReport.CurrMapData.PosY = SimFakeXY_MapDat.Y[ProbeIterator];
		NewestLfDataReport.CurrMapData.TravelledDistance = SyncIdIter;
		NewestLfDataReport.CurrMapData.PosO = M_PI * sin(2 * M_PI * WaveHelper);
		NewestLfDataReport.CurrMapData.WhLftSp = (2 * sin( 2 * M_PI * WaveHelper)) + (0.05 * sin( 2* M_PI * 3 * WaveHelper));
		NewestLfDataReport.CurrMapData.WhRhtSp = (2 * cos( 2 * M_PI * WaveHelper)) + (0.05 * sin( 2* M_PI * 7 * WaveHelper));
		NewestLfDataReport.CurrMapData.YawRate = (2 * sin( 2 * M_PI * WaveHelper)) + (0.3 * sin( 2* M_PI * 10 * WaveHelper));
		NewestLfDataReport.CurrSensorData.SensorData[0] = 40;
		NewestLfDataReport.CurrSensorData.SensorData[1] = 40;
		NewestLfDataReport.CurrSensorData.SensorData[2] = 40;
		NewestLfDataReport.CurrSensorData.SensorData[3] = 63;
		NewestLfDataReport.CurrSensorData.SensorData[4] = 64;
		NewestLfDataReport.CurrSensorData.SensorData[5] = (uint8_t)(65 + SyncIdIter);
		NewestLfDataReport.CurrSensorData.SensorData[6] = (uint8_t)(66 + SyncIdIter);
		NewestLfDataReport.CurrSensorData.SensorData[7] = 40;
		NewestLfDataReport.CurrSensorData.SensorData[8] = 40;
		NewestLfDataReport.CurrSensorData.SensorData[9] = 40;
		NewestLfDataReport.CurrSensorData.SensorData[10] = 40;
		NewestLfDataReport.CurrSensorData.SensorData[11] = 40;
		NewestLfDataReport.CurrSensorData.LastLeftLinePosConfidence = SyncIdIter;
		NewestLfDataReport.CurrSensorData.LastRightLinePosConfidence = SyncIdIter+5;
		NewestLfDataReport.CurrSensorData.PosError = 1 * sin( 2 * M_PI * WaveHelper);
		NewestLfDataReport.LinePidRegData.LinePidRegVal = (2 * sin( 2 * M_PI * WaveHelper)) + (0.2 * sin( 2* M_PI * 13 * WaveHelper));;

		WaveHelper = WaveHelper + 0.01;
		ProbeIterator++;
		SyncIdIter++;
	
	}

}


void BaseDataReporterTask(void)
{
	static uint32_t PreviousReportTransmitTime = 0;

	if( (HAL_GetTick() - PreviousReportTransmitTime >= BLU_DATA_REPORTING_TIME)
			&& (TrueDataLogging == LoggingState))
	{
		PreviousReportTransmitTime = HAL_GetTick();
		TransmitLfBaseDataReport(); 
		/*Base data report struct is updated asynchronicly 
		* by another modules
		* Check desprition of functions BLE_Report..
		*/
	}
	else if(  (HAL_GetTick() - PreviousReportTransmitTime >= BLU_DATA_REPORTING_TIME )
						&& (SimulatorDataLogging == LoggingState) )
	{
		PreviousReportTransmitTime = HAL_GetTick();
		Sim_FakeBaseDataReportTask();
		TransmitLfBaseDataReport();
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
				case BLU_NvM_ErrWeigthSensorData:
				{
					NvmDataUpdatedFlag= true;
					break;
				}

				case BLU_NvM_ErrWeigthSensorDataReq:
				{
					break;
				}

				case BLU_NvM_LinePidRegDataReq:
				{
					break;
				}

				case BLU_NvM_LinePidRegData:
				{
					NvmDataUpdatedFlag= true;
					break;
				}

				case BLU_NvM_VehCfgReq:
				{
	//				BLU_DbgMsgTransmit("VehCfgReq");
					break;
				}

				case BLU_NvM_VehCfgData:
				{
					NvmDataUpdatedFlag= true;
					// BLU_DbgMsgTransmit("Received BaseMotSpd: %f, LedSt %d, EndLMark %d",
					// 		BaseMotSpd,LedState,TryDetEndLine );

					break;
				}

				case BLU_RobotStart:
				{
					InternalRobotState = Driving;
					LoggingState = TrueDataLogging;
					BLU_DbgMsgTransmit("LineFollower start!");
					break;
				}
				case BLU_RobotStop:
				{
					InternalRobotState = Standstill;
					LoggingState = Suspended;
					BLU_DbgMsgTransmit("LineFollower stop");
					break;
				}

				case BLU_SimulatorStart:
				{
					LoggingState = SimulatorDataLogging;
					BLU_DbgMsgTransmit("Simulator data start");
					break;
				}

				case BLU_TrueBaseLoggingStart:
				{
					LoggingState = TrueDataLogging;
					BLU_DbgMsgTransmit("True data logger start");
					break;
				}

				case BLU_SimuAndTrueDataLoggingStop:
				{
					LoggingState = Suspended;
					BLU_DbgMsgTransmit("Logger stop");
					break;
				}

				case BLU_NvM_MotorsFactorsReq:
				{
					break;
				}

				case BLU_NvM_MotorsFactorsData:
				{
					NvmDataUpdatedFlag= true;
					break;

				}

				case BLU_NvM_EncoderModCfgReq:
				{
					break;
				}
				case BLU_NvM_EncoderModCfgData :
				{
					NvmDataUpdatedFlag= true;
					break;
				}

				case	BLU_NvM_ManualCntrlCommand:/* Virutal analog controller frame */
				{
					break;
				}

				case BLU_NvM_SpdProfileReq:
				{
					break;
				}

				case BLU_NvM_SpdProfileData:
				{
					NvmDataUpdatedFlag= true;
					break;
				}

				case BLU_SetNewRobotName:
				{
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

	// if(true == DevNameUpdateFlag)
	// {

	// 	char DevName[16];
	// 	char FakeRecBuf[20];
	// 	char DevNameFullCommandBuffor[16+7+2] = "AT+NAME";
	// 	uint8_t DevNameSize =7;

	// 	for(int i=0; i<16; i++)
	// 	{
	// 		if(DevName[i] == '\0'){
	// 			break;
	// 		}
	// 		DevNameSize++;
	// 	}

	// 	for(int i=7; i<(DevNameSize+7); i++){
	// 		DevNameFullCommandBuffor[i] = DevName[i-7];
	// 	}

	// 	HAL_UART_Transmit_DMA(&huart1, (uint8_t *)DevNameFullCommandBuffor, BLU_SINGLE_MESSAGE_SIZE);
	// 	HAL_UART_Receive(&huart1, (uint8_t *)FakeRecBuf, 20,100);
	// 	//	HAL_Delay(50); /*Short delay to ignore answer :) */
	// }

	// HAL_UART_Receive_DMA(&huart1, &BluMainReceiveMessagesTab[0][0], BLU_SINGLE_REC_MESSAGE_SIZE);
	HAL_UARTEx_ReceiveToIdle_DMA(&huart1, &BluMainReceiveMessagesTab[0][0], BLU_SINGLE_REC_MESSAGE_SIZE);
	Sim_Create_XY_FakeMap();

}

void BLU_Task(void)
{
	BaseDataReporterTask();
	ReceiveDataHandler();
	TransmitDataHandler();
}

void BLU_ReportMapData(BLU_MapDataReport_t *MapData)
{
	NewestLfDataReport.CurrMapData = *MapData;
}

void BLU_ReportSensorData(BLU_SensorDataReport_t *SensorData)
{
	NewestLfDataReport.CurrSensorData = *SensorData;
}

void BLU_RegisterNvMdataUpdateInfoCallBack(void UpdateInfoCb(void) )
{
	for(int i=0; i<BLU_NVM_UPDATE_MAX_CALL_BACKS_COUNT; i++)
	{
		if(NvmUpdateCallBacks[i] == 0)
		{
			NvmUpdateCallBacks[i] = UpdateInfoCb;
			break;
		}
	}
}

void BLU_RegisterManualCntrlRequestCallBack(void ManualCtrlReqCb(float vecV_X, float vecV_Y) )
{
	ManualCtrlRequestCallBackPointer = ManualCtrlReqCb;
}

bool BLU_isExpectedStateDriving(void)
{
	bool retVal = false;

	if(InternalRobotState ==  Driving)
	{
		retVal = true;
	}
	return retVal;
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













