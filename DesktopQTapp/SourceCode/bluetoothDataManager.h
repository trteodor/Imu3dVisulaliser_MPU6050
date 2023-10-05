#ifndef BLUETOOTHDATAMANAGER_H
#define BLUETOOTHDATAMANAGER_H

#include <QMainWindow>
#include "bluetoothclassic.h"
#include "qmutex.h"
#include "qthread.h"




class BluDataManager : public QObject
{
    Q_OBJECT
public:
    BluDataManager();
    ~BluDataManager();

    QThread BluDatMngr_Thread;

    bluetoothClassic bleutoothClassicConnection; /*Must be moved to BluDatMngr_Thread*/

    volatile bool DebugTable_BaseDataLoggingState = false;


    /*Extra variables for synchronization beetween plotting and incomming data
    * To avoid any delays
    */
    QMutex PlottingInfoMutex;
    volatile bool MapPlotPlottingState = false;
    volatile bool rawAccPlotPlottingState = false;
    volatile bool eulerAgPlotPlottingState = false;
    volatile bool fildAccPlotPlottingState = false;
    volatile bool accJerkPlotPlottingState = false;
    volatile bool gyroPlotPlottingState = false;
    volatile bool normAccPlotPlottingState = false;
    volatile bool velPlotPlottingState = false;

    QMutex DebugTableScrollingBottonMutex;
    volatile bool DebugTableScrollingBottomIsActivState = false;


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
    /*
 * Modules should report the newest data then BLE module will transmit it
 * */

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
    }BLU_ImuBaseDataReport_t;

    typedef struct
    {
        uint8_t SyncId;
        uint32_t ucTimeStamp;
        uint16_t RingBufferRemainingSize;
        uint16_t RingBufferOverFlowCounter;
        uint16_t TransmisstedMessagesCounter;
        uint16_t RetransmissionCounter;
    }BLU_StatisticData_t ;


    BLU_ImuBaseDataReport_t ImuDataRep;

signals:


    void BluDatMngrSignal_DebugTable_InsertDataRow(uint32_t ucTimeStamp, uint32_t FrameCounter, uint8_t SyncId, QString DecodedDataString,QColor RowColor = QColor( 255,255,255) );
    void BluDatMngrSignal_DebugTable_ScrollToBottom();


    void BluDatMngrSignal_CommunicationStatisticsUpdate(uint32_t ucTimeStamp,uint16_t RingBufferRemainingSize,uint16_t RingBufferOverFlowCounter,
                                                            uint16_t TransmisstedMessagesCounter,uint16_t RetransmissionCounter);

    void BluDatMngrSignal_Update3DOrientation(float yaw,float pitch,float roll);
//    void BluDatMngrSignal_UpdateOrientation(float Orientation);

//    void BluDatMngrSignal_PlotMapUpdate(void);
    void BluDatMngrSignal_PlotRawAccUpdate(void);
//    void BluDatMngrSignal_PlotEulerAgAUpdate(void);
//    void BluDatMngrSignal_PlotFildAccUpdate(void);
//    void BluDatMngrSignal_PlotAccJerkUpdate(void);
//    void BluDatMngrSignal_PlotGyroUpdate(void);
//    void BluDatMngrSignal_PlotNormAccUpdate(void);
//    void BluDatMngrSignal_PlotVelUpdate(void);


//    void BluDatMngrSignal_PlotMapAppendData(float PosX, float PosY);
    void BluDatMngrSignal_PlotRawAccAppendData(uint32_t FrameId,float AccX,float AccY,float AccZ);
//    void BluDatMngrSignal_PlotEulerAgAppendData(uint32_t FrameId, float SpdValueLeftWh,float SpdValueRightWh);
//    void BluDatMngrSignal_PlotFildAccAppendData(uint32_t FrameId, float PossErrValue);
//    void BluDatMngrSignal_PlotAccJerkAppendData(uint32_t FrameId, float PidRegVal);
//    void BluDatMngrSignal_PlotGyroAppendData(uint32_t FrameId, float Orientation);
//    void BluDatMngrSignal_PlotNormAccAppendData(uint32_t FrameId, float TrvDistance);
//    void BluDatMngrSignal_PlotVelAppendData(uint32_t FrameId, uint8_t LeftPosConf, uint8_t RightPosConf);



private slots:
    void BluDatMngr_InputHanlder( char* data, uint32_t Size);

private:
    void BluDatMngr_BaseDataInsertToDebugTable(uint32_t FrameCounter);
    void BluDatMngr_DebugMessagerHandler(char *data,uint32_t size, BLU_MessageID_t BLE_MessID);
    void BluDatMngr_BaseDataHandler(char *data,uint32_t Size);
    void BluDatMngr_CommunicationStatistics_Handler(char *data,uint32_t Size);

};

#endif // BLUETOOTHDATAMANAGER_H
