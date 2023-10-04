#include "bluetoothDataManager.h"
#include "qthread.h"

BluDataManager::BluDataManager()
{
    this->moveToThread(&BluDatMngr_Thread);
    bleutoothClassicConnection.moveToThread((&BluDatMngr_Thread)) ;

    connect(&bleutoothClassicConnection, SIGNAL(bluetoothSignalNewDataReceived(char *, uint32_t)), this, SLOT(BluDatMngr_InputHanlder( char*, uint32_t) ));

    BluDatMngr_Thread.start();
}

BluDataManager::~BluDataManager()
{
    BluDatMngr_Thread.terminate();
}

/*****************************************/
/*Tools functions start for Main Window and BLU*/
/**/
uint32_t ConvToUint32(char *data)
{
    return (((uint8_t)data[0] ) | ( (uint8_t)data[1] << 8) \
            | ( (uint8_t)data[2] << 16) | ( (uint8_t)data[3] << 24));

}
/**************************************************************/
uint32_t ConvToUint16(char *data)
{
    return (((uint8_t)data[0] ) | ( (uint8_t)data[1] << 8) );
}
/**************************************************************/
float ieee_uint32_AsBitsTo_float32(uint32_t f)
{
    float ret;
    std::memcpy(&ret, &f, sizeof(float));
    return ret;
}
/*Tools functions end */


void BluDataManager::BluDatMngr_CommunicationStatistics_Handler(char *data,uint32_t Size)
{
    (void)Size;
    static BLU_StatisticData_t StatisticData = {0};
    static uint8_t PreviousSyncId = 255U;

//    qDebug() << "StatisticHandler SyncId" <<  ((uint8_t)value.at(1)) ;

    StatisticData.SyncId = ((uint8_t)data[1]) ;
    StatisticData.ucTimeStamp = ConvToUint32(&data[2]);
    StatisticData.RingBufferRemainingSize = ConvToUint16(&data[6]);
    StatisticData.RingBufferOverFlowCounter = ConvToUint16(&data[8]);
    StatisticData.TransmisstedMessagesCounter = ConvToUint16(&data[10]);
    StatisticData.RetransmissionCounter = ConvToUint16(&data[12]);


    if(PreviousSyncId != StatisticData.SyncId)
    {
        emit BluDatMngrSignal_CommunicationStatisticsUpdate(StatisticData.ucTimeStamp,StatisticData.RingBufferRemainingSize,StatisticData.RingBufferOverFlowCounter,
                                                            StatisticData.TransmisstedMessagesCounter,StatisticData.RetransmissionCounter);
    }

    PreviousSyncId = StatisticData.SyncId;
}



void BluDataManager::BluDatMngr_BaseDataInsertToDebugTable(uint32_t FrameCounter)
{

//        QString RgtWhlSpdSimu_s = QString::number(BluDataManager::FullBaseData.CurrMapData.WhLftSp,'f',3);
//        QString LftWhlSpdSimu_s = QString::number(BluDataManager::FullBaseData.CurrMapData.WhRhtSp,'f',3);

//        QString BaseMapData = QString("MapDat: pX: %1 |pY: %2 pO: %3 |L_sp: %4 |R_Sp: %5| Yr: %6")
//                                  .arg(BluDataManager::FullBaseData.CurrMapData.PosX)
//                                  .arg(BluDataManager::FullBaseData.CurrMapData.PosY)
//                                  .arg(BluDataManager::FullBaseData.CurrMapData.PosO)
//                                  .arg(LftWhlSpdSimu_s).arg(RgtWhlSpdSimu_s)
//                                  .arg(BluDataManager::FullBaseData.CurrMapData.YawRate) ;

////        QColor RowColor = QColor(255,255,255); /*Default row color (not modify color*/
//        emit BluDatMngrSignal_DebugTable_InsertDataRow(FullBaseData.ucTimeStamp,FrameCounter,BluDataManager::FullBaseData.SyncId,BaseMapData);
}


void BluDataManager::BluDatMngr_BaseDataHandler(char *data,uint32_t Size)
{
    (void)Size;
    static uint32_t FullFrameCounter = 0;
    uint8_t _inputSyncId = ((uint8_t)data[1]);

    ImuDataRep.SyncId = _inputSyncId;
    ImuDataRep.ucTimeStamp = ConvToUint32(&data[2]);
    ImuDataRep.gyroX = ieee_uint32_AsBitsTo_float32(ConvToUint32(&data[6])) ;
    ImuDataRep.gyroY = ieee_uint32_AsBitsTo_float32(ConvToUint32(&data[10])) ;
    ImuDataRep.gyroZ = ieee_uint32_AsBitsTo_float32(ConvToUint32(&data[14])) ;
    ImuDataRep.accX = ieee_uint32_AsBitsTo_float32(ConvToUint32(&data[18])) ;
    ImuDataRep.accY = ieee_uint32_AsBitsTo_float32(ConvToUint32(&data[22])) ;
    ImuDataRep.accZ = ieee_uint32_AsBitsTo_float32(ConvToUint32(&data[26])) ;
    ImuDataRep.normalizedAccX = ieee_uint32_AsBitsTo_float32(ConvToUint32(&data[30])) ;
    ImuDataRep.normalizedAccY = ieee_uint32_AsBitsTo_float32(ConvToUint32(&data[34])) ;
    ImuDataRep.normalizedAccZ = ieee_uint32_AsBitsTo_float32(ConvToUint32(&data[38])) ;
    ImuDataRep.fildAccX = ieee_uint32_AsBitsTo_float32(ConvToUint32(&data[42])) ;
    ImuDataRep.fildAccY = ieee_uint32_AsBitsTo_float32(ConvToUint32(&data[46])) ;
    ImuDataRep.fildAccZ = ieee_uint32_AsBitsTo_float32(ConvToUint32(&data[50])) ;
    ImuDataRep.jerkX = ieee_uint32_AsBitsTo_float32(ConvToUint32(&data[54])) ;
    ImuDataRep.jerkY = ieee_uint32_AsBitsTo_float32(ConvToUint32(&data[58])) ;
    ImuDataRep.jerkZ = ieee_uint32_AsBitsTo_float32(ConvToUint32(&data[62])) ;
    ImuDataRep.roll = ieee_uint32_AsBitsTo_float32(ConvToUint32(&data[66])) ;
    ImuDataRep.pitch = ieee_uint32_AsBitsTo_float32(ConvToUint32(&data[70])) ;
    ImuDataRep.yaw = ieee_uint32_AsBitsTo_float32(ConvToUint32(&data[74])) ;
    ImuDataRep.velX = ieee_uint32_AsBitsTo_float32(ConvToUint32(&data[78])) ;
    ImuDataRep.velY = ieee_uint32_AsBitsTo_float32(ConvToUint32(&data[82])) ;
    ImuDataRep.velZ = ieee_uint32_AsBitsTo_float32(ConvToUint32(&data[86])) ;
    ImuDataRep.posX = ieee_uint32_AsBitsTo_float32(ConvToUint32(&data[90])) ;
    ImuDataRep.posY = ieee_uint32_AsBitsTo_float32(ConvToUint32(&data[94])) ;
    ImuDataRep.posZ = ieee_uint32_AsBitsTo_float32(ConvToUint32(&data[98])) ;

    FullFrameCounter++;



    if(true == DebugTable_BaseDataLoggingState)
    {
        BluDatMngr_BaseDataInsertToDebugTable(FullFrameCounter);
    }

    emit BluDatMngrSignal_Update3DOrientation(ImuDataRep.yaw,ImuDataRep.pitch,ImuDataRep.roll);
    //    emit BluDatMngrSignal_PlotMapAppendData(FullBaseData.CurrMapData.PosX,FullBaseData.CurrMapData.PosY);


//    if( (false == MapPlotPlottingState) && (false == YawRatePlotPlottingState)) )
//    {
//        PlottingInfoMutex.lock();
//        MapPlotPlottingState = true;
//        YawRatePlotPlottingState = true;
//        SpdPlotPlottingState = true;
//        PosErrPlotPlottingState = true;
//        PidRegValPlotPlottingState = true;
//        LinePosConfPlotPlottingState = true;
//        TrvDistancePlotPlottingState = true;
//        OrientationPlotPlottingState = true;

//        PlottingInfoMutex.unlock();


//        if(true == DebugTable_BaseDataLoggingState)
//        {
//            DebugTableScrollingBottonMutex.lock();
//            emit BluDatMngrSignal_DebugTable_ScrollToBottom();
//            DebugTableScrollingBottonMutex.unlock();
//        }

//        emit BluDatMngrSignal_PlotMapUpdate(); /*Move plotting to MainWindow process*/
//        emit BluDatMngrSignal_PlotYawRateUpdate(); /*Move plotting to MainWindow process*/
//        emit BluDatMngrSignal_PlotSpdUpdate(); /*Move plotting to MainWindow process*/
//        emit BluDatMngrSignal_PlotPosErrUpdate();
//        emit BluDatMngrSignal_PlotPidRegValUpdate();
//        emit BluDatMngrSignal_PlotOrientationReplot();
//        emit BluDatMngrSignal_PlotTrvDistanceReplot();
//        emit BluDatMngrSignal_PlotPosConfidenceReplot();

//    }


    static uint32_t PrevSyncId = 255U;
    if( ((uint8_t)(PrevSyncId+1)) != _inputSyncId)
    {
        QString SyncErrorString = QString("!!!Synchronization Error!!! BaseDataHandler SyncId: %1 |PrSyncId: %2").arg(_inputSyncId).arg(PrevSyncId) ;
        QColor RowColor = QColor(255,0,0);
        emit BluDatMngrSignal_DebugTable_InsertDataRow(0,0,0,SyncErrorString,RowColor);
        emit BluDatMngrSignal_DebugTable_ScrollToBottom();
    }

    PrevSyncId = _inputSyncId;
}

void BluDataManager::BluDatMngr_DebugMessagerHandler(char *data,uint32_t size, BLU_MessageID_t BLE_MessID)
{
    (void )BLE_MessID;
    (void)size;

    uint8_t _inputSyncId = data[1] ;
    static uint8_t PrevSyncId = 255;
    uint32_t ucTimeStamp;
    QString DebugString;
    ucTimeStamp = ConvToUint32(&data[2]);
    DebugString = QString::fromUtf8(&data[6]);
    emit BluDatMngrSignal_DebugTable_InsertDataRow(ucTimeStamp, BLE_MessID,_inputSyncId, DebugString);
    emit BluDatMngrSignal_DebugTable_ScrollToBottom();

    if(_inputSyncId != (uint8_t)(PrevSyncId+1))
    {
        QString SyncErrorString = QString("!!!Synchronization Error!!! DebugMessagerHandler SyncId: %2 |PrSyncId: %3 ")
                                      .arg(_inputSyncId).arg(PrevSyncId) ;
        QColor RowColor = QColor(255,0,0);
        qDebug() << SyncErrorString;
        emit BluDatMngrSignal_DebugTable_InsertDataRow(0,0,0,SyncErrorString,RowColor);
        emit BluDatMngrSignal_DebugTable_ScrollToBottom();
    }

//    qDebug() << "SyncID:" << _inputSyncId << "Size:" << size << "Mes:" << DebugString;
//  qDebug() << "SyncID:" << (uint8_t)data[1] << "ucTimeSt:" << ucTimeStamp << "Mes:" << &data[6];

    PrevSyncId = _inputSyncId;
}



void BluDataManager::BluDatMngr_InputHanlder( char* data, uint32_t Size)
{
    static volatile BLU_MessageID_t BLU_MessageID;
    BLU_MessageID = ((BLU_MessageID_t)data[0] );

    //qDebug() << "RecBaseData? MessID: " << BLU_MessageID;

    switch(BLU_MessageID)
    {
        case BLU_MessageID_t::BLU_CommunicationStats:
        {

            BluDatMngr_CommunicationStatistics_Handler(data,Size);
            break;
        }

        case BLU_MessageID_t::BLU_DebugMessage:
        {
            BluDatMngr_DebugMessagerHandler(data,Size,BLU_MessageID);
            break;
        }

        case BLU_MessageID_t::BLU_ImuData:
        {
            BluDatMngr_BaseDataHandler(data,Size);
            break;
        }


        default:
        {
            break;
        }
    }
}
