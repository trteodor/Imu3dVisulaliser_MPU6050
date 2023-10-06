#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QtConcurrent>
#include "bluetoothDataManager.h"
#include "GenericQCP.h"

#include <QtQuick/QQuickView>
#include <QtQuick/QQuickItem>
#include <QUrl>

#include <QtDataVisualization>

//#include <QInputDialog>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();


    QString CurrentImuProjectFilePath;

private slots:

    void changeEvent( QEvent* e );

    void on_BLU_DisconnectButton_clicked();
    void on_BLU_SimulatorSuspendButton_clicked();
    void on_BLU_TrueLogStartButton_clicked();


    void on_GeneralPlotDataClear_pb_clicked();

    void MainWin_DebugTable_InsertDataRow(uint32_t ucTimeStamp, uint32_t FrameCounter, uint8_t SyncId, QString DecodedDataString,QColor RowColor = QColor( 255,255,255) );

    void MainWin_CommunicationStatisticsUpdate(uint32_t ucTimeStamp,uint16_t RingBufferRemainingSize,uint16_t RingBufferOverFlowCounter,
                                                           uint16_t TransmisstedMessagesCounter,uint16_t RetransmissionCounter);


    void MainWin_DebugTable_ScrollToBottom();

    void MainWinVis_Update3DOrientation(float yaw,float pitch,float roll);

    void MainWinPlot_3DPosUpdateAppendData(float PosX,float PosY,float PosZ);

//    void MainWinPlot_PlotMapReplot(void);
    void MainWinPlot_PlotRawAccReplot(void);
    void MainWinPlot_PlotEulerAgAReplot(void);
    void MainWinPlot_PlotFildAccReplot(void);
    void MainWinPlot_PlotAccJerkReplot(void);
    void MainWinPlot_PlotGyroReplot(void);
    void MainWinPlot_PlotNormAccReplot(void);
    void MainWinPlot_PlotVelReplot(void);



//    void MainWinPlot_PlotMapAppendData(float PosX, float PosY);
    void MainWinPlot_PlotRawAccAppendData(uint32_t FrameId,float AccX,float AccY,float AccZ);
    void MainWinPlot_PlotEulerAgAppendData(uint32_t FrameId,float yaw,float pitch,float roll);
    void MainWinPlot_PlotFildAccAppendData(uint32_t FrameId,float X,float Y,float Z);
    void MainWinPlot_PlotAccJerkAppendData(uint32_t FrameId,float X,float Y,float Z);
    void MainWinPlot_PlotGyroAppendData(uint32_t FrameId,float X,float Y,float Z);
    void MainWinPlot_PlotNormAccAppendData(uint32_t FrameId,float X,float Y,float Z);
    void MainWinPlot_PlotVelAppendData(uint32_t FrameId,float X,float Y,float Z);

    void MainWinPlot_DrawMarkersAtDataIndexInfo(int DataIndex);


    void on_DebugTable_DisableBaseDataLogging_clicked(bool checked);

    void on_ClearLoggerButton_clicked();

    void Plot3DDelayTimerTimeout();

    void on_GeneraReplotAllPlots_pb_clicked();


    void MainWin_bluetoothSlotDeviceDiscovered(QString name);
    void MainWin_bluetoothSlotDiscoveryFinished(void);

    void MainWin_bluetoothSlotConnectingStart();
    void MainWin_bluetoothSlotConnectionEstablished(void);
    void MainWin_bluetoothSlotConnectionInterrupted(void);

    void on_BLU_ScanButton_clicked();

    void on_BLU_ConnectButton_clicked();

    void on_RemoveMarkers_pb_clicked();

    void on_SaveAppState_pb_clicked();

    void on_LoadProject_pb_clicked();

    void on_actionAbout_triggered();

signals:
    void MainWin_bluetoothSignalStartDiscoveryDevices(void);
    void MainWin_bluetoothDisconnect(void);
    void MainWin_bluetootSignalConnectToDeviceByName(QString DevName);

private:
    Ui::MainWindow *ui;

    void Plot3DInit3DScatter3DPosVisualser(void);
    void BLU_InitializeQTConnections(void);

    bool  NvM_DataLoadedFromExternalSourceFlag = false;


    QScatter3DSeries ImuDataseries;
    QScatterDataArray ImuDataArray;
    Q3DScatter Imuscatter;
    QTimer RealTimeData3DTimer;

    QQuickView *view3DOri;
    QObject *object3dview;

    QList<QString> FoundDevices;
    BluDataManager BluInputDataProcessingWrapper;


    QDoubleValidator dblValidator;

    void BLE_CommunicationStatistics_Handler(const QByteArray &value);
    void LoadDataImuDataVisualiserProject(QString FilePath);
    void ConfigureTextLineAndNvMConnections(void);

    GenericQCP PlotMap;
    GenericQCP PlotAcc;
    GenericQCP PlotFildAcc;
    GenericQCP PlotEulerAg;
    GenericQCP PlotGyro;
    GenericQCP PlotNacc;
    GenericQCP PlotJrk;
    GenericQCP PlotVelo;

};
#endif // MAINWINDOW_H
