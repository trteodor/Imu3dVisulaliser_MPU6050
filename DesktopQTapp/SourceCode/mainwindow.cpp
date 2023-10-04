#include "mainwindow.h"
#include "ui_mainwindow.h"



#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>


/**************************************************************/
MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    QThread::currentThread()->setObjectName("Main Window Thread");

    ui->setupUi(this);

    this->setWindowTitle("Imu Data 3D Visuliser QT");


//    /*All declared plots/ graph must be initialized!!!*/
//    PlotMap.LfGraphInitialize(ui->MapViewWidget,QCPGraph::lsNone);
//    PlotYawRate.LfGraphInitialize(ui->PlotYawRateW,QCPGraph::lsLine);
//    PlotSpd.LfGraphInitialize(ui->PlotSpdW,QCPGraph::lsLine);
//    PlotPosErr.LfGraphInitialize(ui->PlotNormAcc,QCPGraph::lsLine);
//    PlotPidRegVal.LfGraphInitialize(ui->PlotAccJerk,QCPGraph::lsLine);
//    PlotTrvDistance.LfGraphInitialize(ui->PlotFildAcc,QCPGraph::lsLine);
//    PlotOrientation.LfGraphInitialize(ui->PlotEulerAg,QCPGraph::lsLine);
//    PlotLinePosConfidence.LfGraphInitialize(ui->PlotLinePosConfW,QCPGraph::lsLine);

//    connect(&PlotMap, SIGNAL(LfGraphSignal_graphClicked(int)), this, SLOT(MainWinPlot_DrawMarkersAtDataIndexInfo(int) ));
//    connect(&PlotYawRate, SIGNAL(LfGraphSignal_graphClicked(int)), this, SLOT(MainWinPlot_DrawMarkersAtDataIndexInfo(int) ));
//    connect(&PlotSpd, SIGNAL(LfGraphSignal_graphClicked(int)), this, SLOT(MainWinPlot_DrawMarkersAtDataIndexInfo(int) ));
//    connect(&PlotPosErr, SIGNAL(LfGraphSignal_graphClicked(int)), this, SLOT(MainWinPlot_DrawMarkersAtDataIndexInfo(int) ));
//    connect(&PlotPidRegVal, SIGNAL(LfGraphSignal_graphClicked(int)), this, SLOT(MainWinPlot_DrawMarkersAtDataIndexInfo(int) ));
//    connect(&PlotTrvDistance, SIGNAL(LfGraphSignal_graphClicked(int)), this, SLOT(MainWinPlot_DrawMarkersAtDataIndexInfo(int) ));
//    connect(&PlotOrientation, SIGNAL(LfGraphSignal_graphClicked(int)), this, SLOT(MainWinPlot_DrawMarkersAtDataIndexInfo(int) ));
//    connect(&PlotLinePosConfidence, SIGNAL(LfGraphSignal_graphClicked(int)), this, SLOT(MainWinPlot_DrawMarkersAtDataIndexInfo(int) ));


    /*Initialize all needed connections for Bluetooth Data Manager*/
    BLU_InitializeQTConnections();


    /*Assign 3D orientation Cube to correct layout/ container*/
    view3DOri = new QQuickView();
    view3DOri->setSource(QUrl("qrc:/res/CubeDiffFaces.qml"));
    #ifndef _WIN32
        view3DOri->setClearBeforeRendering(true);
        view3DOri->setColor(QColor(Qt::transparent));
    #endif
    // Create a container widget for the QQuickView
    QWidget *container3DOri = QWidget::createWindowContainer(view3DOri, this);
    container3DOri->setMinimumSize(220, 220);
    container3DOri->setMaximumSize(220, 220);
    container3DOri->setFocusPolicy(Qt::TabFocus);
    ui->Orientation3DLayout->addWidget(container3DOri);


    object3dview = view3DOri->rootObject();


    /*Debug Table configure*/
    ui->DebugDataTable->setRowCount(0);
    ui->DebugDataTable->setColumnWidth(0,10);
    ui->DebugDataTable->setColumnWidth(1,10);
    ui->DebugDataTable->setColumnWidth(2,10);
    ui->DebugDataTable->setColumnWidth(3,10);
    ui->DebugDataTable->setColumnWidth(4,600);
    ui->DebugDataTable->horizontalHeader()->setDefaultAlignment(Qt::AlignLeft);


    /*Initialize dark theme*/
    QFile f(":qdarkstyle/dark/darkstyle.qss");

    if (!f.exists())   {
        qDebug() << "Unable to set stylesheet, file not found\n";
    }

    else   {
        f.open(QFile::ReadOnly | QFile::Text);
        QTextStream ts(&f);
        qApp->setStyleSheet(ts.readAll());
    }


    QSettings settings("Imu3DVisuApp", "BluDeviceName");
//    settings.setValue("CurrDeviceName", "Franek");
    QVariant CurrDevName = settings.value("CurrDeviceName");
    QString NewLineEditText = CurrDevName.toString();
//    qDebug() << NewLineEditText ;

    ui->BLU_AutoConnDevNameL->setText(NewLineEditText);


//    if(qApp->arguments().count() > 0)
//    {
//        qDebug() << "argumentsCount" << qApp->arguments().at(0);
//    }

    if(qApp->arguments().count() > 1  && qApp->arguments().at(1).endsWith(".lfp") ==true)
    {
        CurrentLfProjectFilePath = qApp->arguments().at(1);
        QString LoadedProjectInfo = QString("DataLoadedFromProjectFile: %1").arg(CurrentLfProjectFilePath);
        MainWin_DebugTable_InsertDataRow(0, 0, 0, LoadedProjectInfo);

        LoadDataImuDataVisualiserProject(CurrentLfProjectFilePath);
    }


    Plot3DInit3DScatter3DPosVisualser();

}


/*********************************************************************************************************/
MainWindow::~MainWindow()
{
    on_BLU_SimulatorSuspendButton_clicked();
    QThread::msleep(25);
    on_BLU_SimulatorSuspendButton_clicked();
    QThread::msleep(25);
    on_BLU_SimulatorSuspendButton_clicked();
    QThread::msleep(25);
    /*Send the command 3x to be sure that fakeProducer will be stopped, if not then re-connection may be impossible
     * - HW reset may be required
    */

//    TODO: Veriyfy? emit BLU_DisconnectDevice();

    QSettings settings("Imu3DVisuApp", "BluDeviceName");
    QString NewSearchedBluDeviceName = ui->BLU_AutoConnDevNameL->text();
    settings.setValue("CurrDeviceName", NewSearchedBluDeviceName);

    delete ui;

    qDebug("Reached end");
}

void MainWindow::Plot3DInit3DScatter3DPosVisualser(void)
{
    qputenv("QSG_RHI_BACKEND", "opengl");


    Imuscatter.setFlags(Imuscatter.flags() ^ Qt::FramelessWindowHint);
    Imuscatter.activeTheme()->setType(Q3DTheme::ThemeIsabelle);
    QFont font = Imuscatter.activeTheme()->font();
    font.setPointSize(40.0F);
    Imuscatter.activeTheme()->setFont(font);
    Imuscatter.setShadowQuality(QAbstract3DGraph::ShadowQualitySoftLow);
    Imuscatter.scene()->activeCamera()->setCameraPreset(Q3DCamera::CameraPresetFront);
    Imuscatter.axisX()->setTitle("X");
    Imuscatter.axisY()->setTitle("Y");
    Imuscatter.axisZ()->setTitle("Z");
    Imuscatter.axisX()->setLabelFormat("%.2f mm");
    Imuscatter.axisY()->setLabelFormat("%.2f mm");
    Imuscatter.axisZ()->setLabelFormat("%.2f mm");

    //    QScatterDataProxy *proxy = new QScatterDataProxy;


    ImuDataseries.setItemLabelFormat(QStringLiteral("@xTitle: @xLabel @yTitle: @yLabel @zTitle: @zLabel"));
    ImuDataseries.setMeshSmooth(true);

    ImuDataArray << QVector3D(0.5f, 0.5f, 0.5f) << QVector3D(-0.3f, -0.5f, -0.4f) << QVector3D(0.0f, -0.3f, 0.2f);
    ImuDataseries.dataProxy()->addItems(ImuDataArray);
    ImuDataseries.setItemSize(0.1F);

    Imuscatter.addSeries(&ImuDataseries);

    QWidget *container = QWidget::createWindowContainer(&Imuscatter);
    ui->D3_LayoutTest->addWidget(container);

    connect(&RealTimeData3DTimer, SIGNAL(timeout()), this, SLOT(Plot3DDelayTimerTimeout()));
    RealTimeData3DTimer.setSingleShot(true);
    RealTimeData3DTimer.start(100);
}

void MainWindow::Plot3DDelayTimerTimeout()
{
    static float iterData=0;
    QScatterDataArray tempImuDataArr;
    tempImuDataArr << QVector3D(5*sin(iterData+ 0.5f),iterData+  0.5f, iterData+ 0.5f);
    ImuDataseries.dataProxy()->addItems(tempImuDataArr);

    QString returnedValue;
    //QString msg = "Hello from C++";

//    static uint32_t yaw,pitch,roll;
//    yaw = yaw + 10;
//    pitch = pitch +20;
//    roll = roll + 5;

//    QMetaObject::invokeMethod(object3dview, "updateCubeOrientation",
//                            Q_ARG(QVariant, yaw),
//                            Q_ARG(QVariant, pitch),
//                            Q_ARG(QVariant, roll)   );

    //qDebug() << "QML function returned:" << returnedValue;

    iterData = iterData + 0.1F;
    RealTimeData3DTimer.setSingleShot(true);
    RealTimeData3DTimer.start(100);


}

void MainWindow::changeEvent( QEvent* e )
{
    if( e->type() == QEvent::WindowStateChange )
    {
        qDebug() << "this->windowState()" << this->windowState();

        QWindowStateChangeEvent* event = static_cast< QWindowStateChangeEvent* >( e );

        if( event->oldState() & Qt::WindowMinimized )
        {
            qDebug() << "Window restored (to normal or maximized state)!";
        }
        else if( event->oldState() == Qt::WindowNoState && this->windowState() == Qt::WindowMaximized )
        {
            qDebug() << "Window Maximized!";

        }
        else if(this->windowState() == Qt::WindowNoState && event->oldState() == Qt::WindowMaximized){
            qDebug() << "Window restored to normal from maximazed state";
        }
    }
}

/*********************************************************************************************************/
/*********************************************************************************************************/
/*********************************************************************************************************/
void MainWindow::MainWin_bluetoothSlotDeviceDiscovered(QString name)
{
//    qDebug() << "MainWin_bluetoothSlotDeviceDiscovered  called";

    ui->BLU_ConnectButton->setEnabled(true);
    ui->BLU_DetectedDeviceComboBox->setEnabled(true);

    ui->BLU_StatusLabel->setText("State:NewDevDet");

    ui->BLU_DetectedDeviceComboBox->addItem(name);

    QString AutoConnDevName = ui->BLU_AutoConnDevNameL->text();

    if( name == AutoConnDevName && ui->BLU_AutoConnCheckBox->isChecked() )
    {
        ui->BLU_DetectedDeviceComboBox->setCurrentIndex( ui->BLU_DetectedDeviceComboBox->count() - 1 );

        ui->DebugDataTable->insertRow(ui->DebugDataTable->rowCount() );
        ui->DebugDataTable->setItem(ui->DebugDataTable->rowCount() -1 ,4,
                                    new QTableWidgetItem(QString("DevNameMatched:Connecting ") ));

        emit MainWin_bluetootSignalConnectToDeviceByName(AutoConnDevName);
    }
    else
    {
        ui->statusbar->showMessage("Please select BLE device",1000);
    }
}

void MainWindow::MainWin_bluetoothSlotDiscoveryFinished(void)
{
    ui->BLU_ConnectButton->setEnabled(true);
    ui->BLU_ScanButton->setEnabled(true);
    ui->BLU_DetectedDeviceComboBox->setEnabled(true);

    ui->statusbar->showMessage("DiscoveryFinished",1000);
//    ui->BLU_StatusLabel->setText("DiscoveryFinished");
}

void MainWindow::MainWin_bluetoothSlotConnectingStart()
{
    ui->BLU_StatusLabel->setText("State:Connecting");
    ui->statusbar->showMessage("Connecting to device...",1000);
}

void MainWindow::MainWin_bluetoothSlotConnectionEstablished(void)
{
    ui->statusbar->showMessage("Aquire data",1000);

    ui->BLU_StatusLabel->setText("State:ConnReadyAcqData");
    QVariant variant= QColor (220,255,220);
    QString colcode = variant.toString();
    ui->BLU_StatusLabel->setAutoFillBackground(true);
    ui->BLU_StatusLabel->setStyleSheet("QLabel { background-color :"+colcode+" ; color : black; }");

    if(false == NvM_DataLoadedFromExternalSourceFlag)
    {
        NvM_DataLoadedFromExternalSourceFlag = true;
    }

}

void MainWindow::MainWin_bluetoothSlotConnectionInterrupted(void)
{

    ui->statusbar->showMessage("Connection interrupted, why?",1000);


    ui->BLU_StatusLabel->setText("State:Disconnected");
    QVariant variant= QColor (255,255,255);
    QString colcode = variant.toString();
    ui->BLU_StatusLabel->setAutoFillBackground(true);
    ui->BLU_StatusLabel->setStyleSheet("QLabel { background-color :"+colcode+" ; color : black; }");
}

void MainWindow::on_BLU_ScanButton_clicked()
{
    ui->BLU_StatusLabel->setText("State:Scanning");
    ui->BLU_DetectedDeviceComboBox->clear();
    ui->BLU_ConnectButton->setEnabled(false);
    ui->BLU_ScanButton->setEnabled(false);
    ui->BLU_DetectedDeviceComboBox->setEnabled(true);
    ui->statusbar->showMessage("Searching for bluetooth devices...",1000);
    emit MainWin_bluetoothSignalStartDiscoveryDevices();
}

void MainWindow::on_BLU_ConnectButton_clicked()
{
    QString NewAutoConnDevName = ui->BLU_DetectedDeviceComboBox->itemText(ui->BLU_DetectedDeviceComboBox->currentIndex());
    ui->BLU_AutoConnDevNameL->setText(NewAutoConnDevName);
    emit MainWin_bluetootSignalConnectToDeviceByName(NewAutoConnDevName);
}

void MainWindow::on_BLU_DisconnectButton_clicked()
{

    emit MainWin_bluetoothDisconnect();

    ui->BLU_StatusLabel->setText("State:Disconnected");
    ui->BLU_ConnectButton->setEnabled(true);
    ui->BLU_ScanButton->setEnabled(true);
    QVariant variant= QColor (255,255,255);
    QString colcode = variant.toString();
    ui->BLU_StatusLabel->setAutoFillBackground(true);
    ui->BLU_StatusLabel->setStyleSheet("QLabel { background-color :"+colcode+" ; color : black; }");
}


/***************************************************************************/
/***************************************************************************/
/***************************************************************************/

void MainWindow::BLU_InitializeQTConnections(void)
{

    /*Bluetooth connect connections start*/
    connect(
        &BluInputDataProcessingWrapper.bleutoothClassicConnection,
        SIGNAL(bluetoothSignalDeviceDiscovered(QString) )
        ,this
        ,SLOT(MainWin_bluetoothSlotDeviceDiscovered(QString) )   );

    connect(
        &BluInputDataProcessingWrapper.bleutoothClassicConnection,
        SIGNAL(bluetoothSignalDiscoveryFinished() )
        ,this
        ,SLOT(MainWin_bluetoothSlotDiscoveryFinished() ));


    connect(
        &BluInputDataProcessingWrapper.bleutoothClassicConnection,
        SIGNAL(bluetoothSignalConnectingStart() )
        ,this
        ,SLOT(MainWin_bluetoothSlotConnectingStart() ));


    connect(
        &BluInputDataProcessingWrapper.bleutoothClassicConnection,
        SIGNAL( bluetoothSignalConnectionEstablished() )
        ,this
        ,SLOT(MainWin_bluetoothSlotConnectionEstablished() ));


    connect(
        &BluInputDataProcessingWrapper.bleutoothClassicConnection,
        SIGNAL( bluetoothSignalConnectionInterrupted() )
        ,this
        ,SLOT(MainWin_bluetoothSlotConnectionInterrupted() ));
    /**/
    connect(
        this,
        SIGNAL( MainWin_bluetoothSignalStartDiscoveryDevices() )
        ,&BluInputDataProcessingWrapper.bleutoothClassicConnection
        ,SLOT(bluetoothStartDiscoveryDevices() ) );

    connect(
        this,
        SIGNAL( MainWin_bluetoothDisconnect() )
        ,&BluInputDataProcessingWrapper.bleutoothClassicConnection
        ,SLOT(bluetoothDisconnect() ) );

    connect(
        this,
        SIGNAL( MainWin_bluetootSignalConnectToDeviceByName(QString) )
        ,&BluInputDataProcessingWrapper.bleutoothClassicConnection
        ,SLOT(bluetootConnectToDeviceByName(QString) ) );
    /*Bluetooth connect connections end*/











//    connect(
//        &BluInputDataProcessingWrapper,
//        SIGNAL(BluDatMngrSignal_PlotMapUpdate() )
//        ,this
//        ,SLOT(MainWinPlot_PlotMapReplot() ));

//    connect(
//        &BluInputDataProcessingWrapper,
//        SIGNAL(BluDatMngrSignal_PlotYawRateUpdate() )
//        ,this
//        ,SLOT(MainWinPlot_PlotYawRateReplot() ));

//    connect(
//        &BluInputDataProcessingWrapper,
//        SIGNAL(BluDatMngrSignal_PlotSpdUpdate() )
//        ,this
//        ,SLOT(MainWinPlot_PlotSpdReplot() ));


//    connect(
//        &BluInputDataProcessingWrapper,
//        SIGNAL(BluDatMngrSignal_PlotPosErrUpdate() )
//        ,this
//        ,SLOT(MainWinPlot_PlotPosErrReplot() ));

//    connect(
//        &BluInputDataProcessingWrapper,
//        SIGNAL(BluDatMngrSignal_PlotPidRegValUpdate() )
//        ,this
//        ,SLOT(MainWinPlot_PlotPidRegValReplot() ));



//    connect(
//        &BluInputDataProcessingWrapper,
//        SIGNAL(BluDatMngrSignal_PlotMapAppendData(float,float) )
//        ,this
//        ,SLOT(MainWinPlot_PlotMapAppendData(float,float) ));

//    connect(
//        &BluInputDataProcessingWrapper,
//        SIGNAL(BluDatMngrSignal_PlotYawRateAppendData(uint32_t,float) )
//        ,this
//        ,SLOT(MainWinPlot_PlotYawRateAppendData(uint32_t,float) ) );

//    connect(
//        &BluInputDataProcessingWrapper,
//        SIGNAL(BluDatMngrSignal_PlotSpdAppendData(uint32_t,float,float) )
//        ,this
//        ,SLOT(MainWinPlot_PlotSpdAppendData(uint32_t,float,float) ) );

//    connect(
//        &BluInputDataProcessingWrapper,
//        SIGNAL(BluDatMngrSignal_PlotPosErrAppendData(uint32_t,float) )
//        ,this
//        ,SLOT(MainWinPlot_PlotPosErrAppendData(uint32_t,float) ) );

//    connect(
//        &BluInputDataProcessingWrapper,
//        SIGNAL(BluDatMngrSignal_PlotPidRegValAppendData(uint32_t,float) )
//        ,this
//        ,SLOT(MainWinPlot_PlotPidRegValAppendData(uint32_t,float) ) );





//    connect(
//        &BluInputDataProcessingWrapper,
//        SIGNAL(BluDatMngrSignal_PlotOrientationAppendData(uint32_t,float) )
//        ,this
//        ,SLOT(MainWinPlot_PlotOrientationAppendData(uint32_t,float) ) );


//    connect(
//        &BluInputDataProcessingWrapper,
//        SIGNAL(BluDatMngrSignal_PlotOrientationReplot() )
//        ,this
//        ,SLOT(MainWinPlot_PlotOrientationReplot() ));

//    connect(
//        &BluInputDataProcessingWrapper,
//        SIGNAL(BluDatMngrSignal_PlotTrvDistanceAppendData(uint32_t,float) )
//        ,this
//        ,SLOT(MainWinPlot_PlotTrvDistanceAppendData(uint32_t,float) ) );


//    connect(
//        &BluInputDataProcessingWrapper,
//        SIGNAL(BluDatMngrSignal_PlotTrvDistanceReplot() )
//        ,this
//        ,SLOT(MainWinPlot_PlotTrvDistanceReplot() ));


//    connect(
//        &BluInputDataProcessingWrapper,
//        SIGNAL(BluDatMngrSignal_PlotPosConfidenceAppendData(uint32_t, uint8_t, uint8_t) )
//        ,this
//        ,SLOT(MainWinPlot_PlotPosConfidenceAppendData(uint32_t, uint8_t, uint8_t) ) );


//    connect(
//        &BluInputDataProcessingWrapper,
//        SIGNAL(BluDatMngrSignal_PlotPosConfidenceReplot() )
//        ,this
//        ,SLOT(MainWinPlot_PlotPosConfidenceReplot() ));




    connect(
        &BluInputDataProcessingWrapper,
        SIGNAL(BluDatMngrSignal_DebugTable_InsertDataRow(uint32_t,uint32_t,uint8_t,QString,QColor) )
        ,this
        ,SLOT(MainWin_DebugTable_InsertDataRow(uint32_t,uint32_t,uint8_t,QString,QColor) ) );

    connect(
        &BluInputDataProcessingWrapper,
        SIGNAL(BluDatMngrSignal_DebugTable_ScrollToBottom() )
        ,this
        ,SLOT(MainWin_DebugTable_ScrollToBottom() ) );



//    connect(
//        &BluInputDataProcessingWrapper,
//        SIGNAL(BluDatMngrSignal_RefreshErrorIndicatorView(uint8_t,uint8_t,uint8_t,uint8_t,uint8_t,uint8_t,
//                                                          uint8_t,uint8_t,uint8_t,uint8_t,uint8_t,uint8_t,
//                                                          uint8_t,uint8_t,
//                                                          float) )
//        ,this
//        ,SLOT(MainWin_RefreshErrorIndicatorView(uint8_t,uint8_t,uint8_t,uint8_t,uint8_t,uint8_t,
//                                               uint8_t,uint8_t,uint8_t,uint8_t,uint8_t,uint8_t,
//                                               uint8_t,uint8_t,
//                                               float) )
//        );

    connect(
        &BluInputDataProcessingWrapper,
        SIGNAL(BluDatMngrSignal_CommunicationStatisticsUpdate(uint32_t,uint16_t,uint16_t,uint16_t,uint16_t) )
        ,this
        ,SLOT(MainWin_CommunicationStatisticsUpdate(uint32_t,uint16_t,uint16_t,uint16_t,uint16_t) ) );


//    connect(
//        &BluInputDataProcessingWrapper,
//        SIGNAL(BluDatMngrSignal_UpdateErrorWeigthData(float,float,float,float,float,float,float,float,float,float,float,float))
//        ,this
//        ,SLOT(MainWin_UpdateNvmErrorWeigthData(float,float,float,float,float,float,float,float,float,float,float,float) ) );


//    connect(
//        &BluInputDataProcessingWrapper,
//        SIGNAL(BluDatMngrSignal_UpdateVehCfgData(float,uint32_t,uint32_t,uint32_t) )
//        ,this
//        ,SLOT(MainWin_UpdateNvM_VehCfgData(float, uint32_t, uint32_t,uint32_t) ) );

//    connect(
//        &BluInputDataProcessingWrapper,
//        SIGNAL(BluDatMngrSignal_UpdatePidData(float,float,float,uint32_t) )
//        ,this
//        ,SLOT(MainWin_UpdateNvM_PidData(float,float,float,uint32_t) ) );


//    connect(
//        &BluInputDataProcessingWrapper,
//        SIGNAL(BluDatMngrSignal_UpdateMotorsFactors(uint32_t,uint32_t,uint32_t,uint32_t) )
//        ,this
//        ,SLOT(MainWin_UpdateMotorsFactors(uint32_t, uint32_t, uint32_t,uint32_t) ) );

//    connect(
//        &BluInputDataProcessingWrapper,
//        SIGNAL(BluDatMngrSignal_UpdateEncoderCfgData(float,float) )
//        ,this
//        ,SLOT(MainWin_UpdateEncoderCfgData(float,float) ) );

//    connect(
//        &BluInputDataProcessingWrapper,
//        SIGNAL(BluDatMngrSignal_UpdateSpeedProfileData(BluDataManager::BLU_NvM_SpdProfileData_t) )
//        ,this
//        ,SLOT(MainWin_UpdateSpeedProfileData(BluDataManager::BLU_NvM_SpdProfileData_t) ));

//    connect(
//        &BluInputDataProcessingWrapper,
//        SIGNAL(BluDatMngrSignal_UpdateOrientation(float) )
//        ,this
//        ,SLOT(MainWin_DrawOrientationIndicator(float) ) );

        connect(
            &BluInputDataProcessingWrapper,
            SIGNAL(BluDatMngrSignal_Update3DOrientation(float,float,float) )
            ,this
            ,SLOT(MainWinVis_Update3DOrientation(float,float,float) ) );

}

void MainWindow::MainWinVis_Update3DOrientation(float yaw, float pitch, float roll)
{
    //qDebug() << "Hello? :/ ";
    QMetaObject::invokeMethod(object3dview, "updateCubeOrientation",
                                  Q_ARG(QVariant, yaw * (180/M_PI) ),
                            Q_ARG(QVariant, pitch* (180/M_PI)      ),
                            Q_ARG(QVariant, roll* (180/M_PI)       ) );
}
/*********************************************************************************************************/
void MainWindow::MainWin_DebugTable_InsertDataRow(uint32_t ucTimeStamp, uint32_t FrameCounter, uint8_t SyncId, QString DecodedDataString,QColor RowColor )
{
    QColor QColorHelper = QColor(255,255,255); /*QColor default arg. value*/


    ui->DebugDataTable->insertRow(ui->DebugDataTable->rowCount() );

    ui->DebugDataTable->setItem(ui->DebugDataTable->rowCount() -1 ,1,new QTableWidgetItem(QString::number(ucTimeStamp) ));
    ui->DebugDataTable->setItem(ui->DebugDataTable->rowCount() -1 ,2,new QTableWidgetItem(QString::number(FrameCounter) ));
    ui->DebugDataTable->setItem(ui->DebugDataTable->rowCount() -1 ,3,new QTableWidgetItem(QString::number(SyncId)));
    ui->DebugDataTable->setItem(ui->DebugDataTable->rowCount() -1 ,4,new QTableWidgetItem((QString)DecodedDataString.data() ) );


    QDateTime t = QDateTime::currentDateTime();
    QString st = t.toString("yy/MM/dd HH:mm:ss.zzz");

    ui->DebugDataTable->setItem(ui->DebugDataTable->rowCount() -1 ,0,new QTableWidgetItem(st)  ) ;

    if(QColorHelper != RowColor) /*default value*/
    {
        ui->DebugDataTable->item(ui->DebugDataTable->rowCount() -1 , 1) -> setData(Qt::BackgroundRole, RowColor);
        ui->DebugDataTable->item(ui->DebugDataTable->rowCount() -1 , 2) -> setData(Qt::BackgroundRole, RowColor);
        ui->DebugDataTable->item(ui->DebugDataTable->rowCount() -1 , 3) -> setData(Qt::BackgroundRole, RowColor);
        ui->DebugDataTable->item(ui->DebugDataTable->rowCount() -1 , 4) -> setData(Qt::BackgroundRole, RowColor);
    }
}


void MainWindow::MainWin_CommunicationStatisticsUpdate(uint32_t ucTimeStamp,uint16_t RingBufferRemainingSize,uint16_t RingBufferOverFlowCounter,
                                                       uint16_t TransmisstedMessagesCounter,uint16_t RetransmissionCounter)
{
    (void)RetransmissionCounter;

    QString StatData = QString("ucT:%1ms |BuffRemSize:%2 |OverFlowC:%3 |FrmCnt: %4")
                           .arg(ucTimeStamp).arg(RingBufferRemainingSize)
                           .arg(RingBufferOverFlowCounter).arg(TransmisstedMessagesCounter);

    ui->Label_CommunicationStat->setText(StatData);
}


void MainWindow::MainWin_DebugTable_ScrollToBottom()
{
    ui->DebugDataTable->scrollToBottom();
    BluInputDataProcessingWrapper.DebugTableScrollingBottonMutex.lock();
    BluInputDataProcessingWrapper.DebugTableScrollingBottomIsActivState = false;
    BluInputDataProcessingWrapper.DebugTableScrollingBottonMutex.unlock();
}



void MainWindow::on_DebugTable_DisableBaseDataLogging_clicked(bool checked)
{
    if(true == checked)
    {
        BluInputDataProcessingWrapper.DebugTable_BaseDataLoggingState = false;
    }
}

/*********************************************************************************************************/



/*********************************************************************************************************/
/*********************************************************************************************************/
/*********************************************************************************************************/
/*********************************************************************************************************/

void MainWindow::on_GeneralPlotDataClear_pb_clicked()
{
    PlotMap.Graph_ClearData();
    PlotSpd.Graph_ClearData();
    PlotYawRate.Graph_ClearData();
    PlotPosErr.Graph_ClearData();
    PlotPidRegVal.Graph_ClearData();
    PlotTrvDistance.Graph_ClearData();
    PlotOrientation.Graph_ClearData();
    PlotLinePosConfidence.Graph_ClearData();
}



/*********************************************************************************************************/

//void MainWindow::MainWinPlot_PlotPosErrReplot(void)
//{
//    int Index = ui->tabWidget_4->currentIndex();
//    if(Index== 2)
//    {
//        PlotPosErr.Graph_UpdateReplot();
//    }

//    BluInputDataProcessingWrapper.PlottingInfoMutex.lock();
//    BluInputDataProcessingWrapper.PosErrPlotPlottingState = FALSE;
//    BluInputDataProcessingWrapper.PlottingInfoMutex.unlock();
//}


//void MainWindow::MainWinPlot_PlotPosErrAppendData(uint32_t FrameId, float PossErrValue)
//{
//    PlotPosErr.Graph_AppendData(FrameId,PossErrValue);
//}

void MainWindow::MainWinPlot_DrawMarkersAtDataIndexInfo(int DataIndex)
{
//    static uint32_t CallCounter = 0;

//    PlotMap.Graph_DrawMarkersAtDataIndex(DataIndex);
//    PlotYawRate.Graph_DrawMarkersAtDataIndex(DataIndex);
//    PlotSpd.Graph_DrawMarkersAtDataIndex(DataIndex);
//    PlotPosErr.Graph_DrawMarkersAtDataIndex(DataIndex);
//    PlotPidRegVal.Graph_DrawMarkersAtDataIndex(DataIndex);
//    PlotTrvDistance.Graph_DrawMarkersAtDataIndex(DataIndex);
//    PlotOrientation.Graph_DrawMarkersAtDataIndex(DataIndex);
//    PlotLinePosConfidence.Graph_DrawMarkersAtDataIndex(DataIndex);

//    float ClickedPosX = PlotMap.DataVector_X1.at(DataIndex);
//    float ClickedPosY = PlotMap.DataVector_Y1.at(DataIndex);

//    float ClickedOri = PlotOrientation.DataVector_Y1.at(DataIndex);
//    MainWin_DrawOrientationIndicator(ClickedOri);

//    float ClickedTrvDist = PlotTrvDistance.DataVector_Y1.at(DataIndex);
//    uint8_t PlotLinePosConfidenceLeft = PlotLinePosConfidence.DataVector_Y1.at(DataIndex);
//    uint8_t PlotLinePosConfidenceRight = PlotLinePosConfidence.DataVector_Y2.at(DataIndex);

//    float ClickedYr = PlotYawRate.DataVector_Y1.at(DataIndex);
//    float ClickedSpdL = PlotSpd.DataVector_Y1.at(DataIndex);
//    float ClickedSpdR = PlotSpd.DataVector_Y2.at(DataIndex);
//    float ClickedPosErr = PlotPosErr.DataVector_Y1.at(DataIndex);
//    float ClickedPid = PlotPidRegVal.DataVector_Y1.at(DataIndex);

//    QString ClickedPointString = QString("Clicked at point: PosX:%1  |PosY:%2  |Yr:%3  |SpdL:%4  |SpdR:%5  |PosErr:%6  |PidV:%7"
//                                         " |Ori:%8 |TrvDist:%9 |LineConfL:%10  |LineConfR:%11")
//                                     .arg(ClickedPosX).arg(ClickedPosY).arg(ClickedYr)
//                                     .arg(ClickedSpdL).arg(ClickedSpdR).arg(ClickedPosErr)
//                                     .arg(ClickedPid).arg(ClickedOri).arg(ClickedTrvDist)
//                                     .arg(PlotLinePosConfidenceLeft).arg(PlotLinePosConfidenceRight);

//    if(CallCounter % 2 == 0)
//    {
//        QVariant variant= QColor (35,35,45,255);
//        QString colcode = variant.toString();
//        ui->ClickedPointInfo_lb->setAutoFillBackground(true);
//        ui->ClickedPointInfo_lb->setStyleSheet("QLabel { background-color :"+colcode+" ; color : white; }");
//    }
//    else{
//        QVariant variant= QColor (25,45,45,255);
//        QString colcode = variant.toString();
//        ui->ClickedPointInfo_lb->setAutoFillBackground(true);
//        ui->ClickedPointInfo_lb->setStyleSheet("QLabel { background-color :"+colcode+" ; color : white; }");
//    }


//    ui->ClickedPointInfo_lb->setText(ClickedPointString);

//    CallCounter++;
}

void MainWindow::on_RemoveMarkers_pb_clicked()
{
//    PlotMap.Graph_DrawMarkersAtDataIndex(0);
//    PlotYawRate.Graph_DrawMarkersAtDataIndex(0);
//    PlotSpd.Graph_DrawMarkersAtDataIndex(0);
//    PlotPosErr.Graph_DrawMarkersAtDataIndex(0);
//    PlotPidRegVal.Graph_DrawMarkersAtDataIndex(0);
//    PlotTrvDistance.Graph_DrawMarkersAtDataIndex(0);
//    PlotOrientation.Graph_DrawMarkersAtDataIndex(0);
//    PlotLinePosConfidence.Graph_DrawMarkersAtDataIndex(0);
}

//void MainWindow::MainWinPlot_PlotMapAppendData(float PosX, float PosY)
//{
//    PlotMap.Graph_AppendData(PosX,PosY);

//}

//void MainWindow::MainWinPlot_PlotMapReplot(void)
//{

////    QElapsedTimer timer;
////    timer.start();
//    int Index = ui->MainTabWidget->currentIndex();
//    if(Index== 1)
//    {
//        PlotMap.Graph_UpdateReplot();
//    }
////    qDebug() << "MainWinPlot_PlotMapReplot TOOK: " << timer.elapsed() << "milliseconds";
//    BluInputDataProcessingWrapper.PlottingInfoMutex.lock();
//    BluInputDataProcessingWrapper.MapPlotPlottingState = FALSE;
//    BluInputDataProcessingWrapper.PlottingInfoMutex.unlock();
//}
/*********************************************************************************************************/
void MainWindow::on_BLU_SimulatorSuspendButton_clicked()
{
    char command[BLU_SINGLE_TR_MESSAGE_SIZE-2];
    command[0] = (char)BluDataManager::BLU_LoggingStop;

    for(int i=1; i<(BLU_SINGLE_TR_MESSAGE_SIZE-2); i++)
    {
            command[i] = 'B';
    }

    QByteArray Helper = QByteArray::fromRawData(command,BLU_SINGLE_TR_MESSAGE_SIZE -2);
    Helper.append("\n\r");
    BluInputDataProcessingWrapper.bleutoothClassicConnection.bluetoothSendDataToDevice(Helper);
}

/*********************************************************************************************************/
void MainWindow::on_BLU_TrueLogStartButton_clicked()
{
    char command[BLU_SINGLE_TR_MESSAGE_SIZE-2];
    command[0] = (char)BluDataManager::BLU_LoggingStart;

    for(int i=1; i<(BLU_SINGLE_TR_MESSAGE_SIZE-2); i++)
    {
            command[i] = 'B';
    }

    QByteArray Helper = QByteArray::fromRawData(command,BLU_SINGLE_TR_MESSAGE_SIZE -2);
    Helper.append("\n\r");
    BluInputDataProcessingWrapper.bleutoothClassicConnection.bluetoothSendDataToDevice(Helper);
}

/*********************************************************************************************************/

void MainWindow::on_ClearLoggerButton_clicked()
{
        ui->DebugDataTable->clearContents();
        ui->DebugDataTable->setRowCount(0);
}

void MainWindow::on_GeneraReplotAllPlots_pb_clicked()
{
//    PlotPosErr.Graph_UpdateReplot();
//    PlotPidRegVal.Graph_UpdateReplot();
//    PlotYawRate.Graph_UpdateReplot();
//    PlotSpd.Graph_UpdateReplot();
//    PlotMap.Graph_UpdateReplot();
//    PlotTrvDistance.Graph_UpdateReplot();
//    PlotOrientation.Graph_UpdateReplot();
//    PlotLinePosConfidence.Graph_UpdateReplot();
}

void MainWindow::on_SaveAppState_pb_clicked()
{
    QString filter = "LfProject (*.lfp) ;; All files (*)";
    QString desktopPath = QDir::toNativeSeparators(QStandardPaths::writableLocation(QStandardPaths::DesktopLocation));
    QString file_name = QFileDialog::getSaveFileName(this,"choose file to overwrite",desktopPath,filter);

    QFile saveFile(file_name);

    if (!saveFile.open(QIODevice::WriteOnly)) {
        qWarning("Couldn't open save file.");
        return;
    }

    QJsonObject object;

    //saveFile.write(QJsonDocument(object).toJson() );

    QString MessageBoxString = QString("Sucessfully saved \n NvM Data \n Plot Data \n Logger Table Data \n to file:\n %1").arg(file_name);
    QMessageBox::about(this,"Success!", MessageBoxString);
}

void MainWindow::LoadDataImuDataVisualiserProject(QString FilePath)
{
    QFile loadFile(FilePath);

    if (!loadFile.open(QIODevice::ReadOnly)) {
        qWarning("Couldn't open load file.");
        return;
    }

    QByteArray loadedData = loadFile.readAll();

    QJsonDocument loadDoc( QJsonDocument::fromJson(loadedData));
    /*
    .
    .
    .
    */

    NvM_DataLoadedFromExternalSourceFlag = true;
    on_GeneraReplotAllPlots_pb_clicked();
}





void MainWindow::on_LoadProject_pb_clicked()
{
    QString filter = "LfProject (*.lfp) ;; All files (*)";
    QString desktopPath = QDir::toNativeSeparators(QStandardPaths::writableLocation(QStandardPaths::DesktopLocation));
    QString file_name = QFileDialog::getOpenFileName(this,"choose file to overwrite",desktopPath,filter);
    LoadDataImuDataVisualiserProject(file_name);
}

void MainWindow::on_actionAbout_triggered()
{
    QString MessageBoxString = QString("This is Imu Visualiser application  \n"
                                       "\nAuthor: Teodor Rosolowski \n"
                                       "\nAppSite: github.com/trteodor \n"
                                       "\nCreated using QT Framework 6.5 and QT Creator 10.0.2"
                                       "\n"
                                       "\n Feel free to use the App in your project!\n"
                                       "\nIf you liked this application please let me star in a git repo\n"
                                       "\n"
                                       ""
                                       "");

    QMessageBox::about(this,"About Imu 3D Visualiser App", MessageBoxString);
}

