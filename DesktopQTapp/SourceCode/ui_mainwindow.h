/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 6.5.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QDockWidget>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenu>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QRadioButton>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QSplitter>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QTabWidget>
#include <QtWidgets/QTableWidget>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QWidget>
#include <qcustomplot.h>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QAction *actionHello;
    QAction *actionAbout;
    QWidget *centralwidget;
    QGridLayout *gridLayout_2;
    QHBoxLayout *horizontalLayout_3;
    QPushButton *GeneralPlotDataClear_pb;
    QPushButton *GeneraReplotAllPlots_pb;
    QPushButton *RemoveMarkers_pb;
    QPushButton *SaveAppState_pb;
    QPushButton *LoadProject_pb;
    QLabel *ClickedPointInfo_lb;
    QSplitter *PlotVerticalSplitter;
    QTabWidget *MainTabWidget;
    QWidget *tab_7;
    QWidget *tab_6;
    QGridLayout *gridLayout_6;
    QGridLayout *D3_LayoutTest;
    QWidget *MapViewTabViewOrg;
    QGridLayout *gridLayout_5;
    QCustomPlot *MapViewWidget;
    QSplitter *lPlotHorizontaSpliter;
    QTabWidget *PlotWidgetTab1;
    QWidget *tab_5;
    QCustomPlot *PlotAccRaw;
    QCustomPlot *PlotFildAcc;
    QCustomPlot *PlotEulerAg;
    QTabWidget *tabWidget_4;
    QWidget *tab_4;
    QCustomPlot *PlotGyro;
    QCustomPlot *PlotNormAcc;
    QCustomPlot *PlotAccJerk;
    QCustomPlot *PlotVel;
    QMenuBar *menubar;
    QMenu *menuFile;
    QMenu *menuAbout;
    QStatusBar *statusbar;
    QToolBar *toolBar;
    QToolBar *toolBar_2;
    QDockWidget *dockWidget_2;
    QWidget *dockWidgetContents_9;
    QGridLayout *gridLayout_3;
    QHBoxLayout *horizontalLayout_19;
    QLabel *BLU_StatusLabel;
    QSpacerItem *horizontalSpacer_4;
    QHBoxLayout *horizontalLayout_6;
    QPushButton *BLU_ConnectButton;
    QPushButton *BLU_DisconnectButton;
    QSpacerItem *verticalSpacer_2;
    QLabel *label;
    QHBoxLayout *horizontalLayout_7;
    QCheckBox *BLU_AutoConnCheckBox;
    QLineEdit *BLU_AutoConnDevNameL;
    QHBoxLayout *horizontalLayout_5;
    QPushButton *BLU_ScanButton;
    QComboBox *BLU_DetectedDeviceComboBox;
    QHBoxLayout *horizontalLayout_18;
    QHBoxLayout *horizontalLayout_11;
    QPushButton *BLU_SimulatorSuspendButton;
    QPushButton *BLU_TrueLogStartButton;
    QHBoxLayout *horizontalLayout_4;
    QDockWidget *dockWidget789;
    QWidget *dockWidgetContents_789;
    QGroupBox *groupBox;
    QGridLayout *Orientation3DLayout;
    QDockWidget *dockWidget_888;
    QWidget *dockWidgetContents_8;
    QGridLayout *gridLayout;
    QRadioButton *DebugTable_DisableBaseDataLogging;
    QPushButton *ClearLoggerButton;
    QPushButton *pushButton_8;
    QLabel *BaseDataLoggingInfoLabel;
    QPushButton *pushButton_7;
    QRadioButton *EnableBaseDataLogging;
    QTableWidget *DebugDataTable;
    QLabel *Label_CommunicationStat;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName("MainWindow");
        MainWindow->resize(1100, 750);
        QSizePolicy sizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
        sizePolicy.setHorizontalStretch(1);
        sizePolicy.setVerticalStretch(1);
        sizePolicy.setHeightForWidth(MainWindow->sizePolicy().hasHeightForWidth());
        MainWindow->setSizePolicy(sizePolicy);
        MainWindow->setMinimumSize(QSize(2, 200));
        MainWindow->setSizeIncrement(QSize(0, 0));
        MainWindow->setMouseTracking(false);
        MainWindow->setDocumentMode(false);
        MainWindow->setDockOptions(QMainWindow::AllowNestedDocks|QMainWindow::AllowTabbedDocks|QMainWindow::AnimatedDocks);
        actionHello = new QAction(MainWindow);
        actionHello->setObjectName("actionHello");
        actionAbout = new QAction(MainWindow);
        actionAbout->setObjectName("actionAbout");
        centralwidget = new QWidget(MainWindow);
        centralwidget->setObjectName("centralwidget");
        QSizePolicy sizePolicy1(QSizePolicy::Preferred, QSizePolicy::Preferred);
        sizePolicy1.setHorizontalStretch(0);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(centralwidget->sizePolicy().hasHeightForWidth());
        centralwidget->setSizePolicy(sizePolicy1);
        centralwidget->setMouseTracking(false);
        centralwidget->setAcceptDrops(false);
        gridLayout_2 = new QGridLayout(centralwidget);
        gridLayout_2->setObjectName("gridLayout_2");
        horizontalLayout_3 = new QHBoxLayout();
        horizontalLayout_3->setObjectName("horizontalLayout_3");
        GeneralPlotDataClear_pb = new QPushButton(centralwidget);
        GeneralPlotDataClear_pb->setObjectName("GeneralPlotDataClear_pb");

        horizontalLayout_3->addWidget(GeneralPlotDataClear_pb);

        GeneraReplotAllPlots_pb = new QPushButton(centralwidget);
        GeneraReplotAllPlots_pb->setObjectName("GeneraReplotAllPlots_pb");

        horizontalLayout_3->addWidget(GeneraReplotAllPlots_pb);

        RemoveMarkers_pb = new QPushButton(centralwidget);
        RemoveMarkers_pb->setObjectName("RemoveMarkers_pb");

        horizontalLayout_3->addWidget(RemoveMarkers_pb);

        SaveAppState_pb = new QPushButton(centralwidget);
        SaveAppState_pb->setObjectName("SaveAppState_pb");

        horizontalLayout_3->addWidget(SaveAppState_pb);

        LoadProject_pb = new QPushButton(centralwidget);
        LoadProject_pb->setObjectName("LoadProject_pb");

        horizontalLayout_3->addWidget(LoadProject_pb);


        gridLayout_2->addLayout(horizontalLayout_3, 2, 0, 1, 1);

        ClickedPointInfo_lb = new QLabel(centralwidget);
        ClickedPointInfo_lb->setObjectName("ClickedPointInfo_lb");

        gridLayout_2->addWidget(ClickedPointInfo_lb, 1, 0, 1, 1);

        PlotVerticalSplitter = new QSplitter(centralwidget);
        PlotVerticalSplitter->setObjectName("PlotVerticalSplitter");
        QSizePolicy sizePolicy2(QSizePolicy::Expanding, QSizePolicy::Expanding);
        sizePolicy2.setHorizontalStretch(0);
        sizePolicy2.setVerticalStretch(0);
        sizePolicy2.setHeightForWidth(PlotVerticalSplitter->sizePolicy().hasHeightForWidth());
        PlotVerticalSplitter->setSizePolicy(sizePolicy2);
        PlotVerticalSplitter->setOrientation(Qt::Vertical);
        PlotVerticalSplitter->setChildrenCollapsible(false);
        MainTabWidget = new QTabWidget(PlotVerticalSplitter);
        MainTabWidget->setObjectName("MainTabWidget");
        QSizePolicy sizePolicy3(QSizePolicy::Minimum, QSizePolicy::Minimum);
        sizePolicy3.setHorizontalStretch(0);
        sizePolicy3.setVerticalStretch(0);
        sizePolicy3.setHeightForWidth(MainTabWidget->sizePolicy().hasHeightForWidth());
        MainTabWidget->setSizePolicy(sizePolicy3);
        MainTabWidget->setMinimumSize(QSize(100, 0));
        tab_7 = new QWidget();
        tab_7->setObjectName("tab_7");
        MainTabWidget->addTab(tab_7, QString());
        tab_6 = new QWidget();
        tab_6->setObjectName("tab_6");
        gridLayout_6 = new QGridLayout(tab_6);
        gridLayout_6->setObjectName("gridLayout_6");
        D3_LayoutTest = new QGridLayout();
        D3_LayoutTest->setSpacing(0);
        D3_LayoutTest->setObjectName("D3_LayoutTest");
        D3_LayoutTest->setContentsMargins(-1, 20, -1, -1);

        gridLayout_6->addLayout(D3_LayoutTest, 0, 0, 1, 1);

        MainTabWidget->addTab(tab_6, QString());
        MapViewTabViewOrg = new QWidget();
        MapViewTabViewOrg->setObjectName("MapViewTabViewOrg");
        gridLayout_5 = new QGridLayout(MapViewTabViewOrg);
        gridLayout_5->setObjectName("gridLayout_5");
        MapViewWidget = new QCustomPlot(MapViewTabViewOrg);
        MapViewWidget->setObjectName("MapViewWidget");
        sizePolicy3.setHeightForWidth(MapViewWidget->sizePolicy().hasHeightForWidth());
        MapViewWidget->setSizePolicy(sizePolicy3);
        MapViewWidget->setMinimumSize(QSize(100, 0));
        MapViewWidget->setSizeIncrement(QSize(1, 1));

        gridLayout_5->addWidget(MapViewWidget, 0, 0, 1, 1);

        MainTabWidget->addTab(MapViewTabViewOrg, QString());
        PlotVerticalSplitter->addWidget(MainTabWidget);
        lPlotHorizontaSpliter = new QSplitter(PlotVerticalSplitter);
        lPlotHorizontaSpliter->setObjectName("lPlotHorizontaSpliter");
        lPlotHorizontaSpliter->setMinimumSize(QSize(0, 70));
        lPlotHorizontaSpliter->setOrientation(Qt::Horizontal);
        lPlotHorizontaSpliter->setChildrenCollapsible(false);
        PlotWidgetTab1 = new QTabWidget(lPlotHorizontaSpliter);
        PlotWidgetTab1->setObjectName("PlotWidgetTab1");
        sizePolicy3.setHeightForWidth(PlotWidgetTab1->sizePolicy().hasHeightForWidth());
        PlotWidgetTab1->setSizePolicy(sizePolicy3);
        PlotWidgetTab1->setMinimumSize(QSize(70, 70));
        tab_5 = new QWidget();
        tab_5->setObjectName("tab_5");
        PlotWidgetTab1->addTab(tab_5, QString());
        PlotAccRaw = new QCustomPlot();
        PlotAccRaw->setObjectName("PlotAccRaw");
        PlotWidgetTab1->addTab(PlotAccRaw, QString());
        PlotFildAcc = new QCustomPlot();
        PlotFildAcc->setObjectName("PlotFildAcc");
        PlotWidgetTab1->addTab(PlotFildAcc, QString());
        PlotEulerAg = new QCustomPlot();
        PlotEulerAg->setObjectName("PlotEulerAg");
        PlotWidgetTab1->addTab(PlotEulerAg, QString());
        lPlotHorizontaSpliter->addWidget(PlotWidgetTab1);
        tabWidget_4 = new QTabWidget(lPlotHorizontaSpliter);
        tabWidget_4->setObjectName("tabWidget_4");
        sizePolicy3.setHeightForWidth(tabWidget_4->sizePolicy().hasHeightForWidth());
        tabWidget_4->setSizePolicy(sizePolicy3);
        tabWidget_4->setMinimumSize(QSize(70, 70));
        tab_4 = new QWidget();
        tab_4->setObjectName("tab_4");
        tabWidget_4->addTab(tab_4, QString());
        PlotGyro = new QCustomPlot();
        PlotGyro->setObjectName("PlotGyro");
        tabWidget_4->addTab(PlotGyro, QString());
        PlotNormAcc = new QCustomPlot();
        PlotNormAcc->setObjectName("PlotNormAcc");
        tabWidget_4->addTab(PlotNormAcc, QString());
        PlotAccJerk = new QCustomPlot();
        PlotAccJerk->setObjectName("PlotAccJerk");
        tabWidget_4->addTab(PlotAccJerk, QString());
        PlotVel = new QCustomPlot();
        PlotVel->setObjectName("PlotVel");
        tabWidget_4->addTab(PlotVel, QString());
        lPlotHorizontaSpliter->addWidget(tabWidget_4);
        PlotVerticalSplitter->addWidget(lPlotHorizontaSpliter);

        gridLayout_2->addWidget(PlotVerticalSplitter, 0, 0, 1, 1);

        MainWindow->setCentralWidget(centralwidget);
        menubar = new QMenuBar(MainWindow);
        menubar->setObjectName("menubar");
        menubar->setGeometry(QRect(0, 0, 1100, 21));
        menuFile = new QMenu(menubar);
        menuFile->setObjectName("menuFile");
        menuAbout = new QMenu(menubar);
        menuAbout->setObjectName("menuAbout");
        MainWindow->setMenuBar(menubar);
        statusbar = new QStatusBar(MainWindow);
        statusbar->setObjectName("statusbar");
        MainWindow->setStatusBar(statusbar);
        toolBar = new QToolBar(MainWindow);
        toolBar->setObjectName("toolBar");
        MainWindow->addToolBar(Qt::TopToolBarArea, toolBar);
        toolBar_2 = new QToolBar(MainWindow);
        toolBar_2->setObjectName("toolBar_2");
        MainWindow->addToolBar(Qt::TopToolBarArea, toolBar_2);
        dockWidget_2 = new QDockWidget(MainWindow);
        dockWidget_2->setObjectName("dockWidget_2");
        sizePolicy3.setHeightForWidth(dockWidget_2->sizePolicy().hasHeightForWidth());
        dockWidget_2->setSizePolicy(sizePolicy3);
        dockWidget_2->setMinimumSize(QSize(195, 465));
        dockWidget_2->setMaximumSize(QSize(250, 524287));
        dockWidgetContents_9 = new QWidget();
        dockWidgetContents_9->setObjectName("dockWidgetContents_9");
        gridLayout_3 = new QGridLayout(dockWidgetContents_9);
        gridLayout_3->setObjectName("gridLayout_3");
        horizontalLayout_19 = new QHBoxLayout();
        horizontalLayout_19->setObjectName("horizontalLayout_19");
        BLU_StatusLabel = new QLabel(dockWidgetContents_9);
        BLU_StatusLabel->setObjectName("BLU_StatusLabel");
        QSizePolicy sizePolicy4(QSizePolicy::Expanding, QSizePolicy::Preferred);
        sizePolicy4.setHorizontalStretch(0);
        sizePolicy4.setVerticalStretch(0);
        sizePolicy4.setHeightForWidth(BLU_StatusLabel->sizePolicy().hasHeightForWidth());
        BLU_StatusLabel->setSizePolicy(sizePolicy4);

        horizontalLayout_19->addWidget(BLU_StatusLabel);

        horizontalSpacer_4 = new QSpacerItem(40, 20, QSizePolicy::Minimum, QSizePolicy::Minimum);

        horizontalLayout_19->addItem(horizontalSpacer_4);


        gridLayout_3->addLayout(horizontalLayout_19, 3, 0, 1, 1);

        horizontalLayout_6 = new QHBoxLayout();
        horizontalLayout_6->setSpacing(0);
        horizontalLayout_6->setObjectName("horizontalLayout_6");
        BLU_ConnectButton = new QPushButton(dockWidgetContents_9);
        BLU_ConnectButton->setObjectName("BLU_ConnectButton");

        horizontalLayout_6->addWidget(BLU_ConnectButton);

        BLU_DisconnectButton = new QPushButton(dockWidgetContents_9);
        BLU_DisconnectButton->setObjectName("BLU_DisconnectButton");

        horizontalLayout_6->addWidget(BLU_DisconnectButton);


        gridLayout_3->addLayout(horizontalLayout_6, 1, 0, 1, 1);

        verticalSpacer_2 = new QSpacerItem(20, 5, QSizePolicy::Minimum, QSizePolicy::Expanding);

        gridLayout_3->addItem(verticalSpacer_2, 8, 0, 1, 1);

        label = new QLabel(dockWidgetContents_9);
        label->setObjectName("label");

        gridLayout_3->addWidget(label, 6, 0, 1, 1);

        horizontalLayout_7 = new QHBoxLayout();
        horizontalLayout_7->setObjectName("horizontalLayout_7");
        BLU_AutoConnCheckBox = new QCheckBox(dockWidgetContents_9);
        BLU_AutoConnCheckBox->setObjectName("BLU_AutoConnCheckBox");
        BLU_AutoConnCheckBox->setChecked(true);

        horizontalLayout_7->addWidget(BLU_AutoConnCheckBox);

        BLU_AutoConnDevNameL = new QLineEdit(dockWidgetContents_9);
        BLU_AutoConnDevNameL->setObjectName("BLU_AutoConnDevNameL");
        QSizePolicy sizePolicy5(QSizePolicy::Expanding, QSizePolicy::Fixed);
        sizePolicy5.setHorizontalStretch(1);
        sizePolicy5.setVerticalStretch(1);
        sizePolicy5.setHeightForWidth(BLU_AutoConnDevNameL->sizePolicy().hasHeightForWidth());
        BLU_AutoConnDevNameL->setSizePolicy(sizePolicy5);
        BLU_AutoConnDevNameL->setSizeIncrement(QSize(1, 1));

        horizontalLayout_7->addWidget(BLU_AutoConnDevNameL);


        gridLayout_3->addLayout(horizontalLayout_7, 2, 0, 1, 1);

        horizontalLayout_5 = new QHBoxLayout();
        horizontalLayout_5->setObjectName("horizontalLayout_5");
        BLU_ScanButton = new QPushButton(dockWidgetContents_9);
        BLU_ScanButton->setObjectName("BLU_ScanButton");
        BLU_ScanButton->setMaximumSize(QSize(60, 16777215));

        horizontalLayout_5->addWidget(BLU_ScanButton);

        BLU_DetectedDeviceComboBox = new QComboBox(dockWidgetContents_9);
        BLU_DetectedDeviceComboBox->setObjectName("BLU_DetectedDeviceComboBox");
        BLU_DetectedDeviceComboBox->setEnabled(true);
        QSizePolicy sizePolicy6(QSizePolicy::Expanding, QSizePolicy::Maximum);
        sizePolicy6.setHorizontalStretch(1);
        sizePolicy6.setVerticalStretch(1);
        sizePolicy6.setHeightForWidth(BLU_DetectedDeviceComboBox->sizePolicy().hasHeightForWidth());
        BLU_DetectedDeviceComboBox->setSizePolicy(sizePolicy6);
        BLU_DetectedDeviceComboBox->setMinimumSize(QSize(70, 0));
        BLU_DetectedDeviceComboBox->setSizeIncrement(QSize(1, 1));

        horizontalLayout_5->addWidget(BLU_DetectedDeviceComboBox);


        gridLayout_3->addLayout(horizontalLayout_5, 0, 0, 1, 1);

        horizontalLayout_18 = new QHBoxLayout();
        horizontalLayout_18->setObjectName("horizontalLayout_18");

        gridLayout_3->addLayout(horizontalLayout_18, 4, 0, 1, 1);

        horizontalLayout_11 = new QHBoxLayout();
        horizontalLayout_11->setSpacing(0);
        horizontalLayout_11->setObjectName("horizontalLayout_11");
        BLU_SimulatorSuspendButton = new QPushButton(dockWidgetContents_9);
        BLU_SimulatorSuspendButton->setObjectName("BLU_SimulatorSuspendButton");
        BLU_SimulatorSuspendButton->setMaximumSize(QSize(50, 16777215));

        horizontalLayout_11->addWidget(BLU_SimulatorSuspendButton);

        BLU_TrueLogStartButton = new QPushButton(dockWidgetContents_9);
        BLU_TrueLogStartButton->setObjectName("BLU_TrueLogStartButton");
        BLU_TrueLogStartButton->setMaximumSize(QSize(50, 16777215));

        horizontalLayout_11->addWidget(BLU_TrueLogStartButton);


        gridLayout_3->addLayout(horizontalLayout_11, 7, 0, 1, 1);

        horizontalLayout_4 = new QHBoxLayout();
        horizontalLayout_4->setSpacing(15);
        horizontalLayout_4->setObjectName("horizontalLayout_4");

        gridLayout_3->addLayout(horizontalLayout_4, 5, 0, 1, 1);

        dockWidget_2->setWidget(dockWidgetContents_9);
        MainWindow->addDockWidget(Qt::LeftDockWidgetArea, dockWidget_2);
        dockWidget789 = new QDockWidget(MainWindow);
        dockWidget789->setObjectName("dockWidget789");
        QSizePolicy sizePolicy7(QSizePolicy::Minimum, QSizePolicy::Minimum);
        sizePolicy7.setHorizontalStretch(1);
        sizePolicy7.setVerticalStretch(0);
        sizePolicy7.setHeightForWidth(dockWidget789->sizePolicy().hasHeightForWidth());
        dockWidget789->setSizePolicy(sizePolicy7);
        dockWidget789->setMinimumSize(QSize(200, 400));
        dockWidget789->setMaximumSize(QSize(524287, 300));
        dockWidget789->setSizeIncrement(QSize(0, 0));
        dockWidget789->setBaseSize(QSize(0, 0));
        dockWidget789->setFloating(false);
        dockWidget789->setFeatures(QDockWidget::DockWidgetFloatable|QDockWidget::DockWidgetMovable);
        dockWidget789->setAllowedAreas(Qt::AllDockWidgetAreas);
        dockWidgetContents_789 = new QWidget();
        dockWidgetContents_789->setObjectName("dockWidgetContents_789");
        groupBox = new QGroupBox(dockWidgetContents_789);
        groupBox->setObjectName("groupBox");
        groupBox->setGeometry(QRect(30, 0, 381, 371));
        Orientation3DLayout = new QGridLayout(groupBox);
        Orientation3DLayout->setSpacing(6);
        Orientation3DLayout->setObjectName("Orientation3DLayout");
        Orientation3DLayout->setContentsMargins(9, 9, 9, 9);
        dockWidget789->setWidget(dockWidgetContents_789);
        MainWindow->addDockWidget(Qt::RightDockWidgetArea, dockWidget789);
        dockWidget_888 = new QDockWidget(MainWindow);
        dockWidget_888->setObjectName("dockWidget_888");
        QSizePolicy sizePolicy8(QSizePolicy::Maximum, QSizePolicy::Maximum);
        sizePolicy8.setHorizontalStretch(1);
        sizePolicy8.setVerticalStretch(1);
        sizePolicy8.setHeightForWidth(dockWidget_888->sizePolicy().hasHeightForWidth());
        dockWidget_888->setSizePolicy(sizePolicy8);
        dockWidget_888->setMinimumSize(QSize(500, 167));
        dockWidget_888->setMaximumSize(QSize(1000, 524287));
        dockWidget_888->setSizeIncrement(QSize(1, 1));
        dockWidget_888->setBaseSize(QSize(0, 0));
        dockWidget_888->setFeatures(QDockWidget::DockWidgetFloatable|QDockWidget::DockWidgetMovable);
        dockWidget_888->setAllowedAreas(Qt::AllDockWidgetAreas);
        dockWidgetContents_8 = new QWidget();
        dockWidgetContents_8->setObjectName("dockWidgetContents_8");
        gridLayout = new QGridLayout(dockWidgetContents_8);
        gridLayout->setObjectName("gridLayout");
        DebugTable_DisableBaseDataLogging = new QRadioButton(dockWidgetContents_8);
        DebugTable_DisableBaseDataLogging->setObjectName("DebugTable_DisableBaseDataLogging");
        DebugTable_DisableBaseDataLogging->setChecked(true);

        gridLayout->addWidget(DebugTable_DisableBaseDataLogging, 1, 2, 1, 1);

        ClearLoggerButton = new QPushButton(dockWidgetContents_8);
        ClearLoggerButton->setObjectName("ClearLoggerButton");

        gridLayout->addWidget(ClearLoggerButton, 0, 0, 1, 1);

        pushButton_8 = new QPushButton(dockWidgetContents_8);
        pushButton_8->setObjectName("pushButton_8");

        gridLayout->addWidget(pushButton_8, 0, 1, 1, 1);

        BaseDataLoggingInfoLabel = new QLabel(dockWidgetContents_8);
        BaseDataLoggingInfoLabel->setObjectName("BaseDataLoggingInfoLabel");

        gridLayout->addWidget(BaseDataLoggingInfoLabel, 1, 0, 1, 1);

        pushButton_7 = new QPushButton(dockWidgetContents_8);
        pushButton_7->setObjectName("pushButton_7");

        gridLayout->addWidget(pushButton_7, 0, 2, 1, 1);

        EnableBaseDataLogging = new QRadioButton(dockWidgetContents_8);
        EnableBaseDataLogging->setObjectName("EnableBaseDataLogging");
        EnableBaseDataLogging->setChecked(false);

        gridLayout->addWidget(EnableBaseDataLogging, 1, 1, 1, 1);

        DebugDataTable = new QTableWidget(dockWidgetContents_8);
        if (DebugDataTable->columnCount() < 5)
            DebugDataTable->setColumnCount(5);
        QTableWidgetItem *__qtablewidgetitem = new QTableWidgetItem();
        DebugDataTable->setHorizontalHeaderItem(0, __qtablewidgetitem);
        QTableWidgetItem *__qtablewidgetitem1 = new QTableWidgetItem();
        DebugDataTable->setHorizontalHeaderItem(1, __qtablewidgetitem1);
        QTableWidgetItem *__qtablewidgetitem2 = new QTableWidgetItem();
        DebugDataTable->setHorizontalHeaderItem(2, __qtablewidgetitem2);
        QTableWidgetItem *__qtablewidgetitem3 = new QTableWidgetItem();
        DebugDataTable->setHorizontalHeaderItem(3, __qtablewidgetitem3);
        QTableWidgetItem *__qtablewidgetitem4 = new QTableWidgetItem();
        DebugDataTable->setHorizontalHeaderItem(4, __qtablewidgetitem4);
        DebugDataTable->setObjectName("DebugDataTable");
        QSizePolicy sizePolicy9(QSizePolicy::Expanding, QSizePolicy::Expanding);
        sizePolicy9.setHorizontalStretch(1);
        sizePolicy9.setVerticalStretch(1);
        sizePolicy9.setHeightForWidth(DebugDataTable->sizePolicy().hasHeightForWidth());
        DebugDataTable->setSizePolicy(sizePolicy9);
        DebugDataTable->setMinimumSize(QSize(100, 50));
        DebugDataTable->setMaximumSize(QSize(16777215, 16777215));
        DebugDataTable->setSizeIncrement(QSize(1, 1));
        DebugDataTable->setBaseSize(QSize(600, 400));

        gridLayout->addWidget(DebugDataTable, 2, 0, 1, 3);

        Label_CommunicationStat = new QLabel(dockWidgetContents_8);
        Label_CommunicationStat->setObjectName("Label_CommunicationStat");

        gridLayout->addWidget(Label_CommunicationStat, 3, 0, 1, 3);

        dockWidget_888->setWidget(dockWidgetContents_8);
        MainWindow->addDockWidget(Qt::RightDockWidgetArea, dockWidget_888);
        dockWidget789->raise();
        dockWidget_888->raise();
        QWidget::setTabOrder(DebugDataTable, GeneraReplotAllPlots_pb);
        QWidget::setTabOrder(GeneraReplotAllPlots_pb, SaveAppState_pb);
        QWidget::setTabOrder(SaveAppState_pb, GeneralPlotDataClear_pb);
        QWidget::setTabOrder(GeneralPlotDataClear_pb, MainTabWidget);
        QWidget::setTabOrder(MainTabWidget, BLU_DetectedDeviceComboBox);
        QWidget::setTabOrder(BLU_DetectedDeviceComboBox, LoadProject_pb);
        QWidget::setTabOrder(LoadProject_pb, PlotWidgetTab1);
        QWidget::setTabOrder(PlotWidgetTab1, tabWidget_4);

        menubar->addAction(menuFile->menuAction());
        menubar->addAction(menuAbout->menuAction());
        menuFile->addAction(actionHello);
        menuAbout->addAction(actionAbout);
        toolBar->addAction(actionHello);
        toolBar->addAction(actionAbout);

        retranslateUi(MainWindow);

        MainTabWidget->setCurrentIndex(0);
        PlotWidgetTab1->setCurrentIndex(0);
        tabWidget_4->setCurrentIndex(0);


        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QCoreApplication::translate("MainWindow", "MainWindow", nullptr));
        actionHello->setText(QCoreApplication::translate("MainWindow", "Hello", nullptr));
        actionAbout->setText(QCoreApplication::translate("MainWindow", "About", nullptr));
        GeneralPlotDataClear_pb->setText(QCoreApplication::translate("MainWindow", "ClearPlotData", nullptr));
        GeneraReplotAllPlots_pb->setText(QCoreApplication::translate("MainWindow", "ReplotAll", nullptr));
        RemoveMarkers_pb->setText(QCoreApplication::translate("MainWindow", "RemoveMarkers", nullptr));
        SaveAppState_pb->setText(QCoreApplication::translate("MainWindow", "SaveAs..", nullptr));
        LoadProject_pb->setText(QCoreApplication::translate("MainWindow", "Load", nullptr));
        ClickedPointInfo_lb->setText(QCoreApplication::translate("MainWindow", "Clicked at point: ? (Please click at graph to show informations about cliked point)", nullptr));
        MainTabWidget->setTabText(MainTabWidget->indexOf(tab_7), QCoreApplication::translate("MainWindow", "nA", nullptr));
        MainTabWidget->setTabText(MainTabWidget->indexOf(tab_6), QCoreApplication::translate("MainWindow", "Imu3D", nullptr));
        MainTabWidget->setTabText(MainTabWidget->indexOf(MapViewTabViewOrg), QCoreApplication::translate("MainWindow", "r1", nullptr));
        PlotWidgetTab1->setTabText(PlotWidgetTab1->indexOf(tab_5), QCoreApplication::translate("MainWindow", "nA", nullptr));
        PlotWidgetTab1->setTabText(PlotWidgetTab1->indexOf(PlotAccRaw), QCoreApplication::translate("MainWindow", "acc", nullptr));
        PlotWidgetTab1->setTabText(PlotWidgetTab1->indexOf(PlotFildAcc), QCoreApplication::translate("MainWindow", "fAcc", nullptr));
        PlotWidgetTab1->setTabText(PlotWidgetTab1->indexOf(PlotEulerAg), QCoreApplication::translate("MainWindow", "eulerAg", nullptr));
        tabWidget_4->setTabText(tabWidget_4->indexOf(tab_4), QCoreApplication::translate("MainWindow", "nA", nullptr));
        tabWidget_4->setTabText(tabWidget_4->indexOf(PlotGyro), QCoreApplication::translate("MainWindow", "Gyro", nullptr));
        tabWidget_4->setTabText(tabWidget_4->indexOf(PlotNormAcc), QCoreApplication::translate("MainWindow", "nAcc", nullptr));
        tabWidget_4->setTabText(tabWidget_4->indexOf(PlotAccJerk), QCoreApplication::translate("MainWindow", "jerk", nullptr));
        tabWidget_4->setTabText(tabWidget_4->indexOf(PlotVel), QCoreApplication::translate("MainWindow", "vel", nullptr));
        menuFile->setTitle(QCoreApplication::translate("MainWindow", "File", nullptr));
        menuAbout->setTitle(QCoreApplication::translate("MainWindow", "About", nullptr));
        toolBar->setWindowTitle(QCoreApplication::translate("MainWindow", "toolBar", nullptr));
        toolBar_2->setWindowTitle(QCoreApplication::translate("MainWindow", "toolBar_2", nullptr));
        dockWidget_2->setWindowTitle(QCoreApplication::translate("MainWindow", "HeyThisIsCrazyWorld", nullptr));
        BLU_StatusLabel->setText(QCoreApplication::translate("MainWindow", "Status:", nullptr));
        BLU_ConnectButton->setText(QCoreApplication::translate("MainWindow", "Connect", nullptr));
        BLU_DisconnectButton->setText(QCoreApplication::translate("MainWindow", "Disconnect", nullptr));
        label->setText(QCoreApplication::translate("MainWindow", "DataLogging:", nullptr));
        BLU_AutoConnCheckBox->setText(QCoreApplication::translate("MainWindow", "LookFor:", nullptr));
        BLU_AutoConnDevNameL->setText(QCoreApplication::translate("MainWindow", "ANDRZEJ", nullptr));
        BLU_ScanButton->setText(QCoreApplication::translate("MainWindow", "Scan BLU", nullptr));
        BLU_SimulatorSuspendButton->setText(QCoreApplication::translate("MainWindow", "Suspend", nullptr));
        BLU_TrueLogStartButton->setText(QCoreApplication::translate("MainWindow", "Activate", nullptr));
        dockWidget789->setWindowTitle(QCoreApplication::translate("MainWindow", "ManualMoveDockWindow", nullptr));
        groupBox->setTitle(QCoreApplication::translate("MainWindow", "Orientation3D", nullptr));
        dockWidget_888->setWindowTitle(QCoreApplication::translate("MainWindow", "Logger", nullptr));
        DebugTable_DisableBaseDataLogging->setText(QCoreApplication::translate("MainWindow", "Disable", nullptr));
        ClearLoggerButton->setText(QCoreApplication::translate("MainWindow", "ClearLogger", nullptr));
        pushButton_8->setText(QCoreApplication::translate("MainWindow", "PushButton", nullptr));
        BaseDataLoggingInfoLabel->setText(QCoreApplication::translate("MainWindow", "PlotDataLogging:", nullptr));
        pushButton_7->setText(QCoreApplication::translate("MainWindow", "PushButton", nullptr));
        EnableBaseDataLogging->setText(QCoreApplication::translate("MainWindow", "Enable", nullptr));
        QTableWidgetItem *___qtablewidgetitem = DebugDataTable->horizontalHeaderItem(0);
        ___qtablewidgetitem->setText(QCoreApplication::translate("MainWindow", "SysTim", nullptr));
        QTableWidgetItem *___qtablewidgetitem1 = DebugDataTable->horizontalHeaderItem(1);
        ___qtablewidgetitem1->setText(QCoreApplication::translate("MainWindow", "ucTime", nullptr));
        QTableWidgetItem *___qtablewidgetitem2 = DebugDataTable->horizontalHeaderItem(2);
        ___qtablewidgetitem2->setText(QCoreApplication::translate("MainWindow", "FrmNum", nullptr));
        QTableWidgetItem *___qtablewidgetitem3 = DebugDataTable->horizontalHeaderItem(3);
        ___qtablewidgetitem3->setText(QCoreApplication::translate("MainWindow", "ScID", nullptr));
        QTableWidgetItem *___qtablewidgetitem4 = DebugDataTable->horizontalHeaderItem(4);
        ___qtablewidgetitem4->setText(QCoreApplication::translate("MainWindow", "Decoded Data", nullptr));
        Label_CommunicationStat->setText(QCoreApplication::translate("MainWindow", "CommunicationStatistics", nullptr));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
