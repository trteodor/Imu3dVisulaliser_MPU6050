QT       += core gui bluetooth concurrent quick widgets datavisualization

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets printsupport

CONFIG += c++17
#CONFIG += no_keywords

# You can make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
    GenericQCP.cpp \
    bluetoothDataManager.cpp \
    bluetoothclassic.cpp \
    main.cpp \
    mainwindow.cpp \
    deviceinfo.cpp \
    qcustomplot.cpp \
    scatterdatamodifier.cpp

HEADERS += \
    GenericQCP.h \
    bluetoothDataManager.h \
    bluetoothclassic.h \
    mainwindow.h \
    deviceinfo.h \
    matplotlibcpp.h \
    qcustomplot.h \
    scatterdatamodifier.h

FORMS += \
    mainwindow.ui

TRANSLATIONS += \
    ImuVisualiser_en_150.ts
CONFIG += lrelease
CONFIG += embed_translations

RESOURCES += qdarkstyle/dark/darkstyle.qrc

RESOURCES += RobotOriRes.qrc
RESOURCES += qdarkstyle/light/lightstyle.qrc


RESOURCES += CubeDiffFacesQML/resCubeDiff.qrc

RC_ICONS = ImuIconn.ico

OTHER_FILES += \
    CubeDiffFacesQML/Dice1.svg \
    CubeDiffFacesQML/Dice2.svg \
    CubeDiffFacesQML/Dice3.svg \
    CubeDiffFacesQML/Dice4.svg \
    CubeDiffFacesQML/Dice5.svg \
    CubeDiffFacesQML/Dice6.svg \
    CubeDiffFacesQML/CubeDiffFaces.qml

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target
