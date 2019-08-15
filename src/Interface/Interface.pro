#-------------------------------------------------
#
# Project created by QtCreator 2013-04-04T22:59:26
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = Interface
TEMPLATE = app

INCLUDEPATH += /usr/local/include/opencv2

SOURCES += main.cpp\
        mainwindow.cpp

HEADERS  += mainwindow.h \
    ../Drawer/Drawer.h \
    ../Distributions/ParticleStats.h \
    ../Distributions/Distributions.h

FORMS    += mainwindow.ui

LIBS += -LC:/usr/local/lib/
LIBS += -lopencv_core
LIBS += -lopencv_highgui
LIBS += -lopencv_imgproc
LIBS += -lopencv_features2d
LIBS += -lopencv_calib3d
