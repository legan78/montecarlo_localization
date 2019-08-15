TEMPLATE = app
CONFIG += console
CONFIG -= qt



SOURCES += main.cpp \
    SqrdIntensityDiff.cpp \
    MeanValDistance.cpp \
    HistDiff.cpp

HEADERS += \
    SqrdIntensityDiff.h \
    MeanValDistance.h \
    Likelihood.h \
    HistDiff.h

INCLUDEPATH += /usr/local/include/opencv2

LIBS += -LC:/usr/local/lib/
LIBS += -lopencv_core
LIBS += -lopencv_highgui
LIBS += -lopencv_imgproc
LIBS += -lopencv_features2d
LIBS += -lopencv_calib3d
