TEMPLATE = app
CONFIG += console
CONFIG -= qt



SOURCES += \
    ParticleStatsTest.cpp \
    ParticleStats.cpp

HEADERS += \
    ParticleStats.h

INCLUDEPATH += /usr/local/include/opencv2

LIBS += -LC:/usr/local/lib/
LIBS += -lopencv_core
LIBS += -lopencv_highgui
LIBS += -lopencv_imgproc
LIBS += -lopencv_features2d
LIBS += -lopencv_calib3d
