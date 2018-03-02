QT += core

QT -= gui

TARGET = captureV
CONFIG += console
CONFIG -= app_bundle
CONFIG(debug, release|debug):DEFINES += _DEBUG

TEMPLATE = app

INCLUDEPATH += /usr/local/include/opencv
LIBS += -L/usr/local/lib -lopencv_core  -lopencv_highgui -lopencv_photo -lopencv_video -lopencv_imgproc -lopencv_calib3d -lgmp



#-lopencv_imgcodecs  -lopencv_videoio


SOURCES += main.cpp \
    triangulation.cpp \
    leopard.cpp

HEADERS += \
    leopard.hpp \
    triangulation.hpp \
    paths.hpp
