QT += core
QT -= gui

TARGET = nuagePoints
CONFIG += console
CONFIG -= app_bundle

TEMPLATE = app

INCLUDEPATH += /usr/local/include/opencv
LIBS += -L/usr/local/lib -lopencv_core  -lopencv_highgui -lopencv_photo -lopencv_video -lopencv_imgproc -lopencv_calib3d

SOURCES += main.cpp

HEADERS +=

