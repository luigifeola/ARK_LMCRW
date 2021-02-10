#-------------------------------------------------
#
# Project created by QtCreator 2018-01-01T07:35:48
#
#-------------------------------------------------

QT       -= gui

QT += widgets network

TARGET = LMCRW
TEMPLATE = lib

DEFINES += LMCRWEXP_LIBRARY

SOURCES += \
    kilobot.cpp \
    LMCRWEnvironment.cpp \
    LMCRWExperiment.cpp

HEADERS +=\
    kilobot.h \
    kilobotexperiment.h \
    kilobotenvironment.h \
    global.h \
    LMCRWEnvironment.h \
    LMCRWExperiment.h

unix {
    target.path = /usr/lib
    INSTALLS += target
}

INCLUDEPATH += /usr/local/include/
LIBS += -L/usr/local/lib \
        -lopencv_core

