TEMPLATE = app
TARGET   = LoboVoxelizor

HEADERS  = geometry.h \
    lobovoxelizor.h \
    mainviewer.h \
    voxelmesh.h \
    zVector.h \
    voxelmeshspeed.h

SOURCES  = trianglemesh.cpp main.cpp \
    lobovoxelizor.cpp \
    voxelmesh.cpp \
    mainviewer.cpp \
    voxelmeshspeed.cpp

FORMS += \
    lobovoxelizor.ui

QMAKE_CXXFLAGS += -std=c++11 -Wall

QT += opengl xml widgets

# For Windows ====================================================================
INCLUDEPATH += F:/MYLIBs/libQGLViewer-2.6.3
LIBS += -lopengl32

CONFIG(debug,debug|release) {
    LIBS += -LF:/MYLIBs/libQGLViewer-2.6.3/QGLViewer -lQGLViewerd2
}else{
    LIBS += -LF:/MYLIBs/libQGLViewer-2.6.3/QGLViewer -lQGLViewer2
}

# For Linux ====================================================================
#INCLUDEPATH += /home/yumingzhang/programfile/eigen-3.2.5/Eigen
#INCLUDEPATH += /home/yumingzhang/programfile/libQGLViewer-2.6.2/QGLViewer
#LIBS += -lQGLViewer
