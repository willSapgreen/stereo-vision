#-------------------------------------------------
#
# Project created by QtCreator 2010-11-24T14:01:01
#
#-------------------------------------------------

QT += core gui opengl

TARGET = stereomapper
TEMPLATE = app

SOURCES += main.cpp\
        maindialog.cpp \
    selectcamerasdialog.cpp \
    framecapturethread.cpp \
    stereoimage.cpp \
    view2d.cpp \
    calibio.cpp \
    view3d.cpp \
    visualodometrythread.cpp \
    stereothread.cpp \
    ../libelas/src/elas.cpp \
    ../libelas/src/descriptor.cpp \
    savestereoimagethread.cpp \
    readfromfilesthread.cpp \
    visualizethread.cpp \
    planeestimation.cpp \
    ../libviso2/src/viso.cpp \
    ../libviso2/src/triangle.cpp \
    ../libviso2/src/matrix.cpp \
    ../libviso2/src/filter.cpp \
    ../libviso2/src/matcher.cpp \
    ../libviso2/src/viso_stereo.cpp

HEADERS  += maindialog.h \
    selectcamerasdialog.h \
    framecapturethread.h \
    stereoimage.h \
    view2d.h \
    calibio.h \
    view3d.h \
    visualodometrythread.h \
    stereothread.h \
    ../libelas/src/elas.h \
    ../libelas/src/descriptor.h \
    savestereoimagethread.h \
    readfromfilesthread.h \
    visualizethread.h\
    planeestimation.h \
    ../libviso2/src/viso.h \
    ../libviso2/src/triangle.h \
    ../libviso2/src/timer.h \
    ../libviso2/src/matrix.h \
    ../libviso2/src/matcher.h \
    ../libviso2/src/filter.h \
    ../libviso2/src/viso_stereo.h

FORMS    += maindialog.ui \
    selectcamerasdialog.ui

LIBS += -ldc1394 -lGLU -L/usr/local/lib \
                 -lopencv_core\
                 -lopencv_highgui\
                 -lopencv_imgproc\
                 -lopencv_imgcodecs\ #-lcxcore -lcv -lhighgui

QMAKE_CXXFLAGS += -O3 -pipe -fomit-frame-pointer -msse3
