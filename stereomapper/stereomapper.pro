#-------------------------------------------------
#
# Project created by QtCreator 2010-11-24T14:01:01
#
#-------------------------------------------------

QT += core gui opengl

TARGET = stereomapper
TEMPLATE = app

INCLUDEPATH += "../gpxlib"

SOURCES += main.cpp\
        maindialog.cpp \
    #selectcamerasdialog.cpp \
    #framecapturethread.cpp \
    stereoimage.cpp \
    view2d.cpp \
    #calibio.cpp \
    calibiokitti.cpp \
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
    ../libviso2/src/viso_stereo.cpp \
    ../gpxlib/gpx/Bounds.cpp \
    ../gpxlib/gpx/Copyright.cpp \
    ../gpxlib/gpx/DateTime.cpp \
    ../gpxlib/gpx/Decimal.cpp \
    ../gpxlib/gpx/Degrees.cpp \
    ../gpxlib/gpx/DGPSStation.cpp \
    ../gpxlib/gpx/EMail.cpp \
    ../gpxlib/gpx/Extensions.cpp \
    ../gpxlib/gpx/Fix.cpp \
    ../gpxlib/gpx/GPX.cpp \
    ../gpxlib/gpx/Latitude.cpp \
    ../gpxlib/gpx/Link.cpp \
    ../gpxlib/gpx/Longitude.cpp \
    ../gpxlib/gpx/Metadata.cpp \
    ../gpxlib/gpx/Node.cpp \
    ../gpxlib/gpx/Parser.cpp \
    ../gpxlib/gpx/Person.cpp \
    ../gpxlib/gpx/PT.cpp \
    ../gpxlib/gpx/PTSeg.cpp \
    ../gpxlib/gpx/Report.cpp \
    ../gpxlib/gpx/ReportCerr.cpp \
    ../gpxlib/gpx/RTE.cpp \
    ../gpxlib/gpx/String.cpp \
    ../gpxlib/gpx/TRK.cpp \
    ../gpxlib/gpx/TRKSeg.cpp \
    ../gpxlib/gpx/Unsigned.cpp \
    ../gpxlib/gpx/URI.cpp \
    ../gpxlib/gpx/WPT.cpp \
    ../gpxlib/gpx/Writer.cpp \
    gpxgenerator.cpp

HEADERS  += maindialog.h \
    #selectcamerasdialog.h \
    #framecapturethread.h \
    stereoimage.h \
    view2d.h \
    #calibio.h \
    calibiokitti.h \
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
    ../libviso2/src/viso_stereo.h \
    ../gpxlib/gpx/Bounds.h \
    ../gpxlib/gpx/Copyright.h \
    ../gpxlib/gpx/DateTime.h \
    ../gpxlib/gpx/Decimal.h \
    ../gpxlib/gpx/Degrees.h \
    ../gpxlib/gpx/DGPSStation.h \
    ../gpxlib/gpx/EMail.h \
    ../gpxlib/gpx/Extensions.h \
    ../gpxlib/gpx/Fix.h \
    ../gpxlib/gpx/GPX.h \
    ../gpxlib/gpx/Latitude.h \
    ../gpxlib/gpx/Link.h \
    ../gpxlib/gpx/List.h \
    ../gpxlib/gpx/Longitude.h \
    ../gpxlib/gpx/Metadata.h \
    ../gpxlib/gpx/Node.h \
    ../gpxlib/gpx/Parser.h \
    ../gpxlib/gpx/Person.h \
    ../gpxlib/gpx/PT.h \
    ../gpxlib/gpx/PTSeg.h \
    ../gpxlib/gpx/Report.h \
    ../gpxlib/gpx/ReportCerr.h \
    ../gpxlib/gpx/RTE.h \
    ../gpxlib/gpx/String_.h \
    ../gpxlib/gpx/TRK.h \
    ../gpxlib/gpx/TRKSeg.h \
    ../gpxlib/gpx/Unsigned.h \
    ../gpxlib/gpx/URI.h \
    ../gpxlib/gpx/WPT.h \
    ../gpxlib/gpx/Writer.h \
    gpxgenerator.h

FORMS    += maindialog.ui \
    selectcamerasdialog.ui

LIBS += -ldc1394 -lGLU -L/usr/local/lib \
                 -lopencv_core\
                 -lopencv_highgui\
                 -lopencv_imgproc\
                 -lopencv_imgcodecs\ #-lcxcore -lcv -lhighgui
        -L"/usr/lib/x85_64-linux-gnu" -lexpat

QMAKE_CXXFLAGS += -O3 -pipe -fomit-frame-pointer -msse3
