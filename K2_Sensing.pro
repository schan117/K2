#-------------------------------------------------
#
# Project created by QtCreator 2014-10-22T16:43:35
#
#-------------------------------------------------

QT       += core gui opengl testlib

CONFIG += console

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = K2_Sensing
TEMPLATE = app

INCLUDEPATH += "C:\v2.0-PublicPreview1409\inc" "C:\opencv\build\include" "C:\Program Files\PCL\include\pcl-1.8" \
               "C:\Program Files (x86)\Eigen\include" "C:\boost\boost_1_56_0" "C:\flann-1.8.4-src\flann-1.8.4-src\src\cpp"

QMAKE_LIBDIR += "C:\v2.0-PublicPreview1409\Lib\x64" "C:\opencv\build\x64\vc11\lib" "C:\Program Files\PCL\lib" \
                "C:\boost\boost_1_56_0\lib64-msvc-12.0" "C:\flann-1.8.4-src\flann-1.8.4-src\build\lib\Release" \



LIBS += Kinect20.lib opencv_core246.lib opencv_imgproc246.lib opencv_highgui246.lib \
        pcl_common_release.lib pcl_io_release.lib pcl_filters_release.lib pcl_kdtree_release.lib pcl_search_release.lib \
        pcl_surface_release.lib \


SOURCES += main.cpp\
        mainwindow.cpp \
    kinect_wrapper.cpp \
    capture_thread.cpp \
    GLWidget.cpp \
    support_math.cpp \
    cloud_processor.cpp

HEADERS  += mainwindow.h \
    kinect_wrapper.h \
    Definitions.h \
    capture_thread.h \
    GLWidget.h \
    support_math.h \
    cloud_processor.h

FORMS    += mainwindow.ui
