#include "capture_thread.h"
#include <QTest>

Capture_Thread::Capture_Thread(QObject *parent) :
    QThread(parent)
{
    keep_capturing = true;
}

void Capture_Thread::Initialize_Capture_Thread(Kinect_Wrapper *kw)
{
    kinect_wrapper = kw;
}

void Capture_Thread::run()
{
    while (keep_capturing)
    {
        mutex.lock();

        if ( (kinect_wrapper->Trigger_Color_Image(current_camera_index)) &&
             (kinect_wrapper->Trigger_Depth_Image(current_camera_index, depth_min, depth_max)) )
        {
            emit Color_And_Depth_Frame_Captured(current_camera_index);
        }

       // QTest::qWait(30);
    }
}
