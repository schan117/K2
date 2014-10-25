#ifndef CAPTURE_THREAD_H
#define CAPTURE_THREAD_H

#include <QThread>
#include "kinect_wrapper.h"
#include <QMutex>

class Capture_Thread : public QThread
{
    Q_OBJECT

public:
    explicit Capture_Thread(QObject *parent = 0);
    void Initialize_Capture_Thread(Kinect_Wrapper* kw);

    void run();

    bool keep_capturing;
    QMutex mutex;

    int current_camera_index;
    int depth_min;
    int depth_max;


signals:

    void Color_And_Depth_Frame_Captured(int index);

public slots:

private:

    Kinect_Wrapper* kinect_wrapper;



};

#endif // CAPTURE_THREAD_H
