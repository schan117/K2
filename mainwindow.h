#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "kinect_wrapper.h"
#include "capture_thread.h"
#include "GLWidget.h"
#include "cloud_processor.h"

using namespace pcl;

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

    void Connect_Signals();

private slots:


    void On_Color_And_Depth_Frame_Captured(int index);

    void on_pushButton_clicked();

    void on_Min_Distance_valueChanged(int arg1);

    void on_Max_Distance_valueChanged(int arg1);

    void on_Capture_clicked();

private:
    Ui::MainWindow *ui;

    Kinect_Wrapper kinect_wrapper;

    Capture_Thread capture_thread;

    Cloud_Processor cloud_processor;

    bool write_points;
};

#endif // MAINWINDOW_H
