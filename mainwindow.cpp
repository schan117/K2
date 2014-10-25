#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QDebug>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    HRESULT hr = kinect_wrapper.Initialize_Kinect(0);
    qDebug() << "Initialize returns:" << hr << "for index:" << 0;

    hr = kinect_wrapper.Open_Color_Stream(0);
    qDebug() << "Open Color Stream returns:" << hr << "for index:" << 0;

    hr = kinect_wrapper.Open_Depth_Stream(0);
    qDebug() << "Open Depth Stream returns:" << hr << "for index:" << 0;

    hr = kinect_wrapper.Open_Coordinate_Mapper(0);

    capture_thread.Initialize_Capture_Thread(&kinect_wrapper);

    Connect_Signals();

    write_points = false;

}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::Connect_Signals()
{
    connect(&capture_thread, SIGNAL(Color_And_Depth_Frame_Captured(int)), this, SLOT(On_Color_And_Depth_Frame_Captured(int)));

}

void MainWindow::On_Color_And_Depth_Frame_Captured(int index)
{
    // Display Color Image
    Mat image = kinect_wrapper.Get_Color_Image(index);

    QImage qi = QImage(image.data, image.cols, image.rows, QImage::Format_RGBA8888);
    ui->Color_Image->setPixmap(QPixmap::fromImage(qi).scaled(ui->Color_Image->width(), ui->Color_Image->height()));

    // Display Depth Image
    Mat image2 = kinect_wrapper.Get_Depth_Image_Display(index);

    QImage qi2 = QImage(image2.data, image2.cols, image2.rows, QImage::Format_Indexed8);

    for (int i=0; i<256; i++)
    {
        qi2.setColor(i, qRgb(i,i,i));
    }

    ui->Depth_Image->setPixmap(QPixmap::fromImage(qi2).scaled(ui->Depth_Image->width(), ui->Depth_Image->height()));

    // Show in 3D
    std::vector<pcl::PointXYZRGB> points = kinect_wrapper.Get_Camera_Points(index);

    ui->View->point_cloud = points;
    ui->View->draw_points = true;
    ui->View->updateGL();


    if (write_points)
    {
        QFile qf("Output.asc");

        qf.open(QFile::Text | QFile::ReadWrite);

        QTextStream qts(&qf);

        for (int i=0; i<points.size(); i++)
        {
              qts << points[i].x << "\t" << points[i].y << "\t" << points[i].z << "\n\r";
        }

        qf.close();

        write_points = false;

    }


    capture_thread.mutex.unlock();
}

void MainWindow::on_pushButton_clicked()
{
    capture_thread.depth_min = ui->Min_Distance->value();
    capture_thread.depth_max = ui->Max_Distance->value();
    capture_thread.current_camera_index = 0;
    capture_thread.start();

}

void MainWindow::on_Min_Distance_valueChanged(int arg1)
{
    capture_thread.depth_min = arg1;
}

void MainWindow::on_Max_Distance_valueChanged(int arg1)
{
    capture_thread.depth_max = arg1;
}

void MainWindow::on_Capture_clicked()
{
    write_points = true;
}
