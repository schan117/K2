#ifndef GLWIDGET_H
#define GLWIDGET_H

#include <QGLWidget>
#include <QMouseEvent>
#include "support_math.h"
#include  <QList>
#include "Definitions.h"
#include <vector>

#ifndef Q_MOC_RUN
#include <pcl/point_types.h>
#endif

using namespace pcl;


class GLWidget : public QGLWidget
{
    Q_OBJECT

public:
    GLWidget(QWidget *parent = NULL);
    ~GLWidget();

    void Zoom(float amount);
    void Rotate(float delta_x, float delta_y);
    void Pan(float delta_x, float delta_y);

    bool draw_points;

    std::vector<PointXYZRGB, Eigen::aligned_allocator<PointXYZRGB> > point_cloud;

protected:
    void initparam();
    void initializeGL();
    void resizeGL(int w, int h);
    void paintGL();
    void wheelEvent(QWheelEvent *event);
    void mousePressEvent(QMouseEvent *event);
    void mouseReleaseEvent(QMouseEvent *event);
    void mouseMoveEvent(QMouseEvent *event);
    void keyPressEvent(QKeyEvent *event);


    void drawFrame();
    void drawGrid();
    void Draw_Point_Cloud();

private:



    QPoint lastpos;
    GLfloat deltaX,deltaY;
    vector3 dirX,dirY,up;
    point3 eye,center;
    bool movement;
    bool zoom,pan,rotate;
};

#endif
