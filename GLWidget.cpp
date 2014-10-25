#include "glwidget.h"
#include <gl/GLU.h>
#include <QDebug>


GLWidget::GLWidget(QWidget *parent)
{
    this->setParent(parent);
    initparam();
    setMouseTracking(true);

    draw_points = false;

}

GLWidget::~GLWidget()
{

}

void GLWidget::Zoom(float amount)
{
    eye.x = center.x + (eye.x - center.x) * (amount + 1);
    eye.y = center.y + (eye.y - center.y) * (amount + 1);
    eye.z = center.z + (eye.z - center.z) * (amount + 1);
    updateGL();
}

void GLWidget::Rotate(float delta_x, float delta_y)
{

    float dx = delta_x / (float) width();
    float dy = delta_y / (float) height();

    dirX = up;
    vector3 watch;
    watch.x = center.x-eye.x;
    watch.y = center.y-eye.y;
    watch.z = center.z-eye.z;
    vector3 upper = crossproduct(up,watch);
    dirY = unitVector(upper);

    //------cal new eye Posistion----------
    eye = rotatePoint(eye,center,dirX,-dx*PI);
    eye = rotatePoint(eye,center,dirY,dy*PI);

    //------cal new up vector--------------
    point3 centerup;
    centerup.x = center.x + up.x;
    centerup.y = center.y + up.y;
    centerup.z = center.z + up.z;
    point3 upPoint = rotatePoint(centerup,center,dirY,dy*PI);
    upPoint.x = upPoint.x - center.x;
    upPoint.y = upPoint.y - center.y;
    upPoint.z = upPoint.z - center.z;

    up = unitVector(pointToVector(upPoint));
    updateGL();

}

void GLWidget::Pan(float delta_x, float delta_y)
{
    float dx = delta_x / (float) width() ;
    float dy = delta_y / (float) height();

    float dis = distance(eye,center);
    float length = 2*dis*tan((22.5/2)*(PI/180));
    // length = 2 ||eye − center|| · tan (FoVy/2)

    center.x = center.x + dirY.x*dx*length*((float) width()/ (float) height())+dirX.x*dy*length;
    center.y = center.y + dirY.y*dx*length*((float) width()/ (float) height())+dirX.y*dy*length;
    center.z = center.z + dirY.z*dx*length*((float) width()/ (float) height())+dirX.z*dy*length;
    updateGL();

}


void GLWidget::initparam()
{
    eye.x = 0;
    eye.y = 50;
    eye.z = -500;
    center.x =0;
    center.y =10;
    center.z =0;
    up.x = 0;
    up.y = 1;
    up.z = 0;
    dirX = up;
    vector3 watch;
    watch.x = center.x-eye.x;
    watch.y = center.y-eye.y;
    watch.z = center.z-eye.z;
    vector3 upper = crossproduct(up,watch);
    dirY = unitVector(upper);

    movement = false;
    zoom = false;
    pan = false;
    rotate = false;
}

void GLWidget::initializeGL()
{
    glClearColor(0.05f, 0.05f, 0.05f, 0.0f);					// Black Background
    //glClearColor(0,0,0,1.0f);
    glClearDepth(1.0f);							// Depth Buffer Setup
    glEnable(GL_DEPTH_TEST);						// Enables Depth Testing
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
    glDepthFunc(GL_LEQUAL);							// The Type Of Depth Test To Do
    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);			// Really Nice Perspective Calculations

    //----------------Setup Light------------------------------
    GLfloat mat_diffuse[4] = {0.75, 0.75, 0.75, 1.0};
    GLfloat mat_specular[4] = {0.55, 0.55, 0.55, 1.0};
    GLfloat mat_shininess[1] = {80};

    glMaterialfv(GL_FRONT_AND_BACK,GL_AMBIENT_AND_DIFFUSE,mat_diffuse);
    glMaterialfv(GL_FRONT,GL_SPECULAR,mat_specular);
    glMaterialfv(GL_FRONT,GL_SHININESS,mat_shininess);

    GLfloat light0_ambient[4] = { 0.0, 0.0, 0.0, 1.0};
    GLfloat light0_color[4] = { 0.4, 0.4, 0.4, 1.0 };
    glLightfv(GL_LIGHT0, GL_AMBIENT, light0_ambient);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, light0_color);
    glLightfv(GL_LIGHT0, GL_SPECULAR, light0_color);

    glEnable(GL_LIGHT0);
    glEnable(GL_LIGHTING);

    float modelTwoside[] = {GL_TRUE};
    glLightModelfv(GL_LIGHT_MODEL_TWO_SIDE, modelTwoside);

    glShadeModel(GL_SMOOTH);						// Enables Smooth Shading

    //----------Prevent Edge/Face overlapping
    glEnable(GL_POLYGON_OFFSET_FILL);
    glPolygonOffset(1.0f, 1.0f);
}


void GLWidget::resizeGL(int w, int h)
{
    if (h==0)								// Prevent A Divide By Zero By
    {
        h=1;							// Making Height Equal One
    }
    glViewport(0, 0, w, h);					// Reset The Current Viewport
    glMatrixMode(GL_PROJECTION);						// Select The Projection Matrix
    glLoadIdentity();						// Reset The Projection Matrix
    // Calculate The Aspect Ratio Of The Window
    GLfloat x = GLfloat(w) / GLfloat(h);
    //glFrustum(-x, x, -100.0, 100.0, 0.1, 100.0);
    gluPerspective(22.5f,x,5.0f,10000.0f);
    glMatrixMode(GL_MODELVIEW);						// Select The Modelview Matrix
    glLoadIdentity();

}

void GLWidget::paintGL()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    /*
    //==========================================
    //Gradient background
    glMatrixMode (GL_MODELVIEW);
    glPushMatrix ();
    glLoadIdentity ();
    glMatrixMode (GL_PROJECTION);
    glPushMatrix ();
    glLoadIdentity ();

    glDisable(GL_LIGHTING);
    glDisable(GL_DEPTH_TEST);

    glBegin (GL_QUADS);
    glColor3f(0.9,0.9,0.9);
    glVertex3f (-1.0f, -1.0f, -1.0f);
    glVertex3f (1.0f, -1.0f, -1.0f);

    glColor3f(0.6,0.6,0.6);
    glVertex3f (1.0f, 1.0f, -1.0f);
    glVertex3f (-1.0f, 1.0f, -1.0f);
    glEnd ();

    glEnable(GL_LIGHTING);
    glEnable(GL_DEPTH_TEST);
    glPopMatrix ();
    glMatrixMode (GL_MODELVIEW);
    glPopMatrix ();
    //================================
    */

    glLoadIdentity();
    gluLookAt(eye.x, eye.y, eye.z, center.x, center.y, center.z, up.x, up.y,up.z);

    // draw reference on every refresh
    drawFrame();
    //drawGrid();

    if (draw_points)
    {
        Draw_Point_Cloud();
    }

    glFlush();

}

void GLWidget::wheelEvent(QWheelEvent *event)
{

}

void GLWidget::mousePressEvent(QMouseEvent *event)
{
    lastpos = event->pos();
    movement = true;

    if(event->button() == Qt::RightButton)
    {
        pan = true;
        event->accept();
    }

    if(event->button() == Qt::LeftButton)
    {
        rotate = true;
        event->accept();
    }
}

void GLWidget::mouseReleaseEvent(QMouseEvent *event)
{
    movement = false;
    zoom = false;
    rotate = false;
    pan = false;
}

void GLWidget::mouseMoveEvent(QMouseEvent *event)
{
    // keep track of movements
    if(movement)
    {
        QPoint curpos = event->pos();
        deltaX = (curpos.x() - lastpos.x())/(float)width();
        deltaY = (curpos.y() - lastpos.y())/(float)height();
        lastpos = event->pos();
    }

    /*
        if(zoom){
            // eye = center + (eye − center) · (y + 1).
            eye.x = center.x + (eye.x - center.x) * (deltaY + 1);
            eye.y = center.y + (eye.y - center.y) * (deltaY + 1);
            eye.z = center.z + (eye.z - center.z) * (deltaY + 1);
            updateGL();
        }*/

    if(rotate)
    {
        //-----define dirX,dirY------
        dirX = up;
        vector3 watch;
        watch.x = center.x-eye.x;
        watch.y = center.y-eye.y;
        watch.z = center.z-eye.z;
        vector3 upper = crossproduct(up,watch);
        dirY = unitVector(upper);

        //------cal new eye Posistion----------
        eye = rotatePoint(eye,center,dirX,-deltaX*PI);
        eye = rotatePoint(eye,center,dirY,deltaY*PI);

        //------cal new up vector--------------
        point3 centerup;
        centerup.x = center.x + up.x;
        centerup.y = center.y + up.y;
        centerup.z = center.z + up.z;
        point3 upPoint = rotatePoint(centerup,center,dirY,deltaY*PI);
        upPoint.x = upPoint.x - center.x;
        upPoint.y = upPoint.y - center.y;
        upPoint.z = upPoint.z - center.z;

        up = unitVector(pointToVector(upPoint));
        updateGL();
    }

    if(pan)
    {
        //float dis = sqrt((eye.x-center.x)*(eye.x-center.x)+(eye.y-center.y)*(eye.y-center.y)+(eye.z-center.z)*(eye.z-center.z));
        float dis = distance(eye,center);
        float length = 2*dis*tan((22.5/2)*(PI/180));
        // length = 2 ||eye − center|| · tan (FoVy/2)

        center.x = center.x + dirY.x*deltaX*length*((float) width()/ (float) height())+dirX.x*deltaY*length;
        center.y = center.y + dirY.y*deltaX*length*((float) width()/ (float) height())+dirX.y*deltaY*length;
        center.z = center.z + dirY.z*deltaX*length*((float) width()/ (float) height())+dirX.z*deltaY*length;
        updateGL();
    }
}

void GLWidget::keyPressEvent(QKeyEvent *event)
{
    /*
    if (event->key() == Qt::Key_A)
    {
        Rotate(-5,0);
        event->accept();
    }
    else if (event->key() == Qt::Key_D)
    {
        Rotate(5,0);
        event->accept();
    }
    else if (event->key() == Qt::Key_W)
    {
        Rotate(0,-5);
        event->accept();
    }
    else if (event->key() == Qt::Key_S)
    {
        Rotate(0,5);
        event->accept();
    }
    */

    if (event->key() == Qt::Key_Z)
    {
        Zoom(0.1);
        event->accept();
    }
    else if (event->key() == Qt::Key_X)
    {
        Zoom(-0.1);
        event->accept();
    }

    /*
    if (event->key() == Qt::Key_Left)
    {
        Pan(-5, 0);
        event->accept();
    }
    else if (event->key() == Qt::Key_Right)
    {
        Pan(5, 0);
        event->accept();
    }
    else if (event->key() == Qt::Key_Up)
    {
        Pan(0, -5);
        event->accept();
    }
    else if (event->key() == Qt::Key_Down)
    {
        Pan(0, 5);
        event->accept();
    }
    */





}

void GLWidget::drawFrame()
{
    GLfloat n = 9;
    GLfloat d = 2*PI/n ;
    GLfloat a = 0;
    GLfloat r = 1.5;

    glPushMatrix();
    glDisable(GL_LIGHTING);
    glLineWidth(2);
    //------------X-Axis-------------
    glBegin(GL_LINES);
    glColor3f(1.0f,0.0f,0.0f);
    glVertex3f( 0.0f, 0.0f, 0.0f);
    glVertex3f( 20.0f,0.0f, 0.0f);
    glEnd();
    a = 0;
    glBegin(GL_TRIANGLE_FAN);
    glVertex3f(20,0,0);
    for (int i = 0 ; i <= n ; i++ ){
        glVertex3f(20,cos(a)*r, sin(a)*r);
        a += d ;
    }
    glEnd() ;
    a = 0.0 ;
    glBegin(GL_TRIANGLE_FAN);
    glVertex3f(25,0,0);
    for (int i = 0 ; i <= n ; i++ ){
        glVertex3f(20,cos(a)*r, sin(a)*r);
        a += d ;
    }
    glEnd() ;

    //------------Y-Axis-------------
    glBegin(GL_LINES);
    glColor3f(0.0f,1.0f,0.0f);
    glVertex3f( 0.0f, 0.0f, 0.0f);
    glVertex3f( 0.0f,20.0f, 0.0f);
    glEnd();
    a = 0;
    glBegin(GL_TRIANGLE_FAN);
    glVertex3f(0,20,0);
    for (int i = 0 ; i <= n ; i++ ){
        glVertex3f(cos(a)*r,20, sin(a)*r);
        a += d ;
    }
    glEnd() ;
    a = 0.0 ;
    glBegin(GL_TRIANGLE_FAN);
    glVertex3f(0,25,0);
    for (int i = 0 ; i <= n ; i++ ){
        glVertex3f(cos(a)*r,20,sin(a)*r);
        a += d ;
    }
    glEnd() ;

    //--------------Z-Axis---------------
    glBegin(GL_LINES);
    glColor3f(0.0f,0.0f,1.0f);
    glVertex3f( 0.0f, 0.0f, 0.0f);
    glVertex3f( 0.0f,0.0f, 20.0f);
    glEnd();
    a = 0;
    glBegin(GL_TRIANGLE_FAN);
    glVertex3f(0,0,20);
    for (int i = 0 ; i <= n ; i++ ){
        glVertex3f(cos(a)*r, sin(a)*r, 20);
        a += d ;
    }
    glEnd() ;
    a = 0.0 ;
    glBegin(GL_TRIANGLE_FAN);
    glVertex3f(0,0,25);
    for (int i = 0 ; i <= n ; i++ ){
        glVertex3f(cos(a)*r, sin(a)*r, 20);
        a += d ;
    }
    glEnd() ;

    glEnable(GL_LIGHTING);
    glLineWidth(1);
    glPopMatrix();

}

void GLWidget::drawGrid()
{
    glPushMatrix();
    glDisable(GL_LIGHTING);
    glColor3f(0.2f,0.2f,0.2f);

    for(int i=-100;i<=100;i=i+10)
    {
        glBegin(GL_LINES);
        glVertex3f( -100.0f, 0, i );
        glVertex3f( 100.0f, 0, i);
        glEnd();
        glBegin(GL_LINES);
        glVertex3f( i, 0 , -100.0f);
        glVertex3f( i, 0 , 100.0f);
        glEnd();
    }

    glEnable(GL_LIGHTING);
    glPopMatrix();
}

void GLWidget::Draw_Point_Cloud()
{
    glPushMatrix();
    glPointSize(3);
    glDisable(GL_LIGHTING);
    glBegin(GL_POINTS);
    //glColor3f(0.3f,0.3f,0.3f);

    for(int i = 0; i < point_cloud.size() ;i++)
    {
       // glColor3f(0.8f,0.8f,0.8f);
        glColor3f(point_cloud[i].r/255.0f, point_cloud[i].g/255.0f, point_cloud[i].b/255.0f);
        glVertex3f(point_cloud[i].x,
                   point_cloud[i].y,
                   point_cloud[i].z);

    }


    glEnd();
    glEnable(GL_LIGHTING);
    glPopMatrix();

}
