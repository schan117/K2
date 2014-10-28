#ifndef KINECT_WRAPPER_H
#define KINECT_WRAPPER_H

#include "Kinect.h"
#include "Definitions.h"
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>
#ifndef Q_MOC_RUN
#include <pcl/point_types.h>
#endif

using namespace cv;
using namespace std;
using namespace pcl;

class Kinect_Wrapper
{
public:
    Kinect_Wrapper();
    ~Kinect_Wrapper();

    HRESULT Initialize_Kinect(int index);
    HRESULT Open_Color_Stream(int index);
    HRESULT Open_Depth_Stream(int index);
    HRESULT Open_Coordinate_Mapper(int index);
    bool Trigger_Color_Image(int index);
    bool Trigger_Depth_Image(int index, int min, int max);

    Mat Get_Color_Image(int index);
    Mat Get_Depth_Image_Display(int index);
    vector<pcl::PointXYZRGB, Eigen::aligned_allocator<PointXYZRGB> > Get_Camera_Points(int index);

private:

    IKinectSensor* kinects[MAX_KINECT_COUNT];
    IColorFrameReader* m_pColorFrameReader[MAX_KINECT_COUNT];
    IDepthFrameReader* m_pDepthFrameReader[MAX_KINECT_COUNT];
    ICoordinateMapper* m_pCoordinateMapper[MAX_KINECT_COUNT];

    Mat color_frames[MAX_KINECT_COUNT];
    Mat depth_frames[MAX_KINECT_COUNT];
    Mat depth_frames_display[MAX_KINECT_COUNT];

    vector<vector<cv::Point> > contours;
    vector<Vec4i> hierarchy;


};

#endif // KINECT_WRAPPER_H
