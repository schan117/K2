#include "kinect_wrapper.h"
#include <QDebug>
#include <QTest>

Kinect_Wrapper::Kinect_Wrapper()
{
    for (int i=0; i<MAX_KINECT_COUNT; i++)
    {
        m_pColorFrameReader[i] = NULL;
    }
}

Kinect_Wrapper::~Kinect_Wrapper()
{

}

HRESULT Kinect_Wrapper::Initialize_Kinect(int index)
{
    HRESULT hr = GetDefaultKinectSensor(&kinects[index]);

    qDebug() << "GetDefaultKinectSensor for returns:" << hr << "for index:" << index;

    if (kinects[index])
    {
        hr = kinects[index]->Open();

        qDebug() << "Open returns:" << hr << "for index:" << index;
    }

    return hr;
}

HRESULT Kinect_Wrapper::Open_Color_Stream(int index)
{
    HRESULT hr;

    if (kinects[index])
    {
        // Initialize the Kinect and get the color reader
        IColorFrameSource* pColorFrameSource = NULL;

        hr = kinects[index]->get_ColorFrameSource(&pColorFrameSource);
        qDebug() << "Get_ColorFrameSource returns:" << hr << "for index:" << index;

        if (SUCCEEDED(hr))
        {
            hr = pColorFrameSource->OpenReader(&m_pColorFrameReader[index]);
            qDebug() << "Open reader returns:" << hr << "for index:" << index;
            qDebug() << "Address of color frame reader:" << m_pColorFrameReader[index];
        }

        pColorFrameSource->Release();
    }

    if (!kinects[index] || FAILED(hr))
    {
        return E_FAIL;
    }

    // wait until sensor is opened

    int sensor_wait_time = 0;
    bool sensor_timeout = false;
    BOOLEAN b;
/*
    do
    {
        hr = kinects[index]->get_IsAvailable(&b);

        if (FAILED(hr))
        {
            return hr;
        }
        QTest::qWait(50);
        sensor_wait_time = sensor_wait_time + 50;

        if (sensor_wait_time > MAX_ENUMERTAION_TIME)
        {
            sensor_timeout = true;
            break;
        }

    } while (b != 1);
*/
    if (!sensor_timeout)
    {
        return S_OK;
    }
    else
    {
        return S_FALSE;
    }

}

HRESULT Kinect_Wrapper::Open_Depth_Stream(int index)
{
    HRESULT hr;

    if (kinects[index])
    {
        // Initialize the Kinect and get the color reader
        IDepthFrameSource* pDepthFrameSource = NULL;

        hr = kinects[index]->get_DepthFrameSource(&pDepthFrameSource);
        qDebug() << "Get_DepthFrameSource returns:" << hr << "for index:" << index;

        if (SUCCEEDED(hr))
        {
            hr = pDepthFrameSource->OpenReader(&m_pDepthFrameReader[index]);
            qDebug() << "Open reader returns:" << hr << "for index:" << index;
            qDebug() << "Address of depth frame reader:" << m_pDepthFrameReader[index];
        }

        pDepthFrameSource->Release();
    }

    if (!kinects[index] || FAILED(hr))
    {
        return E_FAIL;
    }

    // wait until sensor is opened

    int sensor_wait_time = 0;
    bool sensor_timeout = false;
    BOOLEAN b;
/*
    do
    {
        hr = kinects[index]->get_IsAvailable(&b);

        if (FAILED(hr))
        {
            return hr;
        }
        QTest::qWait(50);
        sensor_wait_time = sensor_wait_time + 50;

        if (sensor_wait_time > MAX_ENUMERTAION_TIME)
        {
            sensor_timeout = true;
            break;
        }

    } while (b != 1);
*/
    if (!sensor_timeout)
    {
        return S_OK;
    }
    else
    {
        return S_FALSE;
    }
}

HRESULT Kinect_Wrapper::Open_Coordinate_Mapper(int index)
{
    HRESULT hr;

    if (kinects[index])
    {
        hr = kinects[index]->get_CoordinateMapper(&m_pCoordinateMapper[index]);
        qDebug() << "Get Coordinate Mapper returns:" << hr << "for index:" << index;
    }

    return hr;

}

bool Kinect_Wrapper::Trigger_Color_Image(int index)
{
    if (!m_pColorFrameReader[index])
    {
        return false;
    }

    IColorFrame* pColorFrame = NULL;

    HRESULT hr;

    do
    {
        hr = m_pColorFrameReader[index]->AcquireLatestFrame(&pColorFrame);
        QTest::qWait(30);
    } while (hr != S_OK);

    //qDebug() << "Acquire latest color frame returns:" << hr;

    int nWidth = 0;
    int nHeight = 0;
    ColorImageFormat imageFormat = ColorImageFormat_None;
    UINT nBufferSize = 0;
    BYTE *pBuffer = NULL;
    Mat image;

    if (SUCCEEDED(hr))
    {
        IFrameDescription* pFrameDescription = NULL;

        if (SUCCEEDED(hr))
        {
            hr = pColorFrame->get_FrameDescription(&pFrameDescription);
        }

        if (SUCCEEDED(hr))
        {
            hr = pFrameDescription->get_Width(&nWidth);
        }

        if (SUCCEEDED(hr))
        {
            hr = pFrameDescription->get_Height(&nHeight);
        }

        if (SUCCEEDED(hr))
        {
            hr = pColorFrame->get_RawColorImageFormat(&imageFormat);
        }

        if (SUCCEEDED(hr))
        {
            // Convert buffer to BGRA format
            nBufferSize = nWidth * nHeight * 4;
            pBuffer = new BYTE[nBufferSize];
            hr = pColorFrame->CopyConvertedFrameDataToArray(nBufferSize, reinterpret_cast<BYTE*>(pBuffer), ColorImageFormat_Rgba);
            image = Mat(nHeight, nWidth, CV_8UC4, pBuffer);
        }

        if (SUCCEEDED(hr))
        {
            color_frames[index] = image.clone();
        }

        if (pFrameDescription != NULL)
        {
            pFrameDescription->Release();
        }
    }

    // Critical line, if you dont release, SDK doesnt get the next image
    if (pColorFrame != NULL)
    {
         pColorFrame->Release();
    }

    delete [] pBuffer;

    if (SUCCEEDED(hr))
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool Kinect_Wrapper::Trigger_Depth_Image(int index, int min, int max)
{
    if (!m_pDepthFrameReader[index])
    {
        return true;
    }

    IDepthFrame* pDepthFrame = NULL;

    HRESULT hr;

    do
    {
        hr =  m_pDepthFrameReader[index]->AcquireLatestFrame(&pDepthFrame);
        QTest::qWait(30);
    } while (hr != S_OK);

    //qDebug() << "Acquire latest depth frame returns:" << hr << "for index:" << index;

    IFrameDescription* pFrameDescription = NULL;
    int nWidth = 0;
    int nHeight = 0;
    USHORT nDepthMinReliableDistance = 0;
    USHORT nDepthMaxDistance = 0;
    UINT nBufferSize = 0;
    UINT16 *pBuffer = NULL;
    Mat image;

    if (SUCCEEDED(hr))
    {
        if (SUCCEEDED(hr))
        {
            hr = pDepthFrame->get_FrameDescription(&pFrameDescription);
        }

        if (SUCCEEDED(hr))
        {
            hr = pFrameDescription->get_Width(&nWidth);
        }

        if (SUCCEEDED(hr))
        {
            hr = pFrameDescription->get_Height(&nHeight);
        }

        if (SUCCEEDED(hr))
        {
            hr = pDepthFrame->get_DepthMinReliableDistance(&nDepthMinReliableDistance);
        }

        if (SUCCEEDED(hr))
        {
            // In order to see the full range of depth (including the less reliable far field depth)
            // we are setting nDepthMaxDistance to the extreme potential depth threshold
            nDepthMaxDistance = USHRT_MAX;

            // Note:  If you wish to filter by reliable depth distance, uncomment the following line.
            hr = pDepthFrame->get_DepthMaxReliableDistance(&nDepthMaxDistance);
        }

        if (SUCCEEDED(hr))
        {
            hr = pDepthFrame->AccessUnderlyingBuffer(&nBufferSize, &pBuffer);
            image = Mat(nHeight, nWidth, CV_16UC1, pBuffer).clone();

            UINT16* data = (UINT16*) image.data;
/*
            // Erode to filter some noise
            Mat range_image;
            Mat element = getStructuringElement(MORPH_RECT, Size(3,3));
            inRange(image, Scalar(min), Scalar(max), range_image);
            erode(range_image, range_image, element);
            uchar* range_data = (uchar*) range_image.data;
*/
            // filter according to min and max
            for (int y=0; y < nHeight; y++)
            {
                for (int x=0; x< nWidth; x++)
                {
                    UINT16 value = data[y * nWidth + x];
                    //uchar mask_value = range_data[y * nWidth + x];

                    if  ( (value < min) || (value > max) ) //|| mask_value == 0)
                    {
                        data[y * nWidth + x] = 0;
                    }
                }
            }


            image.convertTo(depth_frames_display[index], CV_8UC1, 255.0 / max);
            depth_frames[index] = image;

        }

        if (pFrameDescription != NULL)
        {
            pFrameDescription->Release();
        }
    }

    if (pDepthFrame != NULL)
    {
        pDepthFrame->Release();
    }

    if (SUCCEEDED(hr))
    {
        return true;
    }
    else
    {
        return false;
    }
}

Mat Kinect_Wrapper::Get_Color_Image(int index)
{
    return color_frames[index];
}

Mat Kinect_Wrapper::Get_Depth_Image_Display(int index)
{
    return depth_frames_display[index];
}

vector<pcl::PointXYZRGB> Kinect_Wrapper::Get_Camera_Points(int index)
{
    UINT point_count = depth_frames[index].rows * depth_frames[index].cols;

    // assume color data has already been captured
    Vec4b* v = (Vec4b*) color_frames[index].data;

    CameraSpacePoint* camera_points = new CameraSpacePoint[point_count];
    ColorSpacePoint* color_points = new ColorSpacePoint[point_count];

    m_pCoordinateMapper[index]->MapDepthFrameToCameraSpace(point_count, (const UINT16*) depth_frames[index].data, point_count, camera_points);
    m_pCoordinateMapper[index]->MapDepthFrameToColorSpace(point_count, (const UINT16*) depth_frames[index].data, point_count, color_points);


    vector<pcl::PointXYZRGB> points;
    pcl::PointXYZRGB point;
    Vec4b vv;

    for (int i=0; i<point_count; i++)
    {
        if (_finite(camera_points[i].X) && _finite(camera_points[i].Y) && _finite(camera_points[i].Z) )
        {
            point.x = camera_points[i].X * 1000; // convert to mm
            point.y = camera_points[i].Y * 1000;
            point.z = camera_points[i].Z * 1000 - 500;

            // make sure color points are within boundary
            if (color_points[i].X < 0)
            {
                color_points[i].X = 0;
            }
            else if (color_points[i].X >= color_frames[index].cols)
            {
                color_points[i].X = color_frames[index].cols - 1;
            }

            if (color_points[i].Y < 0)
            {
                color_points[i].Y = 0;
            }
            else if (color_points[i].Y >= color_frames[index].rows)
            {
                color_points[i].Y = color_frames[index].rows - 1;
            }

            vv = v[color_frames[index].cols * (int) color_points[i].Y + (int) color_points[i].X];

            point.r = vv[0];
            point.g = vv[1];
            point.b = vv[2];

            points.push_back(point);
        }
    }

    delete [] camera_points;
    delete [] color_points;

    return points;
}
