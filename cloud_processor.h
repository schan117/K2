#ifndef CLOUD_PROCESSOR_H
#define CLOUD_PROCESSOR_H

#ifndef Q_MOC_RUN
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#endif

#include <vector>

using namespace pcl;
using namespace std;

class Cloud_Processor
{
public:
    Cloud_Processor();
    ~Cloud_Processor();

     vector<PointXYZRGB> Apply_Voxel_Grid(vector<PointXYZRGB> input_points);

private:

};

#endif // CLOUD_PROCESSOR_H
