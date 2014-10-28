#ifndef CLOUD_PROCESSOR_H
#define CLOUD_PROCESSOR_H

#ifndef Q_MOC_RUN
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#endif

#include <vector>
#include <QString>

using namespace pcl;
using namespace std;

class Cloud_Processor
{
public:
    Cloud_Processor();
    ~Cloud_Processor();

     void Apply_Voxel_Grid(double leaf_size);
     void Apply_Statistical_Outlier_Removal(int k, float sigma);
     void Apply_Moving_Least_Square(double search_radius);

     void Set_Cloud_To_Process(vector<PointXYZRGB, Eigen::aligned_allocator<PointXYZRGB> > input_cloud);
     vector<PointXYZRGB, Eigen::aligned_allocator<PointXYZRGB> > Get_Processed_Cloud();

     void Output_ASC_File(QString filename, vector<PointXYZRGB, Eigen::aligned_allocator<PointXYZRGB> > cloud);
     void Output_Processed_Cloud_As_PCD(QString filename);

private:

     PointCloud<PointXYZRGB>::Ptr process_cloud;
     PointCloud<PointXYZRGB>::Ptr processed_cloud;


};

#endif // CLOUD_PROCESSOR_H
