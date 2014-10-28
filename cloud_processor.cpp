#include "cloud_processor.h"
#include <QDebug>
#include <QFile>
#include <QTextStream>

Cloud_Processor::Cloud_Processor()
{
    // initialize the internal clouds

    process_cloud = PointCloud<PointXYZRGB>::Ptr(new PointCloud<PointXYZRGB>) ;
    processed_cloud = PointCloud<PointXYZRGB>::Ptr(new PointCloud<pcl::PointXYZRGB>);
}

Cloud_Processor::~Cloud_Processor()
{

}

void Cloud_Processor::Apply_Statistical_Outlier_Removal(int k, float sigma)
{
     //cerr << "Cloud before filtering: " << std::endl;
     //cerr << *process_cloud << std::endl;

     // Create the filtering object
     StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
     sor.setInputCloud (process_cloud);
     sor.setMeanK(k);
     sor.setStddevMulThresh(sigma);
     sor.filter (*processed_cloud);

     //std::cerr << "Cloud after filtering: " << std::endl;
     //std::cerr << *processed_cloud << std::endl;

     /*
     QFile qfo("Filtered.asc");

     qfo.open(QFile::Text | QFile::ReadWrite);

     QTextStream qtss(&qfo);

     for (int i=0; i<cloud_filtered->points.size(); i++)
     {
         qtss << cloud_filtered->points[i].x << "\t";
         qtss << cloud_filtered->points[i].y << "\t";
         qtss << cloud_filtered->points[i].z << "\n\r";
     }

     qfo.close();
    */
}

void Cloud_Processor::Apply_Moving_Least_Square(double search_radius)
{
     // Create a KD-Tree
     search::KdTree<PointXYZRGB>::Ptr tree (new search::KdTree<PointXYZRGB>);

     // Output has the PointNormal type in order to store the normals calculated by MLS
     PointCloud<PointNormal> mls_points;

     // Init object (second point type is for the normals, even if unused)
     MovingLeastSquares<PointXYZRGB, pcl::PointNormal> mls;

     mls.setComputeNormals(false);

     // Set parameters
     mls.setInputCloud(process_cloud);
     mls.setPolynomialFit(true);
     mls.setSearchMethod(tree);
     mls.setSearchRadius(search_radius);

     // Reconstruct
     mls.process(mls_points);
}

void Cloud_Processor::Set_Cloud_To_Process(vector<PointXYZRGB, Eigen::aligned_allocator<PointXYZRGB> > input_cloud)
{
    process_cloud->clear();
    process_cloud->points = input_cloud;
    process_cloud->width =  (uint32_t) input_cloud.size();
    process_cloud->height = 1;
}

vector<PointXYZRGB, Eigen::aligned_allocator<PointXYZRGB> > Cloud_Processor::Get_Processed_Cloud()
{
    return processed_cloud->points;
}

void Cloud_Processor::Output_ASC_File(QString filename, vector<PointXYZRGB, Eigen::aligned_allocator<PointXYZRGB> > cloud)
{
    QFile qf(filename);

    qf.open(QFile::Text | QFile::ReadWrite);

    QTextStream qts(&qf);

    for (int i=0; i<cloud.size(); i++)
    {
          qts << cloud[i].x << "\t" << cloud[i].y << "\t" << cloud[i].z << "\n\r";
    }

    qf.close();
}

void Cloud_Processor::Output_Processed_Cloud_As_PCD(QString filename)
{
    io::savePCDFileASCII(filename.toStdString(), *processed_cloud);
}

void Cloud_Processor::Apply_Voxel_Grid(double leaf_size)
{
     // Create the filtering object
     pcl::VoxelGrid<PointXYZRGB> sor;
     sor.setInputCloud (process_cloud);
     sor.setLeafSize (leaf_size, leaf_size, leaf_size);
     sor.filter(*processed_cloud);
}

