#include <pcl/io/pcd_io.h>
#include <pcl/filters/statistical_outlier_removal.h>

int main(int argc, char** argv)
{
    // Objects for storing the point clouds.
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZ>);

    // Read a PCD file from disk.
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("../cmake-build-debug/tree-1.pcd", *cloud) != 0)
    {return -1;}

    // Filter object.
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> filter;
    filter.setInputCloud(cloud);
    // Set number of neighbors to consider to 50.
    filter.setMeanK(50);
    // Set standard deviation multiplier to 1.
    // Points with a distance larger than 1 standard deviation of the mean distance will be outliers.
    filter.setStddevMulThresh(1.0);

    filter.filter(*filteredCloud);
    pcl::io::savePCDFileASCII("statistical_outlier_removal.pcd", *filteredCloud);
    std::cout<<"the number of the original point cloud: "<<cloud->points.size()<<std::endl;
    std::cout<<"the number of the statistical_outlier_removal point cloud: "<<filteredCloud->points.size()<<std::endl;
}