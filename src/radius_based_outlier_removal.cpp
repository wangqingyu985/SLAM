#include <pcl/io/pcd_io.h>
#include <pcl/filters/radius_outlier_removal.h>

int main(int argc, char** argv)
{
    // Objects for storing the point clouds.
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZ>);

    // Read a PCD file from disk.
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("../cmake-build-debug/tree-1.pcd", *cloud) != 0)
    {return -1;}

    // Filter object.
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> filter;
    filter.setInputCloud(cloud);
    // Every point must have 10 neighbors within 15cm, or it will be removed.
    filter.setRadiusSearch(0.01);
    filter.setMinNeighborsInRadius(10);

    filter.filter(*filteredCloud);
    pcl::io::savePCDFileASCII("radius_based_outlier_removal.pcd", *filteredCloud);
    std::cout<<"the number of the original point cloud: "<<cloud->points.size()<<std::endl;
    std::cout<<"the number of the radius_based_outlier_removal point cloud: "<<filteredCloud->points.size()<<std::endl;
}