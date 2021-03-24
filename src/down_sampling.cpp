#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>

int main(int argc, char** argv)
{
    // Objects for storing the point clouds.
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZ>);

    // Read a PCD file from disk.
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("../cmake-build-debug/map.pcd", *cloud) != 0)
    {return -1;}

    // Filter object.
    pcl::VoxelGrid<pcl::PointXYZ> filter;
    filter.setInputCloud(cloud);
    // We set the size of every voxel to be 1x1x1cm
    // (only one point per every cubic centimeter will survive).
    filter.setLeafSize(0.01f, 0.01f, 0.01f);

    filter.filter(*filteredCloud);
    pcl::io::savePCDFileASCII("down_sampling.pcd", *filteredCloud);
    std::cout<<"the number of the original point cloud: "<<cloud->points.size()<<std::endl;
    std::cout<<"the number of the down_sampling point cloud: "<<filteredCloud->points.size()<<std::endl;
}