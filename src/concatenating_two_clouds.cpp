#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

int main(int argc, char** argv)
{
    // Objects for storing the point clouds.
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudA(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudB(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudC(new pcl::PointCloud<pcl::PointXYZRGBA>);

    // Read two PCD files from disk.
    if (pcl::io::loadPCDFile<pcl::PointXYZRGBA>("../cmake-build-debug/tree-1.pcd", *cloudA) != 0)
    {return -1;}
    if (pcl::io::loadPCDFile<pcl::PointXYZRGBA>("../cmake-build-debug/tree-2.pcd", *cloudB) != 0)
    {return -1;}

    // Create cloud "C", with the points of both "A" and "B".
    *cloudC = (*cloudA) + (*cloudB);

    // Now cloudC->points.size() == cloudA->points.size() + cloudB->points.size().
    pcl::io::savePCDFileASCII("output.pcd", *cloudC);
    cout<<"the point number of the first tree:"<<cloudA->points.size()<<endl;
    cout<<"the point number of the second tree:"<<cloudB->points.size()<<endl;
    cout<<"the point number of the concatenating tree:"<<cloudC->points.size()<<endl;
}
