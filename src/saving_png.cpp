#include <pcl/io/pcd_io.h>
#include <pcl/io/png_io.h>

int main(int argc, char** argv)
{
    // Object for storing the point cloud.
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    // Read a PCD file from disk.
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB>("../cmake-build-debug/map.pcd", *cloud) != 0)
    {return -1;}

    // Save the image (cloud must be organized).
    pcl::io::savePNGFile("./output.png", *cloud, "rgb");
}
