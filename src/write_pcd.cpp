#include <pcl/io/pcd_io.h>

int main(int argc, char** argv)
{
    // Object for storing the point cloud.
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // Read a PCD file from disk.
    if (pcl::io::loadPCDFile<pcl::PointXYZ>( "../cmake-build-debug/map.pcd", *cloud) != 0)
    {
        return -1;
    }

    // Write it back to disk under a different name.
    // Another possibility would be "savePCDFileBinary()".
    pcl::io::savePCDFileASCII("output.pcd", *cloud);
}
