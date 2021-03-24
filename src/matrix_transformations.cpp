#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>

int main(int argc, char** argv)
{
    // Objects for storing the point clouds.
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed(new pcl::PointCloud<pcl::PointXYZ>);

    // Read a PCD file from disk.
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("../cmake-build-debug/map.pcd", *cloud) != 0)
    {
        return -1;
    }

    // Transformation matrix object, initialized to the identity matrix
    // (a null transformation).
    Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();

    // Set a rotation around the Z axis (right hand rule).
    float theta = 90.0f * (M_PI / 180.0f); // 90 degrees.
    transformation(0, 0) = cos(theta);
    transformation(0, 1) = -sin(theta);
    transformation(1, 0) = sin(theta);
    transformation(1, 1) = cos(theta);

    // Set a translation on the X axis.
    transformation(0, 3) = 1.0f; // 1 meter (positive direction).

    pcl::transformPointCloud(*cloud, *transformed, transformation);

    // Visualize both the original and the result.
    pcl::visualization::PCLVisualizer viewer("../cmake-build-debug/map.pcd");
    viewer.addPointCloud(cloud, "original");
    // The transformed one's points will be red in color.
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> colorHandler(transformed, 255, 0, 0);
    viewer.addPointCloud(transformed, colorHandler, "transformed");
    // Add 3D colored axes to help see the transformation.
    viewer.addCoordinateSystem(1.0, 0);

    while (!viewer.wasStopped())
    {
        // Do nothing but wait.
    }
}