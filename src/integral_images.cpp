#include <pcl/io/pcd_io.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/visualization/pcl_visualizer.h>

int main(int argc, char** argv)
{
    // Object for storing the point cloud.
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    // Object for storing the normals.
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

    // Read a PCD file from disk.
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("../cmake-build-debug/map.pcd", *cloud) != 0)
    {return -1;}

    // Object for normal estimation.
    pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
    normalEstimation.setInputCloud(cloud);
    // Other estimation methods: COVARIANCE_MATRIX, AVERAGE_DEPTH_CHANGE, SIMPLE_3D_GRADIENT.
    // They determine the smoothness of the result, and the running time.
    normalEstimation.setNormalEstimationMethod(normalEstimation.AVERAGE_3D_GRADIENT);
    // Depth threshold for computing object borders based on depth changes, in meters.
    normalEstimation.setMaxDepthChangeFactor(0.02f);
    // Factor that influences the size of the area used to smooth the normals.
    normalEstimation.setNormalSmoothingSize(10.0f);

    // Calculate the normals.
    normalEstimation.compute(*normals);

    // Visualize them.
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Normals"));
    viewer->addPointCloud<pcl::PointXYZ>(cloud, "cloud");
    // Display one normal out of 20, as a line of length 3cm.
    viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud, normals, 20, 0.03, "normals");
    while (!viewer->wasStopped())
    {viewer->spinOnce(100);}
}
