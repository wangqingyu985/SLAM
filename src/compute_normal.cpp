#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <iostream>

int main(int argc, char** argv)
{
    // Objects for storing the point clouds.
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPoints(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::Normal>::Ptr cloudNormals(new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<pcl::PointNormal>::Ptr cloudAll(new pcl::PointCloud<pcl::PointNormal>);

    // Read a PCD file from disk.
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("../cmake-build-debug/map.pcd", *cloudPoints) != 0)
    {return -1;}

    // Compute the normals of the cloud (do not worry, we will see this later).
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
    normalEstimation.setInputCloud(cloudPoints);
    normalEstimation.setRadiusSearch(0.05);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
    normalEstimation.setSearchMethod(kdtree);
    normalEstimation.compute(*cloudNormals);

    // Concatenate the fields (PointXYZ + Normal = PointNormal).
    pcl::concatenateFields(*cloudPoints, *cloudNormals, *cloudAll);

    // Print the data to standard output.
    for (size_t currentPoint = 0; currentPoint < cloudAll->points.size(); currentPoint++)
    {
        std::cout << "Point:" << std::endl;
        std::cout << "\tXYZ:" << cloudAll->points[currentPoint].x << " "
                  << cloudAll->points[currentPoint].y << " "
                  << cloudAll->points[currentPoint].z << std::endl;
        std::cout << "\tNormal:" << cloudAll->points[currentPoint].normal[0] << " "
                  << cloudAll->points[currentPoint].normal[1] << " "
                  << cloudAll->points[currentPoint].normal[2] << std::endl;
    }
}