#include <pcl/io/pcd_io.h>
#include <pcl/octree/octree.h>
#include <iostream>

int main(int argc, char** argv)
{
    // Objects for storing the point clouds.
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudA(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudB(new pcl::PointCloud<pcl::PointXYZ>);

    // Read two PCD files from disk.
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("../cmake-build-debug/tree-1.pcd", *cloudA) != 0)
    {return -1;}
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("../cmake-build-debug/tree-2.pcd", *cloudB) != 0)
    {return -1;}

    // Change detector object, with a resolution of 128
    // (resolution at lowest octree level).
    pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ> octree(1.0f);

    // Add cloudA to the octree.
    octree.setInputCloud(cloudA);
    octree.addPointsFromInputCloud();
    // The change detector object is able to store two clouds at the same time;
    // with this we can reset the buffer but keep the previous tree saved.
    octree.switchBuffers();
    // Now, add cloudB to the octree like we did before.
    octree.setInputCloud(cloudB);
    octree.addPointsFromInputCloud();

    std::vector<int> newPoints;
    // Get a vector with the indices of all points that are new in cloud B,
    // when compared with the ones that existed in cloud A.
    octree.getPointIndicesFromNewVoxels(newPoints);
    for (size_t i = 0; i < newPoints.size(); ++i)
    {
        std::cout << "Point (" << cloudB->points[newPoints[i]].x << ", "
                  << cloudB->points[newPoints[i]].y << ", "
                  << cloudB->points[newPoints[i]].z
                  << ") was not in cloud A but is in cloud B" << std::endl;
    }
    std::cout << newPoints.size() << std::endl;
}