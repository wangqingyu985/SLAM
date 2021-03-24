#include <pcl/io/pcd_io.h>
#include <pcl/search/kdtree.h>
#include <iostream>
#include <vector>

int main(int argc, char** argv)
{
    // Object for storing the point cloud.
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // Read a PCD file from disk.
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("../cmake-build-debug/map.pcd", *cloud) != 0)
    {return -1;}

    // kd-tree object.
    pcl::search::KdTree<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(cloud);

    // We will find the 5 nearest neighbors of this point
    // (it does not have to be one of the cloud's, we can use any coordinate).
    pcl::PointXYZ point;
    point.x = -0.164755;
    point.y = 0.115467;
    point.z = 1.01352;
    // This vector will store the output neighbors.
    std::vector<int> pointIndices(5);
    // This vector will store their squared distances to the search point.
    std::vector<float> squaredDistances(5);
    // Perform the search, and print out results.
    if (kdtree.nearestKSearch(point, 5, pointIndices, squaredDistances) > 0)
    {
        std::cout << "5 nearest neighbors of the point:" << std::endl;
        for (size_t i = 0; i < pointIndices.size(); ++i)
            std::cout << "\t" << cloud->points[pointIndices[i]].x
                      << " " << cloud->points[pointIndices[i]].y
                      << " " << cloud->points[pointIndices[i]].z
                      << " (squared distance: " << squaredDistances[i] << ")" << std::endl;
    }

    // Now we find all neighbors within 3cm of the point
    // (inside a sphere of radius 3cm centered at the point).
    if (kdtree.radiusSearch(point, 0.03, pointIndices, squaredDistances) > 0)
    {
        std::cout << "Neighbors within 3cm:" << std::endl;
        for (size_t i = 0; i < pointIndices.size(); ++i)
            std::cout << "\t" << cloud->points[pointIndices[i]].x
                      << " " << cloud->points[pointIndices[i]].y
                      << " " << cloud->points[pointIndices[i]].z
                      << " (squared distance: " << squaredDistances[i] << ")" << std::endl;
    }
}
