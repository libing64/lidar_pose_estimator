#include "lidar_pose_estimator.h"
#include <iostream>
#include <Eigen/Eigen>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/cloud_viewer.h>


using namespace std;
using namespace Eigen;


std::vector<float> read_lidar_data(const std::string filename)
{
    std::ifstream lidar_data_file(filename, std::ifstream::in | std::ifstream::binary);
    lidar_data_file.seekg(0, std::ios::end);
    const size_t num_elements = lidar_data_file.tellg() / sizeof(float);
    lidar_data_file.seekg(0, std::ios::beg);

    std::vector<float> lidar_data_buffer(num_elements);
    lidar_data_file.read(reinterpret_cast<char *>(&lidar_data_buffer[0]), num_elements * sizeof(float));
    return lidar_data_buffer;
}

int main(int argc, char** argv)
{
    char filename[1024] = {0};
    if (argc < 2)
    {
        strcpy(filename, "/home/libing/data/kitty_raw/data_odometry_velodyne/dataset/sequences/01/velodyne/000005.bin");
    } else 
    {
        strcpy(filename, argv[1]);
    }
    cout << "filename: " << filename << endl;
    vector<float> lidar_data = read_lidar_data(filename);
    cout << "lidar data size: " << lidar_data.size() << endl;

    pcl::PointCloud<PointType> lidar_cloud;
    for (int i = 0; i < lidar_data.size(); i += 4)
    {
        PointType p;
        p.x = lidar_data[i];
        p.y = lidar_data[i+1];
        p.z = lidar_data[i+2];
        p.intensity = lidar_data[i+3];
        lidar_cloud.push_back(p);

        if (isnan(p.x) || isnan(p.y) || isnan(p.z) || isnan(p.intensity))
        {
            cout << "nan: " << i << endl;
        }
    }
    lidar_cloud.height = 1;
    lidar_cloud.width = lidar_cloud.points.size();
    lidar_cloud.is_dense = true;
    cout << "lidar cloud size: " << lidar_cloud.points.size() << endl;
    pcl::visualization::CloudViewer viewer("lidar_cloud");
    viewer.showCloud(lidar_cloud.makeShared());
    while(!viewer.wasStopped()){}

    lidar_pose_estimator estimator;
    estimator.readin_lidar_cloud(lidar_cloud);
    estimator.remove_invalid_data();
    estimator.get_horizon_angle_range();

    return 0;
}