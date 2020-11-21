#include "lidar_pose_estimator.h"
#include <iostream>
#include <Eigen/Eigen>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>


using namespace std;
using namespace Eigen;
typedef pcl::PointXYZI PointType;

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
        strcpy(filename, "~/data/kitty_raw/data_odometry_velodyne/dataset/sequences/01/velodyne/000965.bin");
    }
    vector<float> lidar_data = read_lidar_data(filename);
    pcl::PointCloud<PointType> lidar_cloud;
    for (int i = 0; i < lidar_data.size(); i += 4)
    {
        PointType p;
        p.x = lidar_data[i];
        p.y = lidar_data[i+1];
        p.z = lidar_data[i+2];
        p.intensity = lidar_data[i+3];
    }
    cout << "lidar data size: " << lidar_data.size() << endl;

    return 0;
}