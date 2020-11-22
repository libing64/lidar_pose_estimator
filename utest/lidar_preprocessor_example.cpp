#include "lidar_preprocessor.h"
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


int main(int argc, char** argv)
{
    char filename[1024] = {0};
    if (argc < 2)
    {
        strcpy(filename, "/home/libing/data/kitti_raw/data_odometry_velodyne/dataset/sequences/01/velodyne/000120.bin");
    } else 
    {
        strcpy(filename, argv[1]);
    }
    cout << "filename: " << filename << endl;

    clock_t start = clock();

    lidar_preprocessor lidar;
    lidar.set_cloud_vis(true);
    lidar.process(filename);

    double dt = ((double)clock() - start) / CLOCKS_PER_SEC;
    cout << "cost time: " << dt << endl;
    return 0;
}