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
    char filename_prev[1024] = {0};
    char filename[1024] = {0};
    if (argc < 3)
    {
        strcpy(filename_prev, "/home/libing/data/kitty_raw/data_odometry_velodyne/dataset/sequences/01/velodyne/000100.bin");
        strcpy(filename, "/home/libing/data/kitty_raw/data_odometry_velodyne/dataset/sequences/01/velodyne/000120.bin");
    } else 
    {
        strcpy(filename_prev, argv[1]);
        strcpy(filename, argv[2]);
    }
    cout << "filename: " << filename_prev << endl << filename << endl;

    lidar_pose_estimator estimator;
    estimator.lidar_prev.process(filename_prev);
    estimator.lidar.process(filename);

    // lidar_preprocessor lidar;
    // lidar.readin_lidar_cloud(lidar_cloud);
    // lidar.inject_invalid_data();//TODO remove
    // lidar.remove_invalid_data();
    // lidar.get_horizon_angle_range();
    // lidar.get_cloud_curvature();
    // lidar.get_feature_points();

    return 0;
}