#include "lidar_preprocessor.h"
#include "lidar_pose_estimator.h"
#include "lidar_mapper.h"
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
        strcpy(filename_prev, "/home/libing/data/kitti_raw/data_odometry_velodyne/dataset/sequences/01/velodyne/000100.bin");
        strcpy(filename, "/home/libing/data/kitti_raw/data_odometry_velodyne/dataset/sequences/01/velodyne/000120.bin");
    } else 
    {
        strcpy(filename_prev, argv[1]);
        strcpy(filename, argv[2]);
    }
    cout << "filename: " << filename_prev << endl << filename << endl;


    clock_t start = clock();
    lidar_pose_estimator estimator;
    estimator.lidar_prev.process(filename_prev);
    estimator.lidar.process(filename);
    estimator.transform_update();

    double dt = ((double)clock() - start) / CLOCKS_PER_SEC;
    cout << "cost time: " << dt << endl;

    lidar_mapper mapper;
    mapper.update(estimator.lidar_prev.edge_points_mapping, estimator.lidar_prev.planar_points_mapping);

    mapper.q = estimator.q;
    mapper.t = estimator.t;
    mapper.update(estimator.lidar.edge_points_mapping, estimator.lidar.planar_points_mapping);

    return 0;
}