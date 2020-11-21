#ifndef _LIDAR_POSE_ESTIMATOR_H
#define _LIDAR_POSE_ESTIMATOR_H

#include <iostream>
#include <vector>
#include <Eigen/Eigen>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>


using std::cout;
using std::endl;
using std::vector;

typedef pcl::PointXYZI PointType;

class lidar_pose_estimator
{
private:
    /* data */
public:
    int channel = 64;
    const float min_range = 0.1;
    float min_angle_hori;
    float max_angle_hori;
    pcl::PointCloud<PointType> lidar_cloud;
    lidar_pose_estimator(/* args */);
    ~lidar_pose_estimator();

    void readin_lidar_cloud(pcl::PointCloud<PointType>& cloud){lidar_cloud = cloud;}
    void remove_invalid_data();
    void get_horizon_angle_range();
};

lidar_pose_estimator::lidar_pose_estimator(/* args */)
{
}

lidar_pose_estimator::~lidar_pose_estimator()
{
}

void lidar_pose_estimator::remove_invalid_data()
{
    vector<int> index;
    cout << "lidar_cloud size: " << lidar_cloud.points.size() << endl;
    pcl::removeNaNFromPointCloud(lidar_cloud, lidar_cloud, index);
    cout << "lidar_cloud size: " << lidar_cloud.points.size() << endl;
    int j = 0;
    for (auto i = 0; i < lidar_cloud.size(); i++)
    {
        PointType p = lidar_cloud.points[i];
        float dist = sqrtf(p.x * p.x + p.y * p.y + p.z * p.z);
        if (dist > min_range)
        {
            j++;
            lidar_cloud[j] = lidar_cloud[i];
        }
    }
    cout << "removed near data cnt: " << (lidar_cloud.size() - j) << endl;
    if (j != lidar_cloud.size()) 
    {
        lidar_cloud.resize(j);
    }
    cout << "lidar_cloud size: " << lidar_cloud.points.size() << endl;
    lidar_cloud.height = 1;
    lidar_cloud.width = j;
    lidar_cloud.is_dense = true;
}

void lidar_pose_estimator::get_horizon_angle_range()
{
    PointType p = lidar_cloud.points.front();
    this->min_angle_hori = atan2(p.y, p.x);

    p = lidar_cloud.points.back();
    this->max_angle_hori = atan2(p.y, p.x);
    cout << "horizon angle range: " << min_angle_hori << "  " << max_angle_hori << endl;

    printf("horizon angle\n");
    for (auto i = 0; i < lidar_cloud.points.size(); i++)
    {
        PointType p = lidar_cloud.points[i];
        float angle = atan2(p.y, p.x);
        printf("%f,%f,%f\n", angle, p.x, p.y);
    }

    printf("\nvertical angle\n");
    for (auto i = 0; i < lidar_cloud.points.size(); i++)
    {
        PointType p = lidar_cloud.points[i];
        float dist_hori = sqrtf(p.x * p.x + p.y * p.y);
        float angle = atan2(p.z, dist_hori);
        // printf("%f,", angle);
        printf("%f,%f,%f,%f\n", angle, p.x, p.y, p.z);
    }
}
#endif
