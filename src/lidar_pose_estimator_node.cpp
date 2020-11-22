#include "lidar_pose_estimator.h"

#include <cmath>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/transformation_estimation.h>
#include <pcl/registration/transformation_estimation_lm.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <Eigen/Eigen>

lidar_pose_estimator estimator;

void velodyne_points_callback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    estimator.update(msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lidar_pose_estimator");
    ros::NodeHandle nh;

    ros::Subscriber sub_pointcloud = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 100, velodyne_points_callback);

    //ros::Rate rate(100);
    ros::spin();
    return 0;
}