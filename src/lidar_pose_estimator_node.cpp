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
ros::Publisher pub_odom, pub_pose, pub_path;
void publish_odom(lidar_pose_estimator& est);
void publish_pose(lidar_pose_estimator &est);

void velodyne_points_callback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    estimator.update(msg);
    publish_odom(estimator);
    publish_pose(estimator);
}

void publish_odom(lidar_pose_estimator& est)
{
    nav_msgs::Odometry odometry;
    odometry.header.frame_id = "odom";
    odometry.header.stamp = ros::Time(est.timestamp);

    odometry.child_frame_id = "base_link";
    odometry.pose.pose.position.x = est.t(0);
    odometry.pose.pose.position.y = est.t(1);
    odometry.pose.pose.position.z = est.t(2);
    odometry.pose.pose.orientation.x = est.q.x();
    odometry.pose.pose.orientation.y = est.q.y();
    odometry.pose.pose.orientation.z = est.q.z();
    odometry.pose.pose.orientation.w = est.q.w();
    pub_odom.publish(odometry);
}

void publish_pose(lidar_pose_estimator &est)
{
    static nav_msgs::Path path;
    geometry_msgs::PoseStamped pose;
    Eigen::Quaterniond q = est.q;
    Eigen::Vector3d p = est.t;

    pose.header.stamp = ros::Time(est.timestamp);
    pose.header.frame_id = "odom";
    pose.pose.orientation.w = q.w();
    pose.pose.orientation.x = q.x();
    pose.pose.orientation.y = q.y();
    pose.pose.orientation.z = q.z();
    pose.pose.position.x = -p(0);
    pose.pose.position.y = -p(1);
    pose.pose.position.z = -p(2);
    pub_pose.publish(pose);

    //cout << "publish pose: " << endl;
    //_pose.print_state();
    path.header.frame_id = "odom";
    path.poses.push_back(pose);
    pub_path.publish(path);

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lidar_pose_estimator");
    ros::NodeHandle nh;

    ros::Subscriber sub_pointcloud = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 100, velodyne_points_callback);
    pub_odom = nh.advertise<nav_msgs::Odometry>("/odom", 10);

    pub_pose = nh.advertise<geometry_msgs::PoseStamped>("/est_pose", 10);
    pub_path = nh.advertise<nav_msgs::Path>("/path", 10);
    //ros::Rate rate(100);
    ros::spin();
    return 0;
}