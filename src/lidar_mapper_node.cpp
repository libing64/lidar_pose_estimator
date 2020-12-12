#include "lidar_mapper.h"

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
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <Eigen/Eigen>

using namespace sensor_msgs;
using namespace message_filters;
using namespace nav_msgs;

lidar_mapper mapper;
ros::Publisher pub_odom, pub_pose, pub_path;
void publish_odom(lidar_mapper& mapper);
void publish_pose(lidar_mapper &mapper);

void cloud_points_callback(const sensor_msgs::PointCloud2ConstPtr &edge_points_msg,
                           const sensor_msgs::PointCloud2ConstPtr &planar_points_msg, 
                           const nav_msgs::OdometryConstPtr& odom_msg)
{
    cout << "cloud msg received" << endl;
}

void publish_odom(lidar_mapper& mapper)
{
    nav_msgs::Odometry odometry;
    odometry.header.frame_id = "odom";
    odometry.header.stamp = ros::Time(mapper.timestamp);

    odometry.child_frame_id = "base_link";
    odometry.pose.pose.position.x = mapper.t(0);
    odometry.pose.pose.position.y = mapper.t(1);
    odometry.pose.pose.position.z = mapper.t(2);
    odometry.pose.pose.orientation.x = mapper.q.x();
    odometry.pose.pose.orientation.y = mapper.q.y();
    odometry.pose.pose.orientation.z = mapper.q.z();
    odometry.pose.pose.orientation.w = mapper.q.w();
    pub_odom.publish(odometry);
}

void publish_pose(lidar_mapper &mapper)
{
    static nav_msgs::Path path;
    geometry_msgs::PoseStamped pose;
    Eigen::Quaterniond q = mapper.q;
    Eigen::Vector3d p = mapper.t;

    pose.header.stamp = ros::Time(mapper.timestamp);
    pose.header.frame_id = "odom";
    pose.pose.orientation.w = q.w();
    pose.pose.orientation.x = q.x();
    pose.pose.orientation.y = q.y();
    pose.pose.orientation.z = q.z();
    pose.pose.position.x = p(0);
    pose.pose.position.y = p(1);
    pose.pose.position.z = p(2);
    pub_pose.publish(pose);

    //cout << "publish pose: " << endl;
    //_pose.print_state();
    path.header.frame_id = "odom";
    path.poses.push_back(pose);
    pub_path.publish(path);

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lidar_mapper");
    ros::NodeHandle nh;

    pub_odom = nh.advertise<nav_msgs::Odometry>("/odom", 10);

    message_filters::Subscriber<PointCloud2> sub_edge_points(nh, "/edge_points", 1);
    message_filters::Subscriber<PointCloud2> sub_planar_points(nh, "/planar_points", 1);
    message_filters::Subscriber<Odometry> sub_odmoetry(nh, "/odom", 1);
    TimeSynchronizer<PointCloud2, PointCloud2, Odometry> sync(sub_edge_points, sub_planar_points, sub_odmoetry, 100);
    sync.registerCallback(boost::bind(&cloud_points_callback, _1, _2, _3));

    pub_pose = nh.advertise<geometry_msgs::PoseStamped>("/pose_mapper", 10);
    pub_path = nh.advertise<nav_msgs::Path>("/path_mapper", 10);
    //ros::Rate rate(100);
    ros::spin();
    return 0;
}