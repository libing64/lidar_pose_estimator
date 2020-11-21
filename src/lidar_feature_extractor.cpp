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
#include <opencv2/opencv.hpp>
#include <mutex>
#include <queue>

using namespace std;
using namespace cv;

typedef pcl::PointXYZI PointType;
#define DEG2RAD (M_PI / 180.0)
int channels = 64;
float fps = 10;
float max_range = 120;
float min_range = 0.1;
float vfov = 26.9 * DEG2RAD;
float vfov_min = -24.9 * DEG2RAD; 
float precise;


std::queue<sensor_msgs::PointCloud2ConstPtr> points_q;

void lidar_feature_extract(pcl::PointCloud<pcl::PointXYZI>::Ptr& points)
{

}

void feature_correspondence()
{


}

void transformation_est()
{

}

void velodyne_points_callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    ROS_INFO_STREAM("velodyne points received" << msg->width << "  " <<  msg->height);
    pcl::PointCloud<PointType>::Ptr pc_raw(new pcl::PointCloud<PointType>());
    ROS_INFO_STREAM("line: " << __LINE__);
    pcl::fromROSMsg(*msg, *pc_raw);

    //return;
    ROS_INFO_STREAM("line: " << __LINE__);
    std::vector<int> index;
    pcl::removeNaNFromPointCloud(*pc_raw, *pc_raw, index);
    ROS_INFO_STREAM("line: " << __LINE__);
    float vres = vfov / (channels - 1);
    int cols = ceil(pc_raw->size() / (float)channels);
    int rows = channels;
    ROS_INFO_STREAM("line: " << __LINE__);
    cv::Mat depth = cv::Mat(rows, cols, CV_32F, cv::Scalar::all(0));
    //imshow("pc_raw", depth);
    //cv::waitKey(2);
    ROS_INFO_STREAM("line: " << __LINE__);
    //project pc_raw to depth image
    for (int i = 0; i < pc_raw->size(); i++)
    {
        PointType p = pc_raw->points[i];
        float hori_dist = sqrtf(p.x * p.x + p.y * p.y);
        float elevation = atan2(p.z, hori_dist);
        float azimuth = atan2(p.y, p.x);
        if (azimuth < 0) azimuth +=  2 * M_PI;
        //64 * 2000
        int row = round((elevation - vfov_min) / vres);
        int col = round(azimuth / (2 * M_PI) * cols);
        depth.at<float>(row, col) = sqrtf(hori_dist * hori_dist + p.z * p.z);
    }
    ROS_INFO_STREAM("line: " << __LINE__);
    // imshow("pc_raw", depth);
    // cv::waitKey(2);
    cout << "depth: " << depth << endl;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lidar_feature_extractor");
    ros::NodeHandle n("~");

    n.getParam("channels", channels);
    n.getParam("max_range", max_range);
    n.getParam("min_range", min_range);
    n.getParam("vfov", vfov);
    n.getParam("vfov_min", vfov_min);

    ROS_INFO_STREAM("channels:" << channels);
    ROS_INFO_STREAM("max_range" << max_range);
    ROS_INFO_STREAM("min_range" << min_range);
    ROS_INFO_STREAM("vfov:" << vfov);
    ROS_INFO_STREAM("vfov_min" << vfov_min);
    ros::Subscriber sub_pointcloud = n.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 100, velodyne_points_callback);

    //ros::Rate rate(100);
    ros::spin();
    return 0;
}