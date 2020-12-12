#ifndef _LIDAR_MAPPER_H
#define _LIDAR_MAPPER_H

#include "lidar_preprocessor.h"
#include "lidar_pose_graph.h"
#include <nav_msgs/Odometry.h>
#include <Eigen/Eigen>

using namespace Eigen;

Eigen::Vector3d point2eigen(PointType p)
{
    Eigen::Vector3d pp;
    pp(0) = p.x;
    pp(1) = p.y;
    pp(2) = p.z;
    return pp;
}

PointType eigen2point(Eigen::Vector3d pp)
{
    PointType p;
    p.x = pp(0);
    p.y = pp(1);
    p.z = pp(2);
    return p;
}

class lidar_mapper
{
private:
    double search_range = 0.5;
public:
    

    //feature map in current frame
    pcl::PointCloud<PointType> edge_point_map;
    pcl::PointCloud<PointType> planar_point_map;


    //edge points and planar points in current frame
    pcl::PointCloud<PointType> edge_points;
    pcl::PointCloud<PointType> planar_points;

    //points projected to global frame
    pcl::PointCloud<PointType> g_edge_points;
    pcl::PointCloud<PointType> g_planar_points;

    //transform  from current frame to init frame
    Eigen::Quaterniond q;
    Eigen::Vector3d t;


    Eigen::Quaterniond qk;
    Eigen::Vector3d tk;
    double timestamp;

    lidar_mapper(/* args */);
    ~lidar_mapper();

    void fit_plane(pcl::PointCloud<PointType> &cloud, Vector3d &center, Vector3d &normal);
    void fit_line(pcl::PointCloud<PointType> &cloud, Vector3d &center, Vector3d &u);
    void odometry2transform(nav_msgs::Odometry& odom, Quaterniond& q, Vector3d& t);
    void predict(nav_msgs::Odometry& odom);
    void readin_cloud_data(const sensor_msgs::PointCloud2ConstPtr &edge_points_msg,
                           const sensor_msgs::PointCloud2ConstPtr &planar_points_msg);
    
    void cloud_transform(pcl::PointCloud<PointType>& cloudin, pcl::PointCloud<PointType>&cloudout, Quaterniond q, Vector3d t);
    void update(const sensor_msgs::PointCloud2ConstPtr &edge_points_msg,
                const sensor_msgs::PointCloud2ConstPtr &planar_points_msg);
    void update_feature_map();
    void transform_update();
};

lidar_mapper::lidar_mapper(/* args */)
{
    q = Eigen::Quaterniond::Identity();
    t = Eigen::Vector3d::Zero();
    qk = q;
    tk = t;
}

lidar_mapper::~lidar_mapper()
{
}

void lidar_mapper::fit_plane(pcl::PointCloud<PointType> &cloud, Vector3d& center, Vector3d& normal)
{
    int n = cloud.points.size();
    VectorXd points = VectorXd::Zero(3, n);
    for (int i = 0; i < 3; i++)
    {
        points.col(i) = point2eigen(cloud.points[i]);
    }
    center = points.rowwise().mean();
    points.colwise() -= center;
    JacobiSVD<MatrixXd> svd(points, ComputeFullU);
    normal = svd.matrixU().col(2);
}

void lidar_mapper::fit_line(pcl::PointCloud<PointType> &cloud, Vector3d &center, Vector3d& u)
{
    int n = cloud.points.size();
    VectorXd points = VectorXd::Zero(3, n);
    for (int i = 0; i < 3; i++)
    {
        points.col(i) = point2eigen(cloud.points[i]);
    }
    center = points.rowwise().mean();
    points.colwise() -= center;
    JacobiSVD<MatrixXd> svd(points, ComputeFullU);
    u = svd.matrixU().col(0);
}

void lidar_mapper::odometry2transform(nav_msgs::Odometry &odom, Quaterniond& q, Vector3d& t)
{
    q.w() = odom.pose.pose.orientation.w;
    q.x() = odom.pose.pose.orientation.x;
    q.y() = odom.pose.pose.orientation.y;
    q.z() = odom.pose.pose.orientation.z;
    t(0) = odom.pose.pose.position.x;
    t(1) = odom.pose.pose.position.y;
    t(2) = odom.pose.pose.position.z;
}
void lidar_mapper::predict(nav_msgs::Odometry &odom)
{
    static bool is_odom_init = false;
    static nav_msgs::Odometry odom_prev;

    if (is_odom_init)
    {
        Quaterniond q1, q2;
        Vector3d t1, t2;
        odometry2transform(odom_prev, q1, t1);
        odometry2transform(odom, q2, t2);

        Quaterniond dq = q1.inverse() * q2;
        Vector3d dt = q.toRotationMatrix() * (t2 - t1);

        qk = q * dq;
        tk = t + dt;
    } else 
    {
        odometry2transform(odom, q, t);
        qk = q;
        tk = t;
        is_odom_init = true;
    }

    odom_prev = odom;
}
void lidar_mapper::readin_cloud_data(const sensor_msgs::PointCloud2ConstPtr &edge_points_msg,
                       const sensor_msgs::PointCloud2ConstPtr &planar_points_msg)
{
    pcl::fromROSMsg(*edge_points_msg, edge_points);
    pcl::fromROSMsg(*planar_points_msg, planar_points);
}

void lidar_mapper::cloud_transform(pcl::PointCloud<PointType> &cloudin, pcl::PointCloud<PointType> &cloudout, Quaterniond q, Vector3d t)
{
    cloudout.resize(cloudin.size());
    Matrix3d R = q.toRotationMatrix();
    for (auto i = 0; i < cloudin.points.size(); i++)
    {
        PointType p = cloudin.points[i];
        cloudout.points[i] = eigen2point(R * point2eigen(p) + t);
    }
}

void lidar_mapper::transform_update()
{
    if (planar_point_map.empty() || edge_point_map.empty())
    {
        return;
    }
    std::cout << "edge point size: " << edge_points.points.size() << std::endl;
    std::cout << "edge point map size: " << edge_point_map.points.size() << std::endl;

    std::cout << "planar point size: " << planar_points.points.size() << std::endl;
    std::cout << "planar point map size: " << planar_point_map.points.size() << std::endl;
    //ceres optimization
    double pose[6] = {0, 0, 0, 0, 0, 0}; //0-2 for roation and 3-5 for tranlation
    Problem problem;

    pcl::KdTreeFLANN<PointType> kdtree;
    kdtree.setInputCloud(edge_point_map.makeShared());
    int K = 2; // K nearest neighbor search
    std::vector<int> index;
    std::vector<float> distance;
    double radius = search_range;

    //add constraint for edge points
    for (int i = 0; i < g_edge_points.points.size(); i++)
    {
        PointType search_point = g_edge_points.points[i];
        pcl::PointCloud<PointType> searched_points;
        if (kdtree.radiusSearch(search_point, radius, index, distance) >= K)
        {
            for (int j = 0; j < index.size(); j++)
            {
                searched_points.points.push_back(edge_point_map.points[index[j]]);
            }
            //add constraints
            Eigen::Vector3d p = point2eigen(search_point);
            Vector3d center, u;
            fit_line(searched_points, center, u);

            ceres::CostFunction *cost_function = lidar_line_error::Create(p, center, u);
            problem.AddResidualBlock(cost_function,
                                     new CauchyLoss(0.5),
                                     pose);
        }
    }

    //add constraints for planar points
    K = 3;
    kdtree.setInputCloud(planar_point_map.makeShared());
    for (int i = 0; i < g_planar_points.points.size(); i++)
    {
        PointType search_point = g_planar_points.points[i];
        pcl::PointCloud<PointType> searched_points;
        if (kdtree.nearestKSearch(search_point, radius, index, distance) >= K)
        {
            //add constraints
            for (int j = 0; j < index.size(); j++)
            {
                searched_points.points.push_back(edge_point_map.points[index[j]]);
            }
            Vector3d center, normal;
            fit_plane(searched_points, center, normal);
            Eigen::Vector3d p = point2eigen(search_point);
            ceres::CostFunction *cost_function = lidar_plane_error::Create(p, center, normal);
            problem.AddResidualBlock(cost_function,
                                     new CauchyLoss(0.5),
                                     pose);
        }
    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.minimizer_progress_to_stdout = false;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    //std::cout << summary.FullReport() << "\n";

    //printf("result: %lf, %lf, %lf, %lf, %lf, %lf\n", pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]);

    double qq[4];
    ceres::AngleAxisToQuaternion(pose, qq);
    Vector3d dt = Eigen::Vector3d(pose[3], pose[4], pose[5]);

    Quaterniond dq;
    dq.w() = qq[0];
    dq.x() = qq[1];
    dq.y() = qq[2];
    dq.z() = qq[3];

    q = dq * q;
    t = dt + dq.toRotationMatrix() * t;

    qk = q;
    tk = t;
}

void lidar_mapper::update_feature_map()
{

    cloud_transform(edge_points, g_edge_points, qk, tk);
    cloud_transform(planar_points, g_planar_points, qk, tk);

    //project features to current frame
    for (auto i = 0; i < g_edge_points.size(); i++)
    {
        edge_point_map.points.push_back(g_edge_points.points[i]);
    }

    for (auto i = 0; i < g_planar_points.size(); i++)
    {
        planar_point_map.points.push_back(g_planar_points.points[i]);
    }
}

void lidar_mapper::update(const sensor_msgs::PointCloud2ConstPtr &edge_points_msg,
                          const sensor_msgs::PointCloud2ConstPtr &planar_points_msg)
{
    clock_t start = clock();
    timestamp = edge_points_msg->header.stamp.toSec();

    readin_cloud_data(edge_points_msg, planar_points_msg);

    cloud_transform(edge_points, g_edge_points, qk, tk);
    cloud_transform(planar_points, g_planar_points, qk, tk);

    transform_update();

    update_feature_map();

    double dt = ((double)clock() - start) / CLOCKS_PER_SEC;
    printf("lidar update cost %lfs", dt);
}

#endif
