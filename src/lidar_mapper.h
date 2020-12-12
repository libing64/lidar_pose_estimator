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
    /* data */
public:
    lidar_preprocessor lidar;
    lidar_preprocessor lidar_prev;

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

    //transform from lidar_prov to lidar
    Eigen::Quaterniond dq;
    Eigen::Vector3d dt;

    lidar_mapper(/* args */);
    ~lidar_mapper();

    void predict(nav_msgs::Odometry& odom);
    void readin_cloud_data(const sensor_msgs::PointCloud2ConstPtr &edge_points_msg,
                           const sensor_msgs::PointCloud2ConstPtr &planar_points_msg);
    
    void cloud_transform(pcl::PointCloud<PointType>& cloudin, pcl::PointCloud<PointType>&cloudout, Quaterniond q, Vector3d t);
    void update(const sensor_msgs::PointCloud2ConstPtr &edge_points_msg,
                const sensor_msgs::PointCloud2ConstPtr &planar_points_msg);
    void update_feature_map();
    void transform_update();
    void transform_accumulate();
};

lidar_mapper::lidar_mapper(/* args */)
{
    q = Eigen::Quaterniond::Identity();
    t = Eigen::Vector3d::Zero();
    dq = Eigen::Quaterniond::Identity();
    dt = Eigen::Vector3d::Zero();
}

lidar_mapper::~lidar_mapper()
{
}

void lidar_mapper::predict(nav_msgs::Odometry &odom)
{


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
    std::cout << "edge point size prev: " << lidar_prev.edge_points.points.size() << std::endl;
    std::cout << "edge point size: " << lidar.edge_points.points.size() << std::endl;

    //ceres optimization
    double pose[6] = {0, 0, 0, 0, 0, 0}; //0-2 for roation and 3-5 for tranlation
    Problem problem;

    //init paramameter
    double q0[4];
    q0[0] = dq.w();
    q0[1] = dq.x();
    q0[2] = dq.y();
    q0[3] = dq.z();
    ceres::QuaternionToAngleAxis(q0, pose);
    pose[3] = dt(0);
    pose[4] = dt(1);
    pose[5] = dt[2];

    pcl::KdTreeFLANN<PointType> kdtree;
    kdtree.setInputCloud(lidar.edge_points.makeShared());
    int K = 2; // K nearest neighbor search
    std::vector<int> index(K);
    std::vector<float> distance(K);

    Eigen::Matrix3d R = dq.toRotationMatrix();
    //add constraint for edge points
    for (int i = 0; i < lidar_prev.edge_points.points.size(); i++)
    {
        PointType search_point = lidar_prev.edge_points.points[i];
        //project search_point to current frame
        PointType search_point_predict = eigen2point(R * point2eigen(search_point) + dt);
        if (kdtree.nearestKSearch(search_point_predict, K, index, distance) == K)
        {
            //add constraints
            Eigen::Vector3d p = point2eigen(search_point);
            Eigen::Vector3d p1 = point2eigen(lidar.edge_points.points[index[0]]);
            Eigen::Vector3d p2 = point2eigen(lidar.edge_points.points[index[1]]);
            ceres::CostFunction *cost_function = lidar_edge_error::Create(p, p1, p2);
            problem.AddResidualBlock(cost_function,
                                     new CauchyLoss(0.5),
                                     pose);
        }
    }

    //add constraints for planar points
    K = 3;
    kdtree.setInputCloud(lidar.planar_points.makeShared());
    index.resize(K);
    distance.resize(K);
    for (int i = 0; i < lidar_prev.planar_points.points.size(); i++)
    {
        PointType search_point = lidar_prev.planar_points.points[i];
        PointType search_point_predict = eigen2point(R * point2eigen(search_point) + dt);
        if (kdtree.nearestKSearch(search_point_predict, K, index, distance) == K)
        {
            //add constraints
            Eigen::Vector3d p = point2eigen(search_point);
            Eigen::Vector3d p1 = point2eigen(lidar.planar_points.points[index[0]]);
            Eigen::Vector3d p2 = point2eigen(lidar.planar_points.points[index[1]]);
            Eigen::Vector3d p3 = point2eigen(lidar.planar_points.points[index[2]]);
            ceres::CostFunction *cost_function = lidar_planar_error::Create(p, p1, p2, p3);
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
    dt = Eigen::Vector3d(pose[3], pose[4], pose[5]);

    dq.w() = qq[0];
    dq.x() = qq[1];
    dq.y() = qq[2];
    dq.z() = qq[3];
}

void lidar_mapper::transform_accumulate()
{
    //update transformation
    Eigen::Quaterniond dq_inv = dq.inverse();
    Eigen::Vector3d dt_inv = -dq_inv.toRotationMatrix() * dt;

    t += q.toRotationMatrix() * dt_inv;
    q *= dq_inv;
}

void lidar_mapper::update_feature_map()
{
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



    double dt = ((double)clock() - start) / CLOCKS_PER_SEC;
    printf("lidar update cost %lfs", dt);
}

#endif
