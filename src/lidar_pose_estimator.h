#ifndef _LIDAR_POSE_ESTIMATOR_H
#define _LIDAR_POSE_ESTIMATOR_H

#include "lidar_preprocessor.h"
#include "lidar_pose_graph.h"
#include <Eigen/Eigen>

Eigen::Vector3d point2eigen(PointType p)
{
    Eigen::Vector3d pp;
    pp(0) = p.x;
    pp(1) = p.y;
    pp(2) = p.z;
    return pp;
}

class lidar_pose_estimator
{
private:
    /* data */
public:
    lidar_preprocessor lidar;
    lidar_preprocessor lidar_prev;

    //transform from lidar_prov to lidar
    Eigen::Quaterniond q;
    Eigen::Vector3d t;

    lidar_pose_estimator(/* args */);
    ~lidar_pose_estimator();

    void transform_estimation();
};

lidar_pose_estimator::lidar_pose_estimator(/* args */)
{
}

lidar_pose_estimator::~lidar_pose_estimator()
{
}

void lidar_pose_estimator::transform_estimation()
{
    pcl::KdTreeFLANN<PointType> kdtree;
    kdtree.setInputCloud(lidar.edge_points.makeShared());
    // K nearest neighbor search
    int K = 2;
    std::vector<int> index(K);
    std::vector<float> distance(K);

    //ceres optimization
    double pose[6] = {0, 0, 0, 0, 0, 0}; //0-2 for roation and 3-5 for tranlation
    Problem problem;

    for (int i = 0; i < lidar_prev.edge_points.points.size(); i++)
    {
        PointType search_point = lidar_prev.edge_points.points[i];
        if (kdtree.nearestKSearch(search_point, K, index, distance) == K)
        {
            //add constraints
            Eigen::Vector3d p = point2eigen(search_point);
            Eigen::Vector3d p1 = point2eigen(lidar.edge_points.points[index[0]]);
            Eigen::Vector3d p2 = point2eigen(lidar.edge_points.points[index[1]]);
            ceres::CostFunction *cost_function = lidar_edge_error::Create(p, p1, p2);
            problem.AddResidualBlock(cost_function,
                                     NULL /* squared loss */,
                                     pose);
            
            std::cout << "p: "  << p.transpose()  << std::endl;
            std::cout << "p1: " << p1.transpose() << std::endl;
            std::cout << "p2: " << p2.transpose() << std::endl;
        }
    }
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.minimizer_progress_to_stdout = true;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.FullReport() << "\n";

    printf("result: %lf, %lf, %lf, %lf, %lf, %lf\n", pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]);

    double qq[4];
    ceres::AngleAxisToQuaternion(pose, qq);
    t = Eigen::Vector3d(pose[3], pose[4], pose[5]);

    q.w() = qq[0];
    q.x() = qq[1];
    q.y() = qq[2];
    q.z() = qq[3];
}

#endif
 