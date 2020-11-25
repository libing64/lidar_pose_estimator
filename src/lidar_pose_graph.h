#ifndef _LIDAR_POSE_GRAPH_H
#define _LIDAR_POSE_GRAPH_H

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <ceres/dynamic_autodiff_cost_function.h>
#include <Eigen/Eigen>

using ceres::AutoDiffCostFunction;
using ceres::CauchyLoss;
using ceres::CostFunction;
using ceres::DynamicAutoDiffCostFunction;
using ceres::LossFunction;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;
using ceres::AngleAxisRotatePoint;
using ceres::CrossProduct;
using ceres::DotProduct;

//parameters: rotation and translation
//rotation in angleAxis format

struct lidar_edge_error
{
    lidar_edge_error(const Eigen::Vector3d p, const Eigen::Vector3d p1, const Eigen::Vector3d p2)
        : p(p), p1(p1), p2(p2) {}

    template <typename T>
    bool operator()(const T *const pose,
                    T *residuals) const
    {
        // pose[0,1,2] are the angle-axis rotation.
        T pi[3];
        pi[0] = T(p(0));
        pi[1] = T(p(1));
        pi[2] = T(p(2));

        T pi_proj[3];//project pi to current frame
        AngleAxisRotatePoint(pose, pi, pi_proj);

        // pose[3,4,5] are the translation.
        pi_proj[0] += pose[3];
        pi_proj[1] += pose[4];
        pi_proj[2] += pose[5];

        //distance between pi_proj to line(p1, p2)
        T d1[3], d2[3], d12[3];
        d1[0] = pi_proj[0] - T(p1(0));
        d1[1] = pi_proj[1] - T(p1(1));
        d1[2] = pi_proj[2] - T(p1(2));

        d2[0] = pi_proj[0] - T(p2(0));
        d2[1] = pi_proj[1] - T(p2(1));
        d2[2] = pi_proj[2] - T(p2(2));

        d12[0] = T(p1(0) - p2(0));
        d12[1] = T(p1(1) - p2(1));
        d12[2] = T(p1(2) - p2(2));

        T cross[3];
        CrossProduct(d1, d2, cross);

        T norm = sqrt(cross[0] * cross[0] + cross[1] * cross[1] + cross[2] * cross[2]);
        T norm12 = sqrt(d12[0] * d12[0] + d12[1] * d12[1] + d12[2] * d12[2]);
        T weight = T(10.0);
        residuals[0] = weight * norm / norm12;
        return true;
    }

    // Factory to hide the construction of the CostFunction object from
    // the client code.
    static ceres::CostFunction *Create(const Eigen::Vector3d p, const Eigen::Vector3d p1, const Eigen::Vector3d p2)
    {
        return (new ceres::AutoDiffCostFunction<lidar_edge_error, 1, 6>(
            new lidar_edge_error(p, p1, p2)));
    }


    //project point p to line (p1 - p2)
    Eigen::Vector3d p;
    Eigen::Vector3d p1;
    Eigen::Vector3d p2;
};

struct lidar_planar_error
{
    lidar_planar_error(const Eigen::Vector3d p, const Eigen::Vector3d p1, const Eigen::Vector3d p2, const Eigen::Vector3d p3)
        : p(p), p1(p1), p2(p2), p3(p3) {}

    template <typename T>
    bool operator()(const T *const pose,
                    T *residuals) const
    {
        // pose[0,1,2] are the angle-axis rotation.
        T pi[3];
        pi[0] = T(p(0));
        pi[1] = T(p(1));
        pi[2] = T(p(2));

        T pi_proj[3]; //project pi to current frame
        AngleAxisRotatePoint(pose, pi, pi_proj);

        // pose[3,4,5] are the translation.
        pi_proj[0] += pose[3];
        pi_proj[1] += pose[4];
        pi_proj[2] += pose[5];


        //normal vector of plane (p1, p2, p3)
        T d12[3], d13[3];
        d12[0] = T(p1(0) - p2(0));
        d12[1] = T(p1(1) - p2(1));
        d12[2] = T(p1(2) - p2(2));

        d13[0] = T(p1(0) - p3(0));
        d13[1] = T(p1(1) - p3(1));
        d13[2] = T(p1(2) - p3(2));

        T cross[3], normal_vec[3];
        CrossProduct(d12, d13, cross);
        T norm = sqrt(cross[0] * cross[0] + cross[1] * cross[1] + cross[2] * cross[2]);
        normal_vec[0] = cross[0] / norm;
        normal_vec[1] = cross[1] / norm;
        normal_vec[2] = cross[2] / norm;

        //distance between pi_proj to plane(p1, p2, p3)
        T d1[3];
        d1[0] = pi_proj[0] - T(p1(0));
        d1[1] = pi_proj[1] - T(p1(1));
        d1[2] = pi_proj[2] - T(p1(2));

        residuals[0] = DotProduct(d1, normal_vec);
        return true;
    }

    // Factory to hide the construction of the CostFunction object from
    // the client code.
    static ceres::CostFunction *Create(const Eigen::Vector3d p, const Eigen::Vector3d p1, const Eigen::Vector3d p2, const Eigen::Vector3d p3)
    {
        return (new ceres::AutoDiffCostFunction<lidar_planar_error, 1, 6>(
            new lidar_planar_error(p, p1, p2, p3)));
    }

    //project point p to plane (p1 - p2 - p3)
    Eigen::Vector3d p;
    Eigen::Vector3d p1;
    Eigen::Vector3d p2;
    Eigen::Vector3d p3;
};

#endif