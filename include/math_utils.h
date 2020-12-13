#ifndef __MATH_UTILS_H
#define __MATH_UTILS_H

#include "lidar_preprocessor.h"
#include <Eigen/Eigen>


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

#endif