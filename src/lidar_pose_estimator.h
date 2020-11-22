#ifndef _LIDAR_POSE_ESTIMATOR_H
#define _LIDAR_POSE_ESTIMATOR_H

#include "lidar_preprocessor.h"


class lidar_pose_estimator
{
private:
    /* data */
public:
    lidar_preprocessor lidar;
    lidar_preprocessor lidar_prev;

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
    //find closest point 
}

#endif
 