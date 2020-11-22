
# 1. lidar_pose_estimator

## 1.1 steps
* readin and publish lidar data
* extract feature data (edge feature and planar feature)
* estimate transformation between two frame
* update transformation
* publish result
* mapping


# 2. problems

## 2.1 Conversion between PointCloud and Ptr 
```
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_Ptr(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ> cloud;
cloud=*cloud_Ptr;
cloud_Ptr=cloud.makeShared();
```



## 2.2 edge_pose_graph 
example for pose_graph in utest/lidar_pose_graph_example.cpp

```
rosrun lidar_pose_estimator lidar_pose_graph_example 


groundtruth: 
angle_axis: -1.69742 0.717996 -1.12579
trans:    0.747958 -0.00371215    0.152399
iter      cost      cost_change  |gradient|   |step|    tr_ratio  tr_radius  ls_iter  iter_time  total_time
   0  5.445473e+05    0.00e+00    1.16e+05   0.00e+00   0.00e+00  1.00e+04        0    3.19e-05    9.66e-05
   1  1.921381e+05    3.52e+05    2.73e+05   3.04e+01   2.91e+00  3.00e+04        0    8.27e-05    1.92e-04
   2  6.196413e+03    1.86e+05    4.81e+04   3.26e+01   1.02e+00  9.00e+04        0    5.67e-05    2.56e-04
   3  8.448327e-01    6.20e+03    4.63e+02   6.66e+00   1.00e+00  2.70e+05        0    5.56e-05    3.16e-04
   4  3.713054e-08    8.45e-01    1.43e-01   1.08e-01   1.00e+00  8.10e+05        0    5.52e-05    3.76e-04
   5  4.365415e-20    3.71e-08    1.59e-07   1.51e-05   1.00e+00  2.43e+06        0    5.54e-05    4.36e-04

Solver Summary (v 1.14.0-eigen-(3.3.7)-lapack-suitesparse-(5.7.1)-cxsparse-(3.2.0)-eigensparse-openmp-no_tbb)

                                     Original                  Reduced
Parameter blocks                            1                        1
Parameters                                  6                        6
Residual blocks                           100                      100
Residuals                                 100                      100

Minimizer                        TRUST_REGION

Dense linear algebra library            EIGEN
Trust region strategy     LEVENBERG_MARQUARDT

                                        Given                     Used
Linear solver                     DENSE_SCHUR              DENSE_SCHUR
Threads                                     1                        1
Linear solver ordering              AUTOMATIC                        1
Schur structure                         1,6,0                    d,d,d

Cost:
Initial                          5.445473e+05
Final                            4.365415e-20
Change                           5.445473e+05

Minimizer iterations                        6
Successful steps                            6
Unsuccessful steps                          0

Time (in seconds):
Preprocessor                         0.000065

  Residual only evaluation           0.000058 (6)
  Jacobian & residual evaluation     0.000140 (6)
  Linear solver                      0.000131 (6)
Minimizer                            0.000409

Postprocessor                        0.000001
Total                                0.000475

Termination:                      CONVERGENCE (Parameter tolerance reached. Relative step_norm: 2.934019e-12 <= 1.000000e-08.)

result: -1.697416, 0.717996, -1.125791, 0.747958, -0.003712, 0.152399

```

## 2.3 planar pose graph

example for pose_graph in utest/lidar_pose_graph_example.cpp
```
rosrun lidar_pose_estimator lidar_pose_graph_example 

groundtruth: 
angle_axis: -1.69742 0.717996 -1.12579
trans:    0.747958 -0.00371215    0.152399
iter      cost      cost_change  |gradient|   |step|    tr_ratio  tr_radius  ls_iter  iter_time  total_time
   0  2.414344e+05    0.00e+00    2.18e+04   0.00e+00   0.00e+00  1.00e+04        0    3.38e-05    8.86e-05
   1  2.307385e+05    1.07e+04    1.62e+04   2.22e+01   1.12e+00  3.00e+04        0    1.18e-04    2.17e-04
   2  2.283459e+05    2.39e+03    1.77e+04   9.11e-01   2.03e+00  9.00e+04        0    9.72e-05    3.22e-04
   3  2.257612e+05    2.58e+03    1.98e+04   9.57e-01   2.05e+00  2.70e+05        0    8.50e-05    4.13e-04
   4  2.228015e+05    2.96e+03    2.28e+04   1.16e+00   2.09e+00  8.10e+05        0    7.04e-05    4.90e-04
   5  2.189365e+05    3.86e+03    2.72e+04   1.56e+00   2.20e+00  2.43e+06        0    7.07e-05    5.67e-04
   6  2.123916e+05    6.54e+03    3.35e+04   2.25e+00   2.41e+00  7.29e+06        0    7.06e-05    6.44e-04
   7  1.948018e+05    1.76e+04    6.06e+04   3.50e+00   2.81e+00  2.19e+07        0    7.06e-05    7.20e-04
   8  1.114366e+05    8.34e+04    1.31e+05   6.34e+00   3.13e+00  6.56e+07        0    6.98e-05    7.96e-04
   9  6.941095e+03    1.04e+05    5.16e+04   8.97e+00   1.21e+00  1.97e+08        0    7.15e-05    8.74e-04
  10  1.681306e+00    6.94e+03    4.49e+02   3.14e+00   1.00e+00  5.90e+08        0    8.11e-05    9.61e-04
  11  1.864407e-07    1.68e+00    2.43e-01   2.39e-01   1.00e+00  1.77e+09        0    8.43e-05    1.05e-03
  12  2.179418e-21    1.86e-07    1.83e-08   3.08e-05   1.00e+00  5.31e+09        0    6.97e-05    1.13e-03

Solver Summary (v 1.14.0-eigen-(3.3.7)-lapack-suitesparse-(5.7.1)-cxsparse-(3.2.0)-eigensparse-openmp-no_tbb)

                                     Original                  Reduced
Parameter blocks                            1                        1
Parameters                                  6                        6
Residual blocks                           100                      100
Residuals                                 100                      100

Minimizer                        TRUST_REGION

Dense linear algebra library            EIGEN
Trust region strategy     LEVENBERG_MARQUARDT

                                        Given                     Used
Linear solver                     DENSE_SCHUR              DENSE_SCHUR
Threads                                     1                        1
Linear solver ordering              AUTOMATIC                        1
Schur structure                         1,6,0                    d,d,d

Cost:
Initial                          2.414344e+05
Final                            2.179418e-21
Change                           2.414344e+05

Minimizer iterations                       13
Successful steps                           13
Unsuccessful steps                          0

Time (in seconds):
Preprocessor                         0.000055

  Residual only evaluation           0.000122 (13)
  Jacobian & residual evaluation     0.000332 (13)
  Linear solver                      0.000477 (13)
Minimizer                            0.001123

Postprocessor                        0.000002
Total                                0.001180

Termination:                      CONVERGENCE (Parameter tolerance reached. Relative step_norm: 2.689042e-12 <= 1.000000e-08.)

result: -1.697416, 0.717996, -1.125791, 0.747958, -0.003712, 0.152399


```

## 2.4 kdtree
```
rosrun lidar_pose_estimator kdtree_example 
K nearest neighbor search at (715.095 814.236 980.555) with K=10
    733.488 761.876 941.759 (squared distance: 4585.04)
    760.975 867.405 977.911 (squared distance: 4938.91)
    663.906 735.162 979.46 (squared distance: 8874.22)
    669.45 723.308 924.492 (squared distance: 13494.3)
    825.581 868.644 979.364 (squared distance: 15168.8)
    705.924 933.798 916.208 (squared distance: 18519.7)
    574.16 836.271 972.582 (squared distance: 20412)
    763.09 810.1 841.056 (squared distance: 21780.7)
    710.103 697.298 885.781 (squared distance: 22681.7)
    591.081 863.471 896.625 (squared distance: 24847.8)
Neighbors within radius search at (715.095 814.236 980.555) with radius=24.6792
```


# 3. TODO

## lidar pose estimator with edge point only


## velodyne coordinate


## how to make the system more robust?
- [x] cauchy loss for ceres optimization
- [ ] position result is negative of groundtruth
- [x] drfit fast if noly edge point for constraints, add planar points for constraints
- [ ] reduce drift, add feature maps
- [ ] scale not accurate
- [x] record screen to gif


## 4. record screen with byzanz
```
byzanz-record --delay 10 -d 30
```


# 5. Kitti dataset

## 5.1 Overview
The odometry benchmark consists of 22 stereo sequences, saved in loss less png format: We provide 11 sequences (00-10) with ground truth trajectories for training and 11 sequences (11-21) without ground truth for evaluation. For this benchmark you may provide results using monocular or stereo visual odometry, laser-based SLAM or algorithms that combine visual and LIDAR information.

## 5.2 Data Collection
Our recording platform is a Volkswagen Passat B6, which has been modified with actuators for the pedals (acceleration and brake) and the steering wheel. The data is recorded using an eight core i7 computer equipped with a RAID system, running Ubuntu Linux and a real-time database. We use the following sensors:

* 1 Inertial Navigation System (GPS/IMU): OXTS RT 3003
* 1 Laserscanner: Velodyne HDL-64E
* 2 Grayscale cameras, 1.4 Megapixels: Point Grey Flea 2 (FL2-14S3M-C)
* 2 Color cameras, 1.4 Megapixels: Point Grey Flea 2 (FL2-14S3C-C)
* 4 Varifocal lenses, 4-8 mm: Edmund Optics NT59-917
The laser scanner spins at 10 frames per second, capturing approximately 100k points per cycle. The vertical resolution of the laser scanner is 64. The cameras are mounted approximately level with the ground plane. The camera images are cropped to a size of 1382 x 512 pixels using libdc's format 7 mode. After rectification, the images get slightly smaller. The cameras are triggered at 10 frames per second by the laser scanner (when facing forward) with shutter time adjusted dynamically (maximum shutter time: 2 ms). Our sensor setup with respect to the vehicle is illustrated in the following figure. Note that more information on calibration parameters is given in the calibration files and the development kit (see raw data section).

## 5.3 velodyne coordinate
direction of x y z: right-forward-up


# 6. reference
