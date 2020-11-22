# kitti dataset

## Overview
The odometry benchmark consists of 22 stereo sequences, saved in loss less png format: We provide 11 sequences (00-10) with ground truth trajectories for training and 11 sequences (11-21) without ground truth for evaluation. For this benchmark you may provide results using monocular or stereo visual odometry, laser-based SLAM or algorithms that combine visual and LIDAR information.

## Data Collection
Our recording platform is a Volkswagen Passat B6, which has been modified with actuators for the pedals (acceleration and brake) and the steering wheel. The data is recorded using an eight core i7 computer equipped with a RAID system, running Ubuntu Linux and a real-time database. We use the following sensors:

* 1 Inertial Navigation System (GPS/IMU): OXTS RT 3003
* 1 Laserscanner: Velodyne HDL-64E
* 2 Grayscale cameras, 1.4 Megapixels: Point Grey Flea 2 (FL2-14S3M-C)
* 2 Color cameras, 1.4 Megapixels: Point Grey Flea 2 (FL2-14S3C-C)
* 4 Varifocal lenses, 4-8 mm: Edmund Optics NT59-917
The laser scanner spins at 10 frames per second, capturing approximately 100k points per cycle. The vertical resolution of the laser scanner is 64. The cameras are mounted approximately level with the ground plane. The camera images are cropped to a size of 1382 x 512 pixels using libdc's format 7 mode. After rectification, the images get slightly smaller. The cameras are triggered at 10 frames per second by the laser scanner (when facing forward) with shutter time adjusted dynamically (maximum shutter time: 2 ms). Our sensor setup with respect to the vehicle is illustrated in the following figure. Note that more information on calibration parameters is given in the calibration files and the development kit (see raw data section).

# lidar pose estimator
rosless pose estimator
* readin lidar data from bin file
* lidar feature extract


## 问题
pcl::removeNaNFromPointCloud(lidar_cloud, lidar_cloud, index);
index 是指valid的点的index

## cloud & Ptr
```
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_Ptr(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ> cloud;
cloud=*cloud_Ptr;
cloud_Ptr=cloud.makeShared;
```

## cloud 拆分成不同的scan，然后每个scan再分组，找edge and planar point 
直接进行分组是不是也是可以的?


## lidar_pose_graph 
```
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