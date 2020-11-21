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