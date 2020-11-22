#ifndef _LIDAR_PREPROCESSOR_H
#define _LIDAR_PREPROCESSOR_H

#include <iostream>
#include <vector>
#include <string>
#include <Eigen/Eigen>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/io.h>

using std::cout;
using std::endl;
using std::vector;
using std::string;

typedef pcl::PointXYZI PointType;
bool comp(PointType pi, PointType pj) { return (pi.intensity < pj.intensity); }
class lidar_preprocessor
{
private:
    /* data */
public:
    int channel = 64;
    const float min_range = 0.1;
    const int HALF_CURVA_LEN = 4;
    const int splite_cnt = channel * 5;
    const float edge_point_thresh = 0.2;
    const float planar_point_thresh = 0.05;
    float min_angle_hori;
    float max_angle_hori;
    bool vis_enable = false;

    vector<float> curvature;
    pcl::PointCloud<PointType> lidar_cloud;
    pcl::PointCloud<PointType> edge_points;
    pcl::PointCloud<PointType> planar_points;

    lidar_preprocessor(/* args */);
    ~lidar_preprocessor();

    void readin_lidar_cloud(string filename);
    void readin_lidar_cloud(pcl::PointCloud<PointType> &cloud); //{lidar_cloud = cloud;}
    void inject_invalid_data();
    void remove_invalid_data();
    void get_horizon_angle_range();
    void visualize_cloud();
    void visualize_cloud_data(pcl::PointCloud<PointType>::Ptr ptr, std::string str);
    void get_cloud_curvature();
    float distance(PointType pi, PointType pj);
    float distance(PointType p);
    void get_feature_points();
    void process(string filename);
    void process();
};

lidar_preprocessor::lidar_preprocessor(/* args */)
{
}

lidar_preprocessor::~lidar_preprocessor()
{
}

void lidar_preprocessor::inject_invalid_data()
{
    if (lidar_cloud.points.size() > 0)
    {
        int index = rand() % lidar_cloud.points.size();
        PointType p = lidar_cloud.points[index];
        p.x = 0;
        p.y = 0;
        p.z = 0;

        index = rand() % lidar_cloud.points.size();
        p = lidar_cloud.points[index];
        p.x = NAN;
    }
}

void lidar_preprocessor::readin_lidar_cloud(string filename)
{
    std::ifstream lidar_data_file(filename, std::ifstream::in | std::ifstream::binary);
    lidar_data_file.seekg(0, std::ios::end);
    const size_t num_elements = lidar_data_file.tellg() / sizeof(float);
    lidar_data_file.seekg(0, std::ios::beg);

    std::vector<float> lidar_data(num_elements);
    lidar_data_file.read(reinterpret_cast<char *>(&lidar_data[0]), num_elements * sizeof(float));

    for (int i = 0; i < lidar_data.size(); i += 4)
    {
        PointType p;
        p.x = lidar_data[i];
        p.y = lidar_data[i + 1];
        p.z = lidar_data[i + 2];
        p.intensity = lidar_data[i + 3];
        lidar_cloud.push_back(p);
    }
    lidar_cloud.height = 1;
    lidar_cloud.width = lidar_cloud.points.size();
    lidar_cloud.is_dense = true;
}
void lidar_preprocessor::readin_lidar_cloud(pcl::PointCloud<PointType> &cloud)
{
    //lidar_cloud = cloud;
    //pcl::fromPCLPointCloud2(cloud, lidar_cloud);
    pcl::copyPointCloud(cloud, lidar_cloud);
    //visualize_cloud();
}
void lidar_preprocessor::remove_invalid_data()
{
    //this->visualize_cloud();
    vector<int> index;
    cout << "lidar_cloud size: " << lidar_cloud.points.size() << endl;
    pcl::removeNaNFromPointCloud(lidar_cloud, lidar_cloud, index);

    //this->visualize_cloud();
    cout << "lidar_cloud size: " << lidar_cloud.points.size() << endl;
    int j = 0;
    for (auto i = 0; i < lidar_cloud.points.size(); i++)
    {
        PointType p = lidar_cloud.points[i];
        float dist = sqrtf(p.x * p.x + p.y * p.y + p.z * p.z);
        if (dist > min_range && j < i)
        {
            j++;
            lidar_cloud.points[j] = lidar_cloud.points[i];
        }
    }
    cout << "j: " << j << endl;
    cout << "removed near data cnt: " << (lidar_cloud.points.size() - j) << endl;
    if (j != lidar_cloud.points.size())
    {
        lidar_cloud.points.resize(j);
    }
    //this->visualize_cloud();
    cout << "lidar_cloud size: " << lidar_cloud.points.size() << endl;
    lidar_cloud.height = 1;
    lidar_cloud.width = j;
    lidar_cloud.is_dense = true;
}

void lidar_preprocessor::get_horizon_angle_range()
{
    //this->visualize_cloud();
    PointType p = lidar_cloud.points.front();
    this->min_angle_hori = atan2(p.y, p.x);

    p = lidar_cloud.points.back();
    this->max_angle_hori = atan2(p.y, p.x);
    cout << "horizon angle range: " << min_angle_hori << "  " << max_angle_hori << endl;

    // printf("horizon angle\n");
    // for (auto i = 0; i < lidar_cloud.points.size(); i++)
    // {
    //     PointType p = lidar_cloud.points[i];
    //     float angle = atan2(p.y, p.x);
    //     printf("%f,", angle);
    // }

    // printf("\nvertical angle\n");
    // for (auto i = 0; i < lidar_cloud.points.size(); i++)
    // {
    //     PointType p = lidar_cloud.points[i];
    //     float dist_hori = sqrtf(p.x * p.x + p.y * p.y);
    //     float angle = atan2(p.z, dist_hori);
    //     printf("%f,", angle);
    // }
    //this->visualize_cloud();
}

void lidar_preprocessor::visualize_cloud()
{
    if (vis_enable)
    {
        pcl::visualization::CloudViewer viewer("lidar_cloud");
        pcl::PointCloud<PointType>::Ptr lidar_cloud_ptr(new pcl::PointCloud<PointType>);
        lidar_cloud_ptr = lidar_cloud.makeShared();
        viewer.showCloud(lidar_cloud_ptr);
        while (!viewer.wasStopped())
        {
        }
    }
}

void lidar_preprocessor::visualize_cloud_data(pcl::PointCloud<PointType>::Ptr ptr, std::string str)
{
    if (vis_enable)
    {
        pcl::visualization::CloudViewer viewer(str);
        viewer.showCloud(ptr);
        while (!viewer.wasStopped())
        {
        }
    }
}

float lidar_preprocessor::distance(PointType pi, PointType pj)
{
    float dx = pi.x - pj.x;
    float dy = pi.y - pj.y;
    float dz = pi.z - pj.z;
    float dist = sqrtf(dx * dx + dy * dy + dz * dz);
    return dist;
}

float lidar_preprocessor::distance(PointType p)
{
    float dist = sqrtf(p.x * p.x + p.y * p.y + p.z * p.z);
    return dist;
}

void lidar_preprocessor::get_cloud_curvature()
{
    int n = lidar_cloud.points.size();
    curvature.resize(n);
    for (auto i = HALF_CURVA_LEN; i < lidar_cloud.points.size() - HALF_CURVA_LEN; i++)
    {
        Eigen::Vector3d dp = Eigen::Vector3d::Zero();
        PointType pi = lidar_cloud.points[i];
        float r = distance(pi);
        for (auto j = i - HALF_CURVA_LEN; j <= i + HALF_CURVA_LEN; j++)
        {
            PointType pj = lidar_cloud.points[j];
            dp(0) += (pj.x - pi.x);
            dp(1) += (pj.y - pi.y);
            dp(2) += (pj.z - pi.z);
        }
        curvature[i] = dp.norm() / r;
        lidar_cloud.points[i].intensity = curvature[i]; //curvature visualization
    }
    for (int i = 0; i < HALF_CURVA_LEN; i++)
        curvature[i] = 0;
    for (int i = lidar_cloud.points.size() - HALF_CURVA_LEN; i < lidar_cloud.points.size(); i++)
        curvature[i] = 0;

    // for (int i = 0; i < curvature.size(); i++)
    // {
    //     printf("%f,", curvature[i]);
    // }
    visualize_cloud();
}

void lidar_preprocessor::get_feature_points()
{
    int seg_len = lidar_cloud.points.size() / splite_cnt;
    edge_points.clear();
    planar_points.clear();
    for (int i = 0; i < splite_cnt; i++)
    {
        int left = i * seg_len;
        int right = (i + 1) * seg_len - 1;
        std::sort(lidar_cloud.points.begin() + left, lidar_cloud.points.begin() + right, comp);
        PointType edge = *(lidar_cloud.points.begin() + right);
        PointType planar = *(lidar_cloud.points.begin() + left);
        if (edge.intensity > edge_point_thresh && planar.intensity < planar_point_thresh)
        {
            edge_points.push_back(edge);
            planar_points.push_back(planar);
        }
        else
        {
            //printf("edge curv: %f, planer curv: %f\n", edge.intensity, planar.intensity);
        }
    }

    visualize_cloud_data(edge_points.makeShared(), "edge");
    visualize_cloud_data(planar_points.makeShared(), "planar");
}
void lidar_preprocessor::process(string filename)
{
    readin_lidar_cloud(filename);
    inject_invalid_data(); //TODO remove
    remove_invalid_data();
    get_horizon_angle_range();
    get_cloud_curvature();
    get_feature_points();
}

void lidar_preprocessor::process()
{
    remove_invalid_data();
    get_horizon_angle_range();
    get_cloud_curvature();
    get_feature_points();
}

#endif
