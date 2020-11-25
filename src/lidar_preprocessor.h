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
    const int splite_cnt = channel * 10;
    const float edge_point_thresh = 0.1;
    const float planar_point_thresh = 0.01;
    const float min_dist_thresh = 0.1;//min distance for selecting features
    bool vis_enable = false;

    vector<float> curvature;
    pcl::PointCloud<PointType> lidar_cloud;
    pcl::PointCloud<PointType> edge_points;
    pcl::PointCloud<PointType> planar_points;

    lidar_preprocessor();
    ~lidar_preprocessor();

    void set_cloud_vis(bool vis){vis_enable = vis;}
    void readin_lidar_cloud(string filename);
    void remove_invalid_data();

    void visualize_cloud(pcl::PointCloud<PointType>::Ptr ptr, std::string str);
    void get_cloud_curvature();
    float distance(PointType p);
    void get_feature_points();
    void remove_invalid_points(pcl::PointCloud<PointType> &cloud, vector<int>& valid_index);
    void remove_neighbor_feature(pcl::PointCloud<PointType> &cloud);
    void process(string filename);
    void process(const sensor_msgs::PointCloud2ConstPtr &msg);
};

lidar_preprocessor::lidar_preprocessor()
{
}

lidar_preprocessor::~lidar_preprocessor()
{
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

void lidar_preprocessor::remove_invalid_data()
{
    vector<int> valid_index(lidar_cloud.points.size());
    cout << "lidar_cloud size: " << lidar_cloud.points.size() << endl;
    for (auto i = 0; i < lidar_cloud.points.size(); i++)
    {
        PointType p = lidar_cloud.points[i];
        if (isnan(p.x) || isnan(p.y) || isnan(p.z))
        {
            valid_index[i] = 0;
        } else 
        {
            float dist = sqrtf(p.x * p.x + p.y * p.y + p.z * p.z);
            if (dist < min_range)
            {
                valid_index[i] = 0;
            } else 
            {
                valid_index[i] = 1;
            }
        }

    }
    remove_invalid_points(lidar_cloud, valid_index);
    cout << "lidar_cloud size: " << lidar_cloud.points.size() << endl;
    lidar_cloud.height = 1;
    lidar_cloud.width = lidar_cloud.points.size();
    lidar_cloud.is_dense = true;
}

void lidar_preprocessor::visualize_cloud(pcl::PointCloud<PointType>::Ptr ptr, std::string str)
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
    {
        curvature[i] = (edge_point_thresh + planar_point_thresh) / 2;
        lidar_cloud.points[i].intensity = curvature[i];
    }

    for (int i = lidar_cloud.points.size() - HALF_CURVA_LEN; i < lidar_cloud.points.size(); i++)
    {
        curvature[i] = (edge_point_thresh + planar_point_thresh) / 2;
        lidar_cloud.points[i].intensity = curvature[i];
    }
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
        PointType planar2 = *(lidar_cloud.points.begin() + left + 1);
        if (edge.intensity > edge_point_thresh )
        {
            edge_points.push_back(edge);
        }
        if (planar.intensity < planar_point_thresh)
        {
            planar_points.push_back(planar);
        }
        if (planar2.intensity < planar_point_thresh)
        {
            planar_points.push_back(planar2);
        }
    }

    visualize_cloud(edge_points.makeShared(), "edge");
    visualize_cloud(planar_points.makeShared(), "planar");
}

void lidar_preprocessor::remove_invalid_points(pcl::PointCloud<PointType> &cloud, vector<int>& valid_index)
{
    std::cout << "cloud size 1: " << cloud.points.size() << std::endl;
    int j = 0;
    for (auto i = 0; i < cloud.points.size(); i++)
    {
        if (valid_index[i])
        {
            j++;
            if (j < i)
            {
                cloud.points[j] = cloud.points[i];
            }

        }
    }
    if (j != cloud.points.size())
    {
        cloud.points.resize(j);
    }
    std::cout << "cloud size 2: " << cloud.points.size() << std::endl;
}

void lidar_preprocessor::remove_neighbor_feature(pcl::PointCloud<PointType> &cloud)
{
    if (cloud.points.empty()) return;
    vector<int> valid_index(cloud.points.size());
    for (int i = 0; i < valid_index.size(); i++)
    {
        valid_index[i] = 1;
    }
    //remove neighbor feature with kdtree
    pcl::KdTreeFLANN<PointType> kdtree;
    kdtree.setInputCloud(cloud.makeShared());
    std::vector<int> index;
    std::vector<float> distance;

    float radius = min_dist_thresh;
    for (int i = 0; i < cloud.size(); i++)
    {
        if (valid_index[i])
        {
            PointType search_point = cloud.points[i];
            if (kdtree.radiusSearch(search_point, radius, index, distance) > 0)
            {
                //remove neighbor points
                for (int j = 0; j < index.size(); j++)
                {
                    if (index[j] != i)
                    {
                        valid_index[index[j]] = 0;
                    }
                }
            }
        }
    }
    remove_invalid_points(cloud, valid_index);

}
void lidar_preprocessor::process(string filename)
{
    readin_lidar_cloud(filename);
    remove_invalid_data();
    get_cloud_curvature();
    get_feature_points();
    remove_neighbor_feature(edge_points);
    remove_neighbor_feature(planar_points);
}

void lidar_preprocessor::process(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    pcl::fromROSMsg(*msg, lidar_cloud);
    remove_invalid_data();
    get_cloud_curvature();
    get_feature_points();
    remove_neighbor_feature(edge_points);
    remove_neighbor_feature(planar_points);
}

#endif
