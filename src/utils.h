#ifndef UTILS_H
#define UTILS_H

#include <glog/logging.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/common/transforms.h>  // 用于 transformPointCloud 函数
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>  // 用于可视化
#include <Eigen/Dense>
#include <iostream>
#include <string>

// !控制是否保存点云结果
#define OUTPUT_RESULTS

// ------------------保存点云结果----------
void SavePointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& pointcloud,
                    std::string name);
void SavePointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pointcloud,
                    std::string name);
void ShowPointQuat(const pcl::PointXYZ& p, const Eigen::Quaternionf& quat,
                   std::string name);
Eigen::Vector3f RotMat2Euler(const Eigen::Matrix3f& R);
void ShowPlane(const pcl::PointXYZ& centroid_point,
               const pcl::ModelCoefficients::Ptr& coefficients,
               std::string file_name, float plane_size = 1.0f,
               int resolution = 100);
void ShowVector(const Eigen::Vector3f& vec, std::string name,
                Eigen::Vector3f start_p, float length);
// ------------------保存点云结果----------

// mm2m
void ConvertPointCloudToMeters(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
double computePointToPlaneDistance(
    const Eigen::Vector4f& point,
    const pcl::ModelCoefficients::Ptr& coefficients);
// 体素滤波
void DownsamplePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                          float leaf_size);
// 在终端可视化
void visualizePointCloudWithVectors(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                    const Eigen::Vector4f& centroid,
                                    const Eigen::Vector3f& major_vector,
                                    const Eigen::Vector3f& middle_vector,
                                    const Eigen::Vector3f& minor_vector);
#endif  // UTILS_H
