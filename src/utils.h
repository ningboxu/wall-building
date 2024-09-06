#ifndef UTILS_H
#define UTILS_H

#include <glog/logging.h>
#include <pcl/common/transforms.h>  // 用于 transformPointCloud 函数
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <string>

// 控制是否保存点云结果
#define OUTPUT_RESULTS
// 声明保存点云的函数
void SavePointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& pointcloud,
                    std::string name);
void SavePointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pointcloud,
                    std::string name);
void ShowPointQuat(const pcl::PointXYZ& p, const Eigen::Quaternionf& quat,
                   std::string name);
Eigen::Vector3f RotMat2Euler(const Eigen::Matrix3f& R);
void ShowPlane(const pcl::ModelCoefficients::Ptr& plane,
               std::string name = "plane", float start_x = 0, float start_y = 0,
               float plane_x = 0.5, float plane_y = 0.5, int cor = 1);

void ConvertPointCloudToMeters(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
double computePointToPlaneDistance(
    const Eigen::Vector4f& point,
    const pcl::ModelCoefficients::Ptr& coefficients);
void DownsamplePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                          float leaf_size);
// 在终端可视化
void visualizePointCloudWithVectors(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                    const Eigen::Vector4f& centroid,
                                    const Eigen::Vector3f& major_vector,
                                    const Eigen::Vector3f& middle_vector,
                                    const Eigen::Vector3f& minor_vector);
#endif  // UTILS_H
