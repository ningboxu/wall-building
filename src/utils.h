#ifndef UTILS_H
#define UTILS_H

#include <glog/logging.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/common/transforms.h>  // 用于 transformPointCloud 函数
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>  // 用于可视化
#include <Eigen/Dense>
#include <Eigen/Geometry>  // For Eigen::Quaternion
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
               std::string file_name, float plane_size = 1.0f,  // if mm
               int resolution = 100);
void ShowPlane(const pcl::PointXYZ& centroid_point,
               const Eigen::Vector4f& coefficients, std::string file_name,
               float plane_size = 1.0f, int resolution = 100);

void ShowVector(const Eigen::Vector3f& vec, std::string name,
                Eigen::Vector3f start_p, float length);
// ------------------保存点云结果----------

// mm2m
void ConvertPointCloudToMeters(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
double computePointToPlaneDistance(
    const Eigen::Vector4f& point,
    const pcl::ModelCoefficients::Ptr& coefficients);
double computePointToPlaneDistance(const Eigen::Vector4f& point,
                                   const Eigen::Vector4f& coefficients);
// 体素滤波
void DownsamplePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                          float leaf_size);
// 在终端可视化
void visualizePointCloudWithVectors(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                    const Eigen::Vector4f& centroid,
                                    const Eigen::Vector3f& major_vector,
                                    const Eigen::Vector3f& middle_vector,
                                    const Eigen::Vector3f& minor_vector);

void filterZAxis(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float z_min,
                 float z_max,
                 pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud);

// 欧拉角转换为四元数 (ZYX顺序)
Eigen::Quaternionf eulerToQuaternion(const Eigen::Vector3f& euler_angles);
// 计算相机到基坐标系的变换矩阵
void computeCameraToBaseTransform(
    const Eigen::Vector3f& translation_TC,  // 工具到相机的平移
    const Eigen::Vector3f& euler_TC,        // 工具到相机的欧拉角
    const Eigen::Vector3f& translation_TB,  // 工具到基坐标系的平移
    const Eigen::Quaternionf& quat_TB,      // 工具到基坐标系的四元数
    Eigen::Vector3f& translation_CB,        // 相机到基坐标系的平移
    Eigen::Quaternionf& quat_CB);

// 计算残差（每个点到平面的距离）
double calculateError(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                      const Eigen::Vector4f& coefficients);

// 使用RANSAC拟合平面
Eigen::Vector4f fitPlaneRANSAC(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

// 使用最小二乘法拟合平面
Eigen::Vector4f fitPlaneLeastSquares(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

// 输出平面系数
void printPlaneCoefficients(const std::string& method_name,
                            const Eigen::Vector4f& coefficients);

#endif  // UTILS_H
