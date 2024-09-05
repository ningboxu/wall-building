#ifndef UTILS_H
#define UTILS_H

#include <glog/logging.h>
#include <pcl/common/transforms.h>  // 用于 transformPointCloud 函数
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <string>

// 声明保存点云的函数
void SavePointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& pointcloud,
                    std::string name);
void SavePointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pointcloud,
                    std::string name);
void ShowPointQuat(const pcl::PointXYZ& p, const Eigen::Quaternionf& quat,
                   std::string name);
Eigen::Vector3f RotMat2Euler(const Eigen::Matrix3f& R);

#endif  // UTILS_H
