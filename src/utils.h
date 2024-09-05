#ifndef UTILS_H
#define UTILS_H

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <string>

// 声明保存点云的函数
void SavePointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& pointcloud,
                    std::string name);

#endif  // UTILS_H
