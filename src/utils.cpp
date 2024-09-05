#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <iostream>
#include "utils.h"

void SavePointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& pointcloud,
                    std::string name)
{
    // 保存到当前编译路径（build目录）
    std::string save_path = name + ".pcd";

    if (pointcloud->size())
    {
        pointcloud->width  = pointcloud->size();
        pointcloud->height = 1;
        pcl::io::savePCDFileASCII(save_path, *pointcloud);
        std::cout << "Saved point cloud to " << save_path << std::endl;
    }
    else
    {
        std::cerr << "WARNING: " << name << " " << pointcloud->size()
                  << std::endl;
    }
}
