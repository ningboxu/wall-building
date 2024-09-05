#include <glog/logging.h>
#include <pcl/common/centroid.h>  // 用于计算点云的质心
#include <pcl/features/moment_of_inertia_estimation.h>  // 用于计算点云的特征信息
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>  // 用于可视化
#include <cmath>  // 用于计算平方根等数学操作
#include <iostream>

#include "utils.h"

// 将点云单位从mm转换为m
void ConvertPointCloudToMeters(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
    for (auto& point : cloud->points)
    {
        point.x /= 1000.0f;  // 将 x 坐标从 mm 转换为 m
        point.y /= 1000.0f;  // 将 y 坐标从 mm 转换为 m
        point.z /= 1000.0f;  // 将 z 坐标从 mm 转换为 m
    }
}

// 计算点到平面的距离
double computePointToPlaneDistance(
    const Eigen::Vector4f& point,
    const pcl::ModelCoefficients::Ptr& coefficients)
{
    // 提取平面方程的系数 A, B, C, D
    double A = coefficients->values[0];
    double B = coefficients->values[1];
    double C = coefficients->values[2];
    double D = coefficients->values[3];

    // 点的坐标 (x, y, z)
    double x = point[0];
    double y = point[1];
    double z = point[2];

    // 计算点到平面的距离
    double numerator   = std::fabs(A * x + B * y + C * z + D);  // 计算分子
    double denominator = std::sqrt(A * A + B * B + C * C);      // 计算分母
    return numerator / denominator;
}

void DownsamplePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                          float leaf_size)
{
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(leaf_size, leaf_size, leaf_size);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(
        new pcl::PointCloud<pcl::PointXYZ>);
    sor.filter(*cloud_filtered);
    cloud = cloud_filtered;
}

void visualizePointCloudWithVectors(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                    const Eigen::Vector4f& centroid,
                                    const Eigen::Vector3f& major_vector,
                                    const Eigen::Vector3f& middle_vector,
                                    const Eigen::Vector3f& minor_vector)
{
    // 创建可视化窗口
    pcl::visualization::PCLVisualizer::Ptr viewer(
        new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);  // 设置背景颜色为黑色

    // 将点云添加到可视化窗口
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color(
        cloud, 255, 255, 255);  // 白色点云
    viewer->addPointCloud<pcl::PointXYZ>(cloud, cloud_color, "sample cloud");

    // 绘制质心
    pcl::PointXYZ centroid_point(centroid[0], centroid[1], centroid[2]);
    viewer->addSphere(centroid_point, 0.02, 1.0, 0.0, 0.0,
                      "centroid");  // 用红色球体表示质心

    // 绘制三个特征向量
    pcl::PointXYZ major_end(centroid[0] + major_vector[0],
                            centroid[1] + major_vector[1],
                            centroid[2] + major_vector[2]);
    pcl::PointXYZ middle_end(centroid[0] + middle_vector[0],
                             centroid[1] + middle_vector[1],
                             centroid[2] + middle_vector[2]);
    pcl::PointXYZ minor_end(centroid[0] + minor_vector[0],
                            centroid[1] + minor_vector[1],
                            centroid[2] + minor_vector[2]);

    // 用线表示特征向量
    viewer->addLine(centroid_point, major_end, 1.0, 0.0, 0.0,
                    "major_vector");  // 红色线表示主方向
    viewer->addLine(centroid_point, middle_end, 0.0, 1.0, 0.0,
                    "middle_vector");  // 绿色线表示中方向
    viewer->addLine(centroid_point, minor_end, 0.0, 0.0, 1.0,
                    "minor_vector");  // 蓝色线表示次方向

    // 设置点云的大小
    viewer->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");

    // 启动可视化循环
    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
    }
}

int main(int argc, char** argv)
{
    if (argc < 2)
    {
        std::cerr << "Please provide a path to the point cloud file!\n";
        return -1;
    }

    std::string file_path = argv[1];

    // 日志设置
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold  = google::ERROR;
    FLAGS_minloglevel      = google::INFO;
    FLAGS_colorlogtostderr = true;
    FLAGS_log_dir          = "./";

    // 加载点云数据
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    if (file_path.substr(file_path.find_last_of(".") + 1) == "pcd")
    {
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_path, *cloud) == -1)
        {
            PCL_ERROR("Couldn't read PCD file\n");
            return -1;
        }
    }
    else if (file_path.substr(file_path.find_last_of(".") + 1) == "ply")
    {
        pcl::PLYReader reader;
        if (reader.read(file_path, *cloud) == -1)
        {
            PCL_ERROR("Couldn't read PLY file\n");
            return -1;
        }
    }
    else
    {
        std::cerr
            << "Unsupported file format! Supported formats are PCD and PLY.\n";
        return -1;
    }

    LOG(INFO) << "Loaded " << cloud->width * cloud->height
              << " data points from " << file_path;
    // 打印加载的点云点数
    std::cout << "PointCloud before conversion: " << cloud->points.size()
              << " points." << std::endl;

    // 将点云单位从毫米转换为米
    ConvertPointCloudToMeters(cloud);
    LOG(INFO) << "Converted point cloud units from mm to meters.";

    // 体素下采样
    DownsamplePointCloud(cloud, 0.005f);
    // 打印转换后的点云点数
    std::cout << "PointCloud after Downsample: " << cloud->points.size()
              << " points." << std::endl;
    SavePointCloud(cloud, "downsampled_cloud");

    // 计算点云中心点
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud, centroid);
    std::cout << "Centroid of point cloud: (" << centroid[0] << ", "
              << centroid[1] << ", " << centroid[2] << ")" << std::endl;
    pcl::PointXYZ centroid_point(centroid[0], centroid[1], centroid[2]);

    // 使用 MomentOfInertiaEstimation 计算特征向量
    pcl::MomentOfInertiaEstimation<pcl::PointXYZ> feature_extractor;
    feature_extractor.setInputCloud(cloud);
    feature_extractor.compute();

    Eigen::Vector3f major_vector, middle_vector, minor_vector;
    feature_extractor.getEigenVectors(major_vector, middle_vector,
                                      minor_vector);

    // 打印主方向的特征向量
    std::cout << "Major eigenvector: [" << major_vector[0] << ", "
              << major_vector[1] << ", " << major_vector[2] << "]" << std::endl;

    std::cout << "Middle eigenvector: [" << middle_vector[0] << ", "
              << middle_vector[1] << ", " << middle_vector[2] << "]"
              << std::endl;

    std::cout << "Minor eigenvector: [" << minor_vector[0] << ", "
              << minor_vector[1] << ", " << minor_vector[2] << "]" << std::endl;
    // 验证正交
    float dot_product_major_middle = major_vector.dot(middle_vector);
    float dot_product_major_minor  = major_vector.dot(minor_vector);
    float dot_product_middle_minor = middle_vector.dot(minor_vector);

    std::cout << "Dot product of major and middle: " << dot_product_major_middle
              << std::endl;
    std::cout << "Dot product of major and minor: " << dot_product_major_minor
              << std::endl;
    std::cout << "Dot product of middle and minor: " << dot_product_middle_minor
              << std::endl;

    Eigen::Matrix3f rotation_matrix;
    rotation_matrix.col(0) = major_vector;
    rotation_matrix.col(1) = middle_vector;
    rotation_matrix.col(2) = minor_vector;
    Eigen::Quaternionf quat(rotation_matrix);
    // todo 不全用特征向量来表示位姿
    // 如使用拟合平面法向量、major_vector,另一个方向叉乘

    // 保存质心和位姿
    ShowPointQuat(centroid_point, quat, "cloud_pose");

    // // 可视化结果
    // visualizePointCloudWithVectors(cloud, centroid, major_vector,
    // middle_vector,
    //                                minor_vector);

    return 0;
}
