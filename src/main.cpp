#include <glog/logging.h>
#include <pcl/common/centroid.h>  // 用于计算点云的质心
#include <pcl/features/moment_of_inertia_estimation.h>  // 用于计算点云的特征信息
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <cmath>  // 用于计算平方根等数学操作
#include <iostream>

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

    // 打印转换后的点云点数
    std::cout << "PointCloud after conversion: " << cloud->points.size()
              << " points." << std::endl;

    // 计算点云中心点
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud, centroid);
    std::cout << "Centroid of point cloud: (" << centroid[0] << ", "
              << centroid[1] << ", " << centroid[2] << ")" << std::endl;

    // 设置平面分割器
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.01);

    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.size() == 0)
    {
        PCL_ERROR("Could not estimate a planar model for the given dataset.");
        return -1;
    }

    // 打印平面模型
    std::cout << "Model coefficients: ";
    for (auto val : coefficients->values) std::cout << val << " ";
    std::cout << std::endl;

    // 计算质心到拟合平面的距离
    double distance = computePointToPlaneDistance(centroid, coefficients);
    std::cout << "Distance from centroid to plane: " << distance << std::endl;

    // 体素下采样
    DownsamplePointCloud(cloud, 0.01f);

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

    return 0;
}
