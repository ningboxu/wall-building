#include <glog/logging.h>
#include <pcl/common/centroid.h>  // 用于计算点云的质心
#include <pcl/features/moment_of_inertia_estimation.h>  // 用于计算点云的特征信息
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <Eigen/Geometry>
#include <chrono>  // 用于统计时间
#include <cmath>   // 用于计算平方根等数学操作
#include <iostream>
#include "utils.h"

int main(int argc, char** argv)
{
    using namespace std::chrono;

    // 记录程序总开始时间
    auto program_start = high_resolution_clock::now();

    if (argc < 2)
    {
        std::cerr << "Please provide a path to the point cloud file!\n";
        return -1;
    }

    std::string file_path = argv[1];

    //! 日志设置
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold  = google::ERROR;
    FLAGS_minloglevel      = google::INFO;
    FLAGS_colorlogtostderr = true;
    FLAGS_log_dir          = "./";

    //! 加载点云数据
    auto start_load = high_resolution_clock::now();  // 记录开始时间
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_original(
        new pcl::PointCloud<pcl::PointXYZ>);
    if (file_path.substr(file_path.find_last_of(".") + 1) == "pcd")
    {
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_path, *cloud_original) ==
            -1)
        {
            PCL_ERROR("Couldn't read PCD file\n");
            return -1;
        }
    }
    else if (file_path.substr(file_path.find_last_of(".") + 1) == "ply")
    {
        pcl::PLYReader reader;
        if (reader.read(file_path, *cloud_original) == -1)
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
    auto end_load = high_resolution_clock::now();  // 记录结束时间
    LOG(INFO) << "Loaded " << cloud_original->width * cloud_original->height
              << " data points from " << file_path;
    std::cout << "Loading time: "
              << duration_cast<milliseconds>(end_load - start_load).count()
              << " ms" << std::endl;

    std::cout << "PointCloud before conversion: "
              << cloud_original->points.size() << " points." << std::endl;

    //! 将点云单位从毫米转换为米
    // auto start_convert = high_resolution_clock::now();  // 记录开始时间
    // ConvertPointCloudToMeters(cloud_original);
    // // ConvertPointCloudToMeters(cloud);
    // LOG(INFO) << "Converted point cloud units from mm to meters.";
    // auto end_convert = high_resolution_clock::now();  // 记录结束时间
    // std::cout
    //     << "Conversion time: "
    //     << duration_cast<milliseconds>(end_convert - start_convert).count()
    //     << " ms" << std::endl;

    //     //! 按z轴过滤点云
    //     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
    //         new pcl::PointCloud<pcl::PointXYZ>);
    //     filterZAxis(cloud_original, 0.675, 0.70,
    //                 cloud);  // 例如保留 z 值在 [0, 1] 范围内的点
    // #ifdef OUTPUT_RESULTS
    //     SavePointCloud(cloud, "cloud_after_z");
    // #endif
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud_original);
    pass.setFilterFieldName("z");
    // pass.setFilterLimits(0.6750, 0.7000);
    pass.setFilterLimits(675.0, 700.0);
    pass.setNegative(false);
    pass.filter(*cloud);

    // todo 检测砖块点云
    //     //! 将点云分割为4cm厚度的层
    //     // 假设高度有1米
    //     int slicing_num = 1.0 / 0.04;
    //     pcl::PointCloud<pcl::PointXYZ>::Ptr slicing_cloud[slicing_num];
    //     for (auto& a : slicing_cloud)
    //     {
    //         a = pcl::PointCloud<pcl::PointXYZ>::Ptr(
    //             new pcl::PointCloud<pcl::PointXYZ>);
    //     }
    //     for (unsigned int i = 0; i < cloud_in_base->size(); i++)
    //     {
    //         const pcl::PointXYZ& Point = cloud_in_base->points[i];
    //         int s = std::floor((cloud_in_base_maxv.z - Point.z) / 0.04);
    //         // LOG(INFO) << "s:" << cloud_in_rotated_min;
    //         if (s < slicing_num - 1)
    //         {
    //             slicing_cloud[s]->points.push_back(Point);
    //         }
    //     }

    // //! 调试 保存每一层的点云数据
    // #ifdef OUTPUT_RESULTS
    //     for (int i = 0; i < slicing_num; i++)
    //     {
    //         if (slicing_cloud[i]->size() > 0)
    //         {  // 确保点云层不是空的
    //             std::ostringstream oss;
    //             oss << "slicing_cloud_layer_" << (i + 1);  //
    //             创建文件名，包含层编号 SavePointCloud(slicing_cloud[i],
    //             oss.str());
    //         }
    //     }
    // #endif

    //! 计算点云中心点
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud, centroid);
    std::cout << "Centroid of point cloud: (" << centroid[0] << ", "
              << centroid[1] << ", " << centroid[2] << ")" << std::endl;
    Eigen::Vector3f centroid_vec(centroid[0], centroid[1], centroid[2]);
    pcl::PointXYZ centroid_point(centroid[0], centroid[1], centroid[2]);

    // 设置平面分割器 //todo 暂没用到
    auto start_seg = high_resolution_clock::now();  // 记录开始时间
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.001);

    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);
    auto end_seg = high_resolution_clock::now();  // 记录结束时间

    if (inliers->indices.size() == 0)
    {
        PCL_ERROR("Could not estimate a planar model for the given dataset.");
        return -1;
    }

    std::cout << "Segmentation time: "
              << duration_cast<milliseconds>(end_seg - start_seg).count()
              << " ms" << std::endl;

    // 打印平面模型
    std::cout << "Model coefficients: ";
    for (auto val : coefficients->values) std::cout << val << " ";
    std::cout << std::endl;
// todo  有问题
#ifdef OUTPUT_RESULTS

    Eigen::Vector3f normal(coefficients->values[0], coefficients->values[1],
                           coefficients->values[2]);
    ShowPlane(centroid_point, coefficients, "coefficients", 1.0f, 100);
    ShowVector(normal, "normal", centroid_vec, 1.0f);  //! if m  if mm

#endif

    // 计算质心到拟合平面的距离
    double distance = computePointToPlaneDistance(centroid, coefficients);
    std::cout << "Distance from centroid to plane: " << distance << std::endl;

    //! 体素下采样
    auto start_downsample = high_resolution_clock::now();  // 记录开始时间
    // DownsamplePointCloud(cloud, 0.005f);                   // 使用m
    DownsamplePointCloud(cloud, 0.005f * 1000);  // mm
    std::cout << "PointCloud after Downsample: " << cloud->points.size()
              << " points." << std::endl;
#ifdef OUTPUT_RESULTS
    SavePointCloud(cloud, "downsampled_cloud");
#endif
    auto end_downsample = high_resolution_clock::now();  // 记录结束时间
    std::cout << "Downsampling time: "
              << duration_cast<milliseconds>(end_downsample - start_downsample)
                     .count()
              << " ms" << std::endl;

    //! 使用 MomentOfInertiaEstimation 计算特征向量
    auto start_features = high_resolution_clock::now();  // 记录开始时间
    pcl::MomentOfInertiaEstimation<pcl::PointXYZ> feature_extractor;
    feature_extractor.setInputCloud(cloud);
    feature_extractor.compute();

    Eigen::Vector3f major_vector, middle_vector, minor_vector;
    feature_extractor.getEigenVectors(major_vector, middle_vector,
                                      minor_vector);
    auto end_features = high_resolution_clock::now();  // 记录结束时间

    std::cout
        << "Feature extraction time: "
        << duration_cast<milliseconds>(end_features - start_features).count()
        << " ms" << std::endl;

    // 打印主方向的特征向量
    std::cout << "Major eigenvector: [" << major_vector[0] << ", "
              << major_vector[1] << ", " << major_vector[2] << "]" << std::endl;

    std::cout << "Middle eigenvector: [" << middle_vector[0] << ", "
              << middle_vector[1] << ", " << middle_vector[2] << "]"
              << std::endl;

    std::cout << "Minor eigenvector: [" << minor_vector[0] << ", "
              << minor_vector[1] << ", " << minor_vector[2] << "]" << std::endl;

    Eigen::Matrix3f rotation_matrix;
    rotation_matrix.col(0) = major_vector;
    rotation_matrix.col(1) = middle_vector;
    rotation_matrix.col(2) = minor_vector;
    Eigen::Quaternionf quat(rotation_matrix);

    // 保存质心和位姿
    ShowPointQuat(centroid_point, quat, "pose_camera");

    //! 坐标系转换--------------------------
    // 工具到相机的变换 (平移和欧拉角)
    Eigen::Vector3f translation_TC(46.307365116896307, 83.64781988216231,
                                   -270.34468528536888);
    Eigen::Vector3f euler_TC(0.050761172951085926, 0.0047434909777201821,
                             -1.5778546245587957);  //  (ZYX顺序欧拉角)

    // 工具到基坐标系的变换 (平移和四元数)
    Eigen::Vector3f translation_TB(2077, 689.31,
                                   1405.42);  // 工具到基坐标系的平移
    Eigen::Quaternionf quat_TB(0.00392, 0.91621, -0.40062,
                               -0.00601);  // 工具到基坐标系的旋转 (四元数)

    // 保存相机到基坐标系的平移和旋转
    Eigen::Vector3f translation_CB;
    Eigen::Quaternionf quat_CB;

    // 计算相机到基坐标系的变换
    computeCameraToBaseTransform(translation_TC, euler_TC, translation_TB,
                                 quat_TB, translation_CB, quat_CB);

    // 打印最终结果
    std::cout << "最终相机到基坐标系的平移: " << translation_CB.transpose()
              << std::endl;
    //! 打印是x y z w
    std::cout << "最终相机到基坐标系的旋转 (四元数): "
              << quat_CB.coeffs().transpose() << std::endl;
    //! 坐标系转换----------------------------------------

    // 变换矩阵 //todo 重命名
    Eigen::Isometry3f camera_calibrate_ = Eigen::Isometry3f::Identity();
    camera_calibrate_.rotate(quat_CB);
    // translation_CB = translation_CB / 1000;  //! 转换为m
    // 不转化为m
    translation_CB = translation_CB;
    camera_calibrate_.pretranslate(translation_CB);

    //! 在base下的质心和位姿
    Eigen::Vector3f centroid_base =
        camera_calibrate_ * centroid_vec;           // 质心转换
    Eigen::Quaternionf quat_base = quat_CB * quat;  // 姿态转换
    std::cout << "camera_calibrate_ matrix:" << std::endl;
    std::cout << camera_calibrate_.matrix() << std::endl;
    std::cout << "center point  in base : " << centroid_base.transpose()
              << std::endl;
    std::cout << " orientation in base (quaternion x y z w): "
              << quat_base.coeffs().transpose() << std::endl;
    pcl::PointXYZ centroid_p_base(centroid_base[0], centroid_base[1],
                                  centroid_base[2]);

#ifdef OUTPUT_RESULTS
    ShowPointQuat(centroid_p_base, quat_base, "pose_base");
#endif

    //! 根据需求调整位姿，绕z方向旋转多少度
    // 先计算当前的旋转矩阵
    Eigen::Matrix3f rotation_matrix1 = quat_base.toRotationMatrix();

    // 提取当前的Z方向向量作为旋转轴
    Eigen::Vector3f z_direction = rotation_matrix1.col(2);  // Z方向向量

    // 创建绕当前Z方向旋转90度的旋转矩阵
    float angle = M_PI / 2;  // 90度 = pi/2 弧度  //todo
    Eigen::Matrix3f rotation_around_z_direction;
    rotation_around_z_direction =
        Eigen::AngleAxisf(angle, z_direction.normalized());

    // 将绕Z方向的旋转矩阵与当前的旋转矩阵相乘
    Eigen::Matrix3f new_rotation_matrix =
        rotation_around_z_direction * rotation_matrix1;

    // 将更新后的旋转矩阵转换回四元数
    Eigen::Quaternionf new_quat(new_rotation_matrix);

    // 输出新的姿态方向向量
    Eigen::Vector3f new_x_direction = new_rotation_matrix.col(0);  // x方向向量
    Eigen::Vector3f new_y_direction = new_rotation_matrix.col(1);  // y方向向量
    Eigen::Vector3f new_z_direction = new_rotation_matrix.col(2);  // z方向向量

    // 输出结果
    std::cout << "New orientation in base (quaternion x y z w): "
              << new_quat.coeffs().transpose() << std::endl;

    std::cout << "New X direction in base: " << new_x_direction.transpose()
              << std::endl;
    std::cout << "New Y direction in base: " << new_y_direction.transpose()
              << std::endl;
    std::cout << "New Z direction in base: " << new_z_direction.transpose()
              << std::endl;

#ifdef OUTPUT_RESULTS
    ShowPointQuat(centroid_p_base, new_quat, "new_pose_base");
#endif

    //! cloud_in_base(下采样后)
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in_base(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*cloud, *cloud_in_base,
                             camera_calibrate_.cast<float>());
    cloud_in_base->width  = cloud_in_base->size();
    cloud_in_base->height = 1;
    LOG(INFO) << "cloud_in_base->points.size: " << cloud_in_base->points.size();
    std::cout << " cloud_in_base->points.size: " << cloud_in_base->points.size()
              << std::endl;

#ifdef OUTPUT_RESULTS
    SavePointCloud(cloud_in_base, "cloud_in_base");
#endif

    // 记录程序总结束时间
    auto program_end = high_resolution_clock::now();
    std::cout
        << "Total program execution time: "
        << duration_cast<milliseconds>(program_end - program_start).count()
        << " ms" << std::endl;

    return 0;
}
