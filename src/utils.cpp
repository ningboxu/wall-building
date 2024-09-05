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
// 针对 pcl::PointXYZRGB 类型的重载版本
void SavePointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pointcloud,
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
Eigen::Vector3f RotMat2Euler(const Eigen::Matrix3f& R)
{
    Eigen::Vector3f n = R.col(0);
    Eigen::Vector3f o = R.col(1);
    Eigen::Vector3f a = R.col(2);

    Eigen::Vector3f rpy(3);
    double y = atan2(n(1), n(0));
    double p = atan2(-n(2), n(0) * cos(y) + n(1) * sin(y));
    double r =
        atan2(a(0) * sin(y) - a(1) * cos(y), -o(0) * sin(y) + o(1) * cos(y));
    rpy(0) = r;
    rpy(1) = p;
    rpy(2) = y;

    // return ypr / M_PI * 180.0;
    return rpy;
}

void ShowPointQuat(const pcl::PointXYZ& p, const Eigen::Quaternionf& quat,
                   std::string name)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_x(
        new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_y(
        new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_z(
        new pcl::PointCloud<pcl::PointXYZRGB>);
    Eigen::Vector3f vec_x(1, 0, 0);  // x
    Eigen::Vector3f vec_y(0, 1, 0);  // y
    Eigen::Vector3f vec_z(0, 0, 1);  // z

    std::uint8_t a_255 = 255, b_0 = 0;
    std::uint32_t r = ((std::uint32_t)(a_255) << 16 |
                       (std::uint32_t)(b_0) << 8 | (std::uint32_t)(b_0));
    std::uint32_t g = ((std::uint32_t)(b_0) << 16 |
                       (std::uint32_t)(a_255) << 8 | (std::uint32_t)(b_0));
    std::uint32_t b = ((std::uint32_t)(b_0) << 16 | (std::uint32_t)(b_0) << 8 |
                       (std::uint32_t)(a_255));

    for (float i = 0; i < 0.2;)
    {
        Eigen::Vector3f p_x = i * vec_x;
        Eigen::Vector3f p_y = i * vec_y;
        Eigen::Vector3f p_z = i * vec_z;

        pcl::PointXYZRGB x_point;
        pcl::PointXYZRGB y_point;
        pcl::PointXYZRGB z_point;

        x_point.x = p_x(0);
        x_point.y = p_x(1);
        x_point.z = p_x(2);

        y_point.x = p_y(0);
        y_point.y = p_y(1);
        y_point.z = p_y(2);

        z_point.x = p_z(0);
        z_point.y = p_z(1);
        z_point.z = p_z(2);

        x_point.rgb = *reinterpret_cast<float*>(&r);
        y_point.rgb = *reinterpret_cast<float*>(&g);
        z_point.rgb = *reinterpret_cast<float*>(&b);

        pc_x->push_back(x_point);
        pc_y->push_back(y_point);
        pc_z->push_back(z_point);
        i += 0.01;
    }

    // robot arm coordinate: front(x) left(y) up(z)
    // camera coordinate: right(x) front(y) up(z)
    // the eulerAngles function in eigen, may return wrong result.use this
    // function: RotMat2Euler
    Eigen::Matrix3f rotation_matrix = quat.toRotationMatrix();
    Eigen::Vector3f euler_angles    = RotMat2Euler(rotation_matrix);
    LOG(INFO) << name << " euler_angles: " << euler_angles * 180.0 / M_PI;
    Eigen::Quaternionf eu2q;
    eu2q = Eigen::AngleAxisf(euler_angles[2], Eigen::Vector3f::UnitZ()) *
           Eigen::AngleAxisf(euler_angles[1], Eigen::Vector3f::UnitY()) *
           Eigen::AngleAxisf(euler_angles[0], Eigen::Vector3f::UnitX());

    // transform point with quat
    Eigen::Isometry3f trans = Eigen::Isometry3f::Identity();
    trans.rotate(eu2q);

    // ######### test #########
    // Eigen::Vector3f eulerAngle(3.14*45/180, 0, 0);
    // Eigen::AngleAxisf
    // rollAngle(Eigen::AngleAxisf(eulerAngle(2),Eigen::Vector3f::UnitX()));
    // Eigen::AngleAxisf
    // pitchAngle(Eigen::AngleAxisf(eulerAngle(1),Eigen::Vector3f::UnitY()));
    // Eigen::AngleAxisf
    // yawAngle(Eigen::AngleAxisf(eulerAngle(0),Eigen::Vector3f::UnitZ()));
    // Eigen::Quaternionf tt=yawAngle*pitchAngle*rollAngle;
    // tt.normalize();
    // trans.rotate(tt);
    // ######### test #########

    trans.pretranslate(Eigen::Vector3f(p.x, p.y, p.z));
    pcl::transformPointCloud(*pc_x, *pc_x, trans);
    pcl::transformPointCloud(*pc_y, *pc_y, trans);
    pcl::transformPointCloud(*pc_z, *pc_z, trans);
    SavePointCloud(pc_x, name + "_p_x");
    SavePointCloud(pc_y, name + "_p_y");
    SavePointCloud(pc_z, name + "_p_z");
}
