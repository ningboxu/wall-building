
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
void ShowVector(const Eigen::Vector3f& vec, std::string name,
                Eigen::Vector3f start_p, float length)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc(new pcl::PointCloud<pcl::PointXYZ>);
    Eigen::Vector3f vec_tmp = vec.normalized();
    for (float i = 0; i < length;)
    {
        Eigen::Vector3f p = start_p + i * vec_tmp;
        pc->push_back(pcl::PointXYZ(p(0), p(1), p(2)));
        i += 0.01;
    }
    SavePointCloud(pc, name);
}
void ShowPlane(const pcl::PointXYZ& centroid_point,
               const pcl::ModelCoefficients::Ptr& coefficients,
               std::string file_name, float plane_size, int resolution)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr plane_cloud(
        new pcl::PointCloud<pcl::PointXYZ>());

    // 获取平面方程 ax + by + cz + d = 0 中的 a, b, c
    float a = coefficients->values[0];
    float b = coefficients->values[1];
    float c = coefficients->values[2];
    float d = coefficients->values[3];

    // 确定平面上的两个方向向量
    Eigen::Vector3f normal(a, b, c);
    Eigen::Vector3f v1, v2;

    // 创建平面上的两个正交向量
    if (std::fabs(normal[0]) > std::fabs(normal[1]))
        v1 = Eigen::Vector3f(-normal[2], 0, normal[0]).normalized();
    else
        v1 = Eigen::Vector3f(0, -normal[2], normal[1]).normalized();

    v2 = normal.cross(v1).normalized();

    // 使用规则网格生成点云
    float step = plane_size / resolution;  // 根据分辨率设置步长

    for (int i = -resolution / 2; i < resolution / 2; ++i)
    {
        for (int j = -resolution / 2; j < resolution / 2; ++j)
        {
            pcl::PointXYZ point;
            point.x = centroid_point.x + i * step * v1[0] + j * step * v2[0];
            point.y = centroid_point.y + i * step * v1[1] + j * step * v2[1];
            point.z = centroid_point.z + i * step * v1[2] + j * step * v2[2];

            plane_cloud->points.push_back(point);
        }
    }
    // todo 可调用函数SavePointCloud
    //  保存生成的平面点云
    if (plane_cloud->size())
    {
        plane_cloud->width    = plane_cloud->size();
        plane_cloud->height   = 1;
        std::string save_path = file_name + ".pcd";
        pcl::io::savePCDFileASCII(save_path, *plane_cloud);
        std::cout << "Saved point cloud to " << save_path << std::endl;
    }
    else
    {
        std::cerr << "WARNING: Plane cloud is empty" << std::endl;
    }
}

//--------------------上面是保存数据的功能---------------------

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
void filterZAxis(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float z_min,
                 float z_max,
                 pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud)
{
    // 创建 PassThrough 滤波器对象
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);

    // 设置 z 轴方向上的范围
    pass.setFilterFieldName("z");
    pass.setFilterLimits(z_min, z_max);

    // 应用滤波器，结果存储在 filtered_cloud 中
    pass.filter(*filtered_cloud);
}

// 欧拉角转换为四元数 (ZYX顺序)
Eigen::Quaternionf eulerToQuaternion(const Eigen::Vector3f& euler_angles)
{
    // ZYX顺序：先绕z轴旋转，再绕y轴，最后绕x轴
    Eigen::AngleAxisf rollAngle(euler_angles(0),
                                Eigen::Vector3f::UnitX());  // 绕x轴
    Eigen::AngleAxisf pitchAngle(euler_angles(1),
                                 Eigen::Vector3f::UnitY());  // 绕y轴
    Eigen::AngleAxisf yawAngle(euler_angles(2),
                               Eigen::Vector3f::UnitZ());  // 绕z轴

    // 四元数表示的旋转
    Eigen::Quaternionf q = yawAngle * pitchAngle * rollAngle;
    return q;
}

// 计算相机到基坐标系的变换矩阵
void computeCameraToBaseTransform(const Eigen::Vector3f& translation_CT,
                                  const Eigen::Vector3f& euler_CT,
                                  const Eigen::Vector3f& translation_TB,
                                  const Eigen::Quaternionf& quat_TB,
                                  Eigen::Vector3f& translation_CB,
                                  Eigen::Quaternionf& quat_CB)
{
    // 相机到工具的旋转四元数 (欧拉角 -> 四元数)
    Eigen::Quaternionf quat_CT = eulerToQuaternion(euler_CT);

    // 相机到工具的变换矩阵
    Eigen::Matrix4f T_CT   = Eigen::Matrix4f::Identity();
    T_CT.block<3, 3>(0, 0) = quat_CT.toRotationMatrix();  // 旋转部分
    T_CT.block<3, 1>(0, 3) = translation_CT;              // 平移部分

    // 工具到基坐标系的变换矩阵
    Eigen::Matrix4f T_TB   = Eigen::Matrix4f::Identity();
    T_TB.block<3, 3>(0, 0) = quat_TB.toRotationMatrix();  // 旋转部分
    T_TB.block<3, 1>(0, 3) = translation_TB;              // 平移部分

    // 相机到基坐标系的变换
    Eigen::Matrix4f T_CB = T_TB * T_CT;

    // 提取相机到基坐标系的平移和旋转（四元数）
    translation_CB                     = T_CB.block<3, 1>(0, 3);
    Eigen::Matrix3f rotation_CB_matrix = T_CB.block<3, 3>(0, 0);
    quat_CB                            = Eigen::Quaternionf(rotation_CB_matrix);

    // 打印信息
    std::cout << "相机到工具的平移: " << translation_CT.transpose()
              << std::endl;
    std::cout << "相机到工具的旋转 (四元数): " << quat_CT.coeffs().transpose()
              << std::endl;

    std::cout << "工具到基坐标系的平移: " << translation_TB.transpose()
              << std::endl;
    std::cout << "工具到基坐标系的旋转 (四元数): "
              << quat_TB.coeffs().transpose() << std::endl;

    std::cout << "相机到基坐标系的平移: " << translation_CB.transpose()
              << std::endl;
    std::cout << "相机到基坐标系的旋转 (四元数): "
              << quat_CB.coeffs().transpose() << std::endl;
}
