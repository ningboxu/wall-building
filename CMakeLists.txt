cmake_minimum_required(VERSION 3.10)

project(PointCloudProcessing)

# 查找PCL库
find_package(PCL REQUIRED)

# 查找Eigen库
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# 添加PCL和Eigen头文件路径
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})

# 添加源文件和头文件目录
include_directories(include)

# 添加可执行文件和源文件
add_executable(main src/main.cpp)

# 链接PCL和glog库
target_link_libraries(main ${PCL_LIBRARIES} glog)

# Suppress developer warnings
set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS 1)
