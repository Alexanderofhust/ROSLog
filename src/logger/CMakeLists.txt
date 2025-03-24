# 设置CMake的最低版本要求
cmake_minimum_required(VERSION 3.0.2)

# 定义项目名称（必须与package.xml中的<name>一致）
project(my_logger_cpp)

# 查找catkin构建系统
find_package(catkin REQUIRED)

# 查找其他依赖项（与package.xml中的<build_depend>一致）
find_package(roscpp REQUIRED)
find_package(std_msgs REQUIRED)

# 设置catkin的构建选项
catkin_package(
  INCLUDE_DIRS include  # 导出头文件目录
  LIBRARIES             # 导出库（如果有）
  CATKIN_DEPENDS roscpp std_msgs  # 导出依赖项
)

# 添加头文件目录
include_directories(
  include
  ${catkin_INCLUDE_DIRS}  # 包含catkin和依赖项的头文件目录
)

# 添加可执行文件
add_executable(logger_node src/logger_node.cpp)

# 链接库
target_link_libraries(logger_node ${catkin_LIBRARIES})