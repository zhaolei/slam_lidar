cmake_minimum_required(VERSION 2.8)
project(LD06_LiDAR)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# 查找 OpenCV
find_package(OpenCV REQUIRED)

# 添加头文件目录
include_directories(${OpenCV_INCLUDE_DIRS})

# 源码文件列表
set(SOURCES 
    cmd_interface_linux.cpp 
        lipkg.cpp 
            main.cpp 
                tofbf.cpp
                )

set(HEADERS 
    cmd_interface_linux.h 
        lipkg.h 
            tofbf.h
            )

# 生成可执行文件
add_executable(ld06_lidar ${SOURCES} ${HEADERS})

# 链接 OpenCV 和 pthread
target_link_libraries(ld06_lidar 
    ${OpenCV_LIBS} 
    udev
        )

# 输出构建信息
message(STATUS "Using OpenCV version: ${OpenCV_VERSION}")

