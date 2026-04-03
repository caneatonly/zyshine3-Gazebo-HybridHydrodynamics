cmake_minimum_required(VERSION 3.10)
project(hippo_gz_plugins)
set(lib_name hydrodynamics)
# 查找所需的 Gazebo 相关依赖
find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(gz-sim8 REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz-math7 REQUIRED)
find_package(gz-common5 REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(Protobuf REQUIRED)  # 用于 gz-msgs

# 设置 C++ 标准和编译选项
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# 添加共享库，关联浮力插件的源文件
add_library(${lib_name} SHARED
  hydrodynamics.cpp
)

# 链接依赖库
target_link_libraries(${lib_name}
  PUBLIC
    gz-plugin2::register
    gz-sim8::gz-sim8
    gz-transport13::gz-transport13
    gz-math7::gz-math7
    gz-common5::gz-common5
    gz-msgs10::gz-msgs10
)

# 指定头文件目录
target_include_directories(${lib_name}
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# 安装库文件
install(TARGETS ${lib_name}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
)
