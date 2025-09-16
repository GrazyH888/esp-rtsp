cmake_minimum_required(VERSION 3.5)

# 设置ADF和IDF路径
set(ADF_PATH "${CMAKE_CURRENT_SOURCE_DIR}")
set(IDF_PATH "/home/hyl/esp/esp-idf")

# 设置环境变量
set(ENV{ADF_PATH} "${ADF_PATH}")
set(ENV{IDF_PATH} "${IDF_PATH}")

include(${IDF_PATH}/tools/cmake/project.cmake)

# 添加组件路径
list(APPEND EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/components")

project(my_rtsp_project VERSION 1.0.0)
