cmake_minimum_required(VERSION 3.0.0)

# Create Project
project(pcd_process VERSION 0.1.0)

# Find Packages
find_package(PCL 1.8 REQUIRED)


include(CTest)
enable_testing()

add_executable(pcd_process main.cpp)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
