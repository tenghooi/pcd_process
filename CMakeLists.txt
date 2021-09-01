cmake_minimum_required(VERSION 3.0.0 FATAL_ERROR)

# Create Project - set the project name and version
project(pcd_process VERSION 0.1.0)

# Specify the C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Set destination of executable
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_HOME_DIRECTORY})

# Find Packages
find_package(PCL 1.8 REQUIRED)

# Include header files
include_directories(${PROJECT_SOURCE_DIR}/include)

# Include PCL library
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Add executable
add_executable(pcd_process src/main.cpp src/functions.cpp)
target_link_libraries(pcd_process ${PCL_LIBRARIES})




