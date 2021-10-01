cmake_minimum_required(VERSION 3.0.0 FATAL_ERROR)

# Create Project - set the project name and version
project(pcd_process VERSION 0.1.0)

# Specify the C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Set destination of executable
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_HOME_DIRECTORY})

# Set path of OctoMap cmake files
# Change the path to where the octomap is installed in your machine
# list(APPEND CMAKE_PREFIX_PATH "/home/tenghooi/MyRepos/octomap/lib/cmake/octomap")
list(APPEND CMAKE_PREFIX_PATH "/home/tenghooi/External_Repo/octomap/lib/cmake/octomap")

# Find Packages
find_package(PCL 1.8 REQUIRED)
find_package(OCTOMAP CONFIG REQUIRED)

# Include header files
include_directories(${PROJECT_SOURCE_DIR}/include)

# Include PCL library
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Include OctoMap library
include_directories(${OCTOMAP_INCLUDE_DIRS})
link_libraries(${OCTOMAP_LIBRARIES})

# Add executable
add_executable(pcd_process src/main.cpp src/functions.cpp)
target_link_libraries(pcd_process ${PCL_LIBRARIES})




