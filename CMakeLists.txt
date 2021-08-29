cmake_minimum_required(VERSION 3.0.0 FATAL_ERROR)

# Create Project - set the project name and version
project(pcd_process VERSION 0.1.0)

# Specify the C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Find Packages
find_package(PCL 1.8 REQUIRED)

if(PCL_FOUND)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
endif()

# Add Executable - add the executable
add_executable(process src/pcd_read.cpp)
target_link_libraries(process ${PCL_LIBRARIES})

#[[
cmake_minimum_required(VERSION 3.0.0)

#set the project name and version
project(helloWorld VERSION 1.0)

#add the executable
add_executable(helloWorld helloworld.cpp)

#specify the C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# configure a header file to pass the version number only
configure_file(helloWorldConfig.h.in helloWorldConfig.h)
]]



