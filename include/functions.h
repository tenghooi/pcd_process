#ifndef _FUNCTIONS_H_
#define _FUNCTIONS_H_

#include <iostream>
#include <fstream>
#include <string>

// header files from PCL library
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h> 
#include <pcl/filters/crop_box.h>  

// Attributes of config.
struct Config
{
   bool filter = false;
   Eigen::Vector4f min_vec;
   Eigen::Vector4f max_vec;
   
   bool view_cloud = false;
   bool view_filtered_cloud = false;
   bool save_ascii = false;

   std::string input_pcd_path = "test_data/sample1.pcd";
   std::string save_pcd_path = "test_data/sampleXYZ.pcd";
   
};

// Utility function used in readConfig that convert '(' , ')' and ',' to space.
std::string punc2space(std::string& str);

// Simple parser that reads config.txt and set the config object attributes.
void readConfig(Config& config);

// Function to opens a window to view point cloud.
void viewCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);

// Function to crop the point cloud in a box region.
void filterBox(pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud,
               pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered,
               Eigen::Vector4f min_vec,
               Eigen::Vector4f max_vec);

// Save binary pcd in ascii (xyz) format.
void saveAsASCII(std::string path, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);

// Load pcd file into program.
int loadPCD(std::string path, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);

#endif