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

void readConfig(Config& config);

void viewCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);

void filterBox(pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud,
               pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered,
               Eigen::Vector4f min_vec,
               Eigen::Vector4f max_vec);

void saveAsASCII(std::string path, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);

#endif