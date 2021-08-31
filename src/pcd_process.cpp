#include <iostream>
#include <fstream>
#include <string>
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
   bool save_ascii = false;

   std::string input_pcd_path = "test_data/sample1.pcd";
   std::string save_pcd_path = "test_data/sampleXYZ.pcd";
   
};

void readConfig(Config& config)
{
   std::ifstream fin("config.txt");
   
   if(fin.is_open())
   {
      std::string line;
      while (std::getline(fin, line))
      {
         line.erase(std::remove_if(line.begin(), line.end(), isspace), line.end());

         auto delimiterPos = line.find("=");
         auto key = line.substr(0, delimiterPos);
         auto value = line.substr(delimiterPos + 1);

         if (key == "filter")
         {
            std::istringstream is(value);
            is >> std::boolalpha >> config.filter;
         }
         else if (key == "view_cloud")
         {
            std::istringstream is(value);
            is >> std::boolalpha >> config.view_cloud;
         }
         else if (key == "save_ascii")
         {
            std::istringstream is(value);
            is >> std::boolalpha >> config.save_ascii;
         }
         else if (key == "input_pcd_path")
         {
            config.input_pcd_path = value;
         }
         else if (key == "save_pcd_path")
         {
            config.save_pcd_path = value;
         }
         else if (key == "min_vec")
         {

         }
         else if (key == "max_vec")
         {

         }
         else
            std::cerr << key << " shouldn't exist in config.txt" << std::endl;
      }
      
   }
}

void viewCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
   pcl::visualization::CloudViewer viewer ("Cloud Viewer");
   viewer.showCloud(cloud);
   while (!viewer.wasStopped()){}
}

void filterBox(pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud,
               pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered,
               Eigen::Vector4f min_vec,
               Eigen::Vector4f max_vec)
{
   pcl::CropBox<pcl::PointXYZI> boxFilter;
   boxFilter.setMin(min_vec);
   boxFilter.setMax(max_vec);
   boxFilter.setInputCloud(input_cloud);
   boxFilter.filter(*cloud_filtered);
}

void saveAsASCII(std::string path, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
   pcl::io::savePCDFileASCII(path, *cloud);
}


int main(int argc, const char** argv)
{    
   pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
   pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZI>);

   // load pcd file
   if(pcl::io::loadPCDFile<pcl::PointXYZI> ("test_data/sample1.pcd", *cloud)==-1)
   {
      PCL_ERROR("Couldn't load pcd file. \n");
      return -1;
   }

   Eigen::Vector4f min_vec(-4.0,-5.0,-1.0,1.0); 
   Eigen::Vector4f max_vec(5.0,5.0,5.0,1.0);

   Config config;
   readConfig(config);
   
   if(config.filter)
      filterBox(cloud, cloud_filtered, min_vec, max_vec);

   if(config.view_cloud)
      viewCloud(cloud_filtered);
   
   if(config.save_ascii)
      saveAsASCII(config.save_pcd_path, cloud_filtered);
   
   std::cout << "pcd_process ended." << std::endl;
   return 0;
} 