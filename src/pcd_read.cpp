#include <iostream>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h> 
#include <pcl/filters/crop_box.h>           
                   
int main(int argc, const char** argv)
{    
   pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
   pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZI>);
   pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
   // load pcd file
   if(pcl::io::loadPCDFile<pcl::PointXYZI> ("../test_data/sample1.pcd", *cloud)==-1)
   {
      PCL_ERROR("Couldn't load pcd file. \n");
      return -1;
   }

   pcl::CropBox<pcl::PointXYZI> boxFilter;
   boxFilter.setMin(Eigen::Vector4f(-5.0,-5.0,-5.0,1.0)); 
   boxFilter.setMax(Eigen::Vector4f(5.0,5.0,5.0,1.0));
   boxFilter.setInputCloud(cloud);
   boxFilter.filter(*cloud_filtered);

   viewer.showCloud(cloud_filtered);
   while(!viewer.wasStopped()){}

   pcl::io::savePCDFileASCII("../test_data/sampleXYZ.pcd", *cloud_filtered);
   
   std::cout << "Hello World" << std::endl;
   return 0;
} 