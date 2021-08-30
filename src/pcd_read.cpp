#include <iostream>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>            
                   
int main(int argc, const char** argv)
{    
   pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
   pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
   // load pcd file
   if(pcl::io::loadPCDFile<pcl::PointXYZI> ("../test_data/sample1.pcd", *cloud)==-1)
   {
      PCL_ERROR("Couldn't load pcd file. \n");
      return -1;
   }

   pcl::io::savePCDFileASCII("../test_data/sampleXYZ.pcd", *cloud);
   
   viewer.showCloud(cloud);
   while(!viewer.wasStopped()){}
   /*
   for(const auto &point: *cloud)
   {
      std::cout << "x: " << point.x
                << "\ty: " << point.y 
                << "\tz: " << point.z  
                << "\nI: " << point.intensity << std::endl;
   }

   std::cout << "Loaded " << cloud->width * cloud->height
               << " data points from pcd file with the following fields: "
               << std::endl;

   */

   std::cout << "Hello World" << std::endl;
   return 0;
} 