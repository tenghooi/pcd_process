#include <iostream>
#include "functions.h"
                 
                   
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

    Config config;
    readConfig(config);
    
    if(config.filter)
    {
        std::cout << "Filtering in point cloud using a box region..." << std::endl;
        filterBox(cloud, cloud_filtered, config.min_vec, config.max_vec);
        std::cout << "Filtering completed" << std::endl;
    }

    if(config.view_filtered_cloud)
        viewCloud(cloud_filtered);

    if(config.view_cloud)
        viewCloud(cloud);
    
    if(config.save_ascii)
        saveAsASCII(config.save_pcd_path, cloud_filtered);
    
    std::cout << "pcd_process ended." << std::endl;      
    return 0;
}