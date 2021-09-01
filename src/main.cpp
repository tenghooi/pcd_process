#include <iostream>
#include "functions.h"
                 
                   
int main(int argc, const char** argv)
{           
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZI>);

    Config config;
    readConfig(config);

    // load pcd file
    if(loadPCD(config.input_pcd_path, cloud) == -1)
        return -1;
    
    if(config.filter)
    {
        std::cout << "Filtering in point cloud using a box region..." << std::endl;
        filterBox(cloud, cloud_filtered, config.min_vec, config.max_vec);
        std::cout << "Filtering completed" << std::endl;
    }

    if(config.view_cloud)
        viewCloud(cloud);

    if(config.view_filtered_cloud)
        viewCloud(cloud_filtered);

    if(config.save_ascii)
        saveAsASCII(config.save_pcd_path, cloud_filtered);
    
    std::cout << "pcd_process ended." << std::endl;      
    return 0;
}