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