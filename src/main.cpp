#include <iostream>
#include "functions.h"

#include <octomap/octomap.h>                 

void calcThresholdedNodes(const octomap::OcTree tree,
                          unsigned int& num_thresholded,
                          unsigned int& num_other)
{
  num_thresholded = 0;
  num_other = 0;

  for(octomap::OcTree::tree_iterator it = tree.begin_tree(), end=tree.end_tree(); it!= end; ++it){
    if (tree.isNodeAtThreshold(*it))
      num_thresholded++;
    else
      num_other++;
  }
}

void outputStatistics(const octomap::OcTree tree){
  unsigned int numThresholded, numOther;
  calcThresholdedNodes(tree, numThresholded, numOther);
  size_t memUsage = tree.memoryUsage();
  unsigned long long memFullGrid = tree.memoryFullGrid();
  size_t numLeafNodes = tree.getNumLeafNodes();

  cout << "Tree size: " << tree.size() <<" nodes (" << numLeafNodes<< " leafs). " <<numThresholded <<" nodes thresholded, "<< numOther << " other\n";
  cout << "Memory: " << memUsage << " byte (" << memUsage/(1024.*1024.) << " MB)" << endl;
  cout << "Full grid: "<< memFullGrid << " byte (" << memFullGrid/(1024.*1024.) << " MB)" << endl;
  double x, y, z;
  tree.getMetricSize(x, y, z);
  cout << "Size: " << x << " x " << y << " x " << z << " m^3\n";
  cout << endl;
}

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

    // insert cloud_filtered into Octree and convert to OctoMap
    // create octree with 0.01 resolution
    octomap::OcTree tree(0.01);
    octomap::point3d sensor_origin {0.0, 0.0, 0.0};
    octomap::Pointcloud octocloud;
    
    /*
    for(auto p:(*cloud_filtered).points)
    {
        tree.updateNode(octomap::point3d(p.x, p.y, p.z), true);
    }
    */
    for(auto p:(*cloud_filtered).points)
    {
        octomap::point3d endpoint(p.x, p.y, p.z);
        octocloud.push_back(endpoint);
    }

    tree.insertPointCloud(octocloud, sensor_origin);
    tree.updateInnerOccupancy();
    outputStatistics(tree);
    // tree.writeBinary("sample.bt");
    tree.write("sam.ot");

    if(config.view_cloud)
        viewCloud(cloud);

    if(config.view_filtered_cloud)
        viewCloud(cloud_filtered);

    if(config.save_ascii)
        saveAsASCII(config.save_pcd_path, cloud_filtered);
    
    std::cout << "pcd_process ended." << std::endl;      
    return 0;
}