#include "functions.h"        

// Utility function used in readConfig that convert '(' , ')' and ',' to space.
std::string punc2space(std::string& str)
{
   for(std::string::iterator it = str.begin(); it != str.end(); ++it)
   {
      if(*it == '(' | *it == ')' | *it == ',')
         *it = ' ';
   }
   return str;
}

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
         else if (key == "view_filtered_cloud")
         {
            std::istringstream is(value);
            is >> std::boolalpha >> config.view_filtered_cloud;
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
            std::vector<float> v;
            float temp;

            std::istringstream is(punc2space(value));

            while(is >> temp)
            {
               v.push_back(temp);
            }

            config.min_vec = Eigen::Vector4f(v.data());
            //std::cout << config.min_vec << std::endl;

         }
         else if (key == "max_vec")
         {
            std::vector<float> v;
            float temp;

            std::istringstream is(punc2space(value));

            while(is >> temp)
            {
               v.push_back(temp);
            }

            config.max_vec = Eigen::Vector4f(v.data());
            //std::cout << config.max_vec << std::endl;
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

