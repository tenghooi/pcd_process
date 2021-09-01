# README

## pcd_process

**PCD_PROCESS** is a small application that uses point cloud library **(PCL)** to process point cloud data **(pcd)** format file.

## Main Functions

You can use **pcd_process** to 
- Read *ascii* or *binary* pcd files
- View the point cloud in pcd file
- Save the point cloud in *ascii* format pcd if input is *binary*
- Save the point cloud in *binary* format pcd if input is *ascii* (TODO)
- Crop the point cloud to a defined box size
- More to come ...

### **Note** 
- As of now only limited to process one pcd file at a time.
- Tested only on Ubuntu 18.04

## Prerequisites:
- Point Cloud Library (PCL) >= 1.8
- CMake >= 3.0
- GCC/G++ compiler >= 4.7 on Linux

## How To Use
### Building the code
After git cloning the repository, execute the following in the root folder:
```
mkdir build
cd build
cmake ..
make
```
### Running the application
At the root folder, run: `./pcl_process`

### Configuring the parameter
In the root folder, you can find the ***config.txt*** file. Use the file to specify the *input pcd path directory* and *save path directory*.

You can also set which function to activate using boolean value in the configuration file.

### TODO
-[ ] Allow for different point type. Currently works for pcl::PointXYZI.
-[ ] Function for saving ascii to binary format pcd.