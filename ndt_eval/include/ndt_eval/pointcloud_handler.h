#ifndef PCD_HANDLER_H
#define PCD_HANDLER_H

// STL
#include <iostream>
#include <string>

// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>

// ROS
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Transform.h>

// Eigen
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"

// NDT
#include "ndt_eval/transform_handler.h"

namespace ndt2d
{
  class PointCloudHandler
  {
    private:
    typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

    PointCloud::Ptr cloud_;
    std::string frame_;
    std::string stamp_;
    
    public:
    PointCloudHandler(std::string frame = "/base_laser");

    PointCloudHandler(
      const sensor_msgs::PointCloud2& pc_message, std::string frame = "/base_laser");
    
    ~PointCloudHandler();

    int getNumberOfPoints() const;
    
    void loadPCD(std::string pcd_path);
    
    void setStamp(const std::string& stamp);
    
    const std::string& getStamp() const;
    
    sensor_msgs::PointCloud2 getPointCloud2Message(
      bool reduced = false, double grid_size = 1.0) const;
    
    sensor_msgs::PointCloud2 getPointCloud2Message(
      const geometry_msgs::Transform& transform, 
      bool reduced = false, double grid_size = 1.0) const;
    
    Eigen::MatrixXd getPointMatrix(
      bool reduced = false, double grid_size = 1.0) const;
    
    private:
    sensor_msgs::PointCloud2 pclToPC2(const PointCloud& pcl_cloud) const;
    
    void downsampleCloud(double grid_size, PointCloud::Ptr input_cloud, PointCloud& output_cloud) const;
  };

};

#endif