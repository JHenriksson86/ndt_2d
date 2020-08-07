#ifndef REGISTRATION_EVALUATION_H
#define REGISTRATION_EVALUATION_H
#define _USE_MATH_DEFINES

// STL
#include <string>
#include <iostream>
#include <cmath>
#include <fstream>

// ROS
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// Eigen
#include "eigen3/Eigen/Geometry"

// NDT
#include "ndt_msgs/Registration.h"
#include "ndt_msgs/NDT2dParameters.h"
#include "ndt_eval/pointcloud_handler.h"
#include "ndt_eval/transform_handler.h"

namespace ndt2d
{

  class RegistrationEvaluation
  {
    private:
    ros::NodeHandle* nh_;
    ros::ServiceClient registration_client_;
    ros::Publisher fixed_cloud_pub_;
    ros::Publisher movable_cloud_pub_;
    ros::Publisher result_cloud_pub_;
    PointCloudHandler* fixed_cloud_;
    PointCloudHandler* movable_cloud_;
    TransformHandler robot_pose_;
    ndt_msgs::NDT2dParameters registration_parameters_;
    std::string pcd_path_, pcd_name_, output_file_, odometry_file_;
    int pcd_start_, pcd_stop_, pcd_step_size_;
    bool initial_guess_;

    public:
    RegistrationEvaluation(ros::NodeHandle* nh);
    ~RegistrationEvaluation();
    void run();
    
    private:
    geometry_msgs::Transform callRegistrationService(
      ndt_msgs::Registration& srv, PointCloudHandler* fixed_cloud, 
      PointCloudHandler* movable_cloud, const geometry_msgs::Transform& initial_guess);
    void saveEstimate() const;
    void getPCDParameters();
    void getRegistrationParameters();
    Eigen::Affine3d getScannerTransform() const;
  };

}

#endif