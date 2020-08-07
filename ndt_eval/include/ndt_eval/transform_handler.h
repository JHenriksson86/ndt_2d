#ifndef TRANSFORM_HANDLER_H
#define TRANSFORM_HANDLER_H

// STL
#include <iostream>
#include <string>
#include <sstream>
#include <fstream>
#include <limits>

// ROS
#include <ros/ros.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// Eigen
#include "eigen3/Eigen/Geometry"
#include "eigen3/Eigen/Dense"

namespace ndt2d
{
  class TransformHandler
  {
    private:
    Eigen::Affine3d registration_transform_;
    Eigen::Affine3d robot_pose_;
    Eigen::Affine3d odometry_;
    Eigen::Affine3d old_odometry_;
    Eigen::Affine3d robot_to_laser_;
    std::string stamp_;
    tf2_ros::TransformBroadcaster broadcaster_;
    tf2_ros::StaticTransformBroadcaster static_broadcaster_;

    Eigen::Affine3d global_sensor_pose_;
    
    public:
    TransformHandler();
    
    TransformHandler(const Eigen::Affine3d& robot_to_laser);
   
    TransformHandler(const TransformHandler& transform);
    
    ~TransformHandler();
    
    void setStamp(const std::string& stamp);
    
    const std::string& getStamp() const;

    void setRobotToLaserTransform(const Eigen::Affine3d& robot_to_laser);

    const Eigen::Affine3d& getRobotToLaserTransform() const;

    void setRobotPose(const Eigen::Affine3d& pose);

    const Eigen::Affine3d& getRobotPose() const;

    void addRegistrationResult(const geometry_msgs::Transform& transform);
    
    static geometry_msgs::Transform getInitialGuessMsg(
      double x, double y, double z, 
      double roll, double pitch, double yaw);
    
    geometry_msgs::Transform getInitialGuessMsg() const;
    
    geometry_msgs::Transform getTransformMsg() const;

    geometry_msgs::Transform getGlobalSensorPoseTransformMsg() const;
    
    std::string robotPoseToString() const;
    
    std::string to_string() const;
    
    void loadStamp(const std::string& file_path, int row);
    
    void loadOdometry(const std::string& file_path, int row);
    
    void broadcastTransform();
    
    void broadcastStaticTransform();

    static void transformMessageToEigenTransform(
      const geometry_msgs::Transform& msg, Eigen::Affine3d& transform_out);
    
    static void eigenTransformToTransformMessage(
      const Eigen::Affine3d& transform, geometry_msgs::Transform& msg_out);
    
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    private:
    std::ifstream& gotoLine(std::ifstream& file, int num);

    geometry_msgs::TransformStamped getStampedTransform(
      const Eigen::Affine3d& transform, const std::string& parent_frame, 
      const std::string& child_frame) const;

    void init();
  };
};

#endif