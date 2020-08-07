#ifndef NDT_2D_H
#define NDT_2D_H

// ROS
#include <ros/ros.h>
#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/Transform.h"

// NDT
#include "ndt_2d_map.h"
#include "ndt_2d_scan.h"
#include "ndt_2d_registration.h"
#include "ndt_msgs/Registration.h"
#include "ndt_msgs/NDT2dParameters.h"

namespace ndt2d
{
  class NDT2dService
  {
  private:
    ros::NodeHandle             *nh_;
    ros::ServiceServer          service_;
    PointCloud2d                fixed_cloud_;
    PointCloud2d                movable_cloud_;
    TransformationVector        initial_guess_;
    TransformationVector        result_transform_;
    
    int map_cell_count_;
    double map_cell_size_;

  public:
    NDT2dService(){}

    void subscribeAndAdvertise(ros::NodeHandle *node_handle)
    {
      this->nh_ = node_handle;
      this->service_ = nh_->advertiseService("registration", &NDT2dService::serviceCallback, this);
    }

    void runRegistration()
    {
      NDTRegistrationCeres registration(CostFunction::Step);
      
      // Fixed NDT
      NDTMap fixed_ndt(fixed_cloud_, map_cell_count_, map_cell_count_, map_cell_size_);
      fixed_ndt.calculateGaussians();

      // Movable NDT
      NDTMap movable_ndt(movable_cloud_, map_cell_count_, map_cell_count_, map_cell_size_);
      movable_ndt.calculateGaussians();

      // Registration
      registration.setMaps(&fixed_ndt, &movable_ndt);
      registration.setTransformation(initial_guess_);
      registration.optimize();

      ROS_INFO("Time taken = %.4f, iterations = %d",
               registration.getTimeTaken(), registration.getNumberOfIterations());

      result_transform_ = registration.getTransformation();
    }

    ~NDT2dService() {}

  private:
    bool serviceCallback(
        ndt_msgs::Registration::Request &request,
        ndt_msgs::Registration::Response &response)
    {
      this->fixed_cloud_.setPointCloud(request.fixed_cloud);
      this->movable_cloud_.setPointCloud(request.movable_cloud);
      this->initial_guess_.setTransformation(request.initial_guess);
      this->map_cell_size_ = request.parameters.map_cell_size;
      this->map_cell_count_ = request.parameters.map_cell_count;
      ROS_INFO("Map cell size = %.2f, count = %d", map_cell_size_, map_cell_count_);

      runRegistration();

      geometry_msgs::Transform result_msg = result_transform_.getTransformMessage();
      ROS_INFO_STREAM("Registration result:\n" << result_msg);
      response.result = result_msg;
      return true;
    }
  };
}; // namespace ndt2d

#endif
