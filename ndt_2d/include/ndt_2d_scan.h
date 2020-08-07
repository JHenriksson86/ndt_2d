#ifndef NDT_2D_SCAN_H
#define NDT_2D_SCAN_H

// STL
#include <iostream>
#include <vector>
#include <cmath>
#include <fstream>
#include <string>

// PCL
//#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
//#include <pcl/common/transforms.h>

// ROS
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>

// Eigen
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"

// NDT
#include "ndt_2d_util.h"

namespace ndt2d
{
  class LaserScanPoint
  {
  private:
    UVector2d polar_coordinates_;
    UVector2d cart_coordinates_;

  public:
    LaserScanPoint()
    {
      polar_coordinates_ = UVector2d::Zero();
      cart_coordinates_ = UVector2d::Zero();
    }

    LaserScanPoint(double distance, double angle)
    {
      this->setPolarCoordinates(distance, angle);
    }

    LaserScanPoint(const LaserScanPoint &point)
    {
      this->polar_coordinates_ = point.polar_coordinates_;
      this->cart_coordinates_ = point.cart_coordinates_;
    }

    void setPolarCoordinates(double distance, double angle)
    {
      polar_coordinates_ << distance, angle;
      cart_coordinates_ << distance * std::cos(angle),
          distance * std::sin(angle);
    }

    void setCartesianCoordinates(double x, double y)
    {
      cart_coordinates_ << x, y;
      polar_coordinates_ << std::sqrt(std::pow(x, 2) + std::pow(y, 2)),
          std::atan2(y, x);
    }

    UVector2d getCartesianCoordinates() const
    {
      return cart_coordinates_;
    }

    void transform(const TransformationVector &trans)
    {
      UVector2d coordinates = trans.getRotationMatrix() * cart_coordinates_ + trans.getTranslationVector();
      this->setCartesianCoordinates(coordinates[0], coordinates[1]);
    }

    LaserScanPoint getTransformed(const TransformationVector &trans) const
    {
      UVector2d coordinates = trans.getRotationMatrix() * cart_coordinates_ + trans.getTranslationVector();
      LaserScanPoint result;
      result.setCartesianCoordinates(coordinates[0], coordinates[1]);
      return result;
    }

    double getDistance() const { return polar_coordinates_[0]; }

    double getAngle() const { return polar_coordinates_[1]; }

    double getX() const { return cart_coordinates_[0]; }

    double getY() const { return cart_coordinates_[1]; }

    ~LaserScanPoint() {}

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  class LaserScan
  {
  private:
    std::vector<LaserScanPoint> points_;

  public:
    LaserScan()
    {
      points_.reserve(720);
    }

    LaserScan(const LaserScan &scan)
    {
      for (unsigned int i = 0; i < scan.size(); i++)
      {
        points_.push_back(scan.points_[i]);
      }
    }

    void clear() { points_.clear(); }

    void push_back(const LaserScanPoint &point)
    {
      points_.push_back(point);
    }

    void transform(const TransformationVector &trans)
    {
      for (unsigned int i = 0; i < points_.size(); i++)
      {
        points_[i].transform(trans);
      }
    }

    LaserScan getTransformed(const TransformationVector &trans) const
    {
      LaserScan result;
      for (unsigned int i = 0; i < points_.size(); i++)
      {
        result.push_back(points_[i].getTransformed(trans));
      }
      return result;
    }

    void addScanMessage(const sensor_msgs::LaserScan::ConstPtr &msg)
    {
      for (unsigned int i = 0; i < msg->ranges.size(); i++)
      {
        if (msg->ranges[i] > 0.0)
        {
          double angle = msg->angle_min + msg->angle_increment * (double)i;
          double distance = msg->ranges[i];
          LaserScanPoint point(distance, angle);
          points_.push_back(point);
        }
      }
    }

    unsigned int size() const { return points_.size(); }

    const LaserScanPoint &operator[](unsigned int index) const
    {
      return points_[index];
    }

    LaserScanPoint &operator[](unsigned int index)
    {
      return points_[index];
    }

    bool saveLaserScan(const std::string &filename, const std::string &path = "")
    {
      std::string output_filename = path + "/" + filename + ".scan";
      ROS_INFO("Saving data to %s", output_filename.c_str());

      std::ofstream output;
      output.open(output_filename.c_str(), std::ofstream::out | std::ofstream::trunc);

      for (unsigned int i = 0; i < points_.size(); i++)
      {
        output << points_[i].getX() << "," << points_[i].getY() << std::endl;
      }

      output.close();
      return true;
    }

    bool loadLaserScan(const std::string &filename, const std::string &path = "")
    {
      std::string input_filename = path + "/" + filename + ".scan";
      ROS_INFO("Loading data from %s", input_filename.c_str());

      std::ifstream input;
      input.open(input_filename.c_str(), std::ifstream::in);

      bool load_ok = true;
      while (load_ok)
      {
        std::string x, y;
        std::getline(input, x, ',');
        std::getline(input, y);
        load_ok = input.good();

        if (load_ok)
        {
          double x_cord = std::stod(x);
          double y_cord = std::stod(y);
          LaserScanPoint point;
          point.setCartesianCoordinates(x_cord, y_cord);
          points_.push_back(point);
        }
        else if (input.eof())
        {
          ROS_INFO("End of file reached.");
        }
        else
        {
          ROS_ERROR("Something went wrong with load.");
        }
      }

      input.close();
      return true;
    }

    visualization_msgs::Marker createMarker(double red, double green, double blue)
    {
      visualization_msgs::Marker marker;

      marker.header.frame_id = "world";
      marker.header.stamp = ros::Time::now();
      marker.ns = "my_namespace";
      marker.id = 0;
      marker.type = visualization_msgs::Marker::POINTS;
      marker.action = visualization_msgs::Marker::ADD;

      marker.pose.position.x = 0.0;
      marker.pose.position.y = 0.0;
      marker.pose.position.z = 0.0;

      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;

      marker.scale.x = 0.1;
      marker.scale.y = 0.1;
      marker.scale.z = 0.1;

      marker.color.a = 1.0; // Don't forget to set the alpha!
      marker.color.r = red;
      marker.color.g = green;
      marker.color.b = blue;

      for (unsigned int i = 0; i < points_.size(); i++)
      {
        if (points_[i].getDistance() > 0.0)
        {
          geometry_msgs::Point point;
          point.x = points_[i].getX();
          point.y = points_[i].getY();
          point.z = 0.0;
          marker.points.push_back(point);
        }
      }

      return marker;
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  class PointCloud2d
  {
    private:
    typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

    PointCloud cloud_;
    std::string     frame_;

    public:
    PointCloud2d(std::string frame = "world")
    {
      this->frame_ = frame;
    }

    PointCloud2d(const sensor_msgs::PointCloud2& pc2_message, std::string frame = "world")
    {
      this->frame_ = frame;
      setPointCloud(pc2_message);
    }

    ~PointCloud2d(){}

    int numberOfPoints() const
    {
      return int(cloud_.size());
    }

    UVector2d at(std::size_t n) const
    {
      pcl::PointXYZ point = cloud_.at(n);
      return UVector2d(point.x, point.y);
    }

    void setPointCloud(const sensor_msgs::PointCloud2& pc2_message)
    {
      pcl::fromROSMsg(pc2_message, cloud_);
    }

    private:

  };

} // namespace ndt2d

#endif