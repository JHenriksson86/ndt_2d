#ifndef NDT_2D_MAP_H
#define NDT_2D_MAP_H

// STL
#include <vector>
#include <list>
#include <iostream>
#include <cmath>

// ROS
#include <ros/ros.h>

// Eigen
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/StdVector"

// NDT
#include "ndt_2d_mapcell.h"
#include "ndt_2d_scan.h"
#include "ndt_2d_util.h"

namespace ndt2d
{
  class NDTMap
  {
  private:
    std::vector<std::vector<NDTCell>> map_;
    UMatrix2d rotation_;
    UVector2d translation_;
    const int rows_;
    const int columns_;
    const double cell_size_;

  public:
    NDTMap(int rows = 20, int columns = 20, double cell_size = 1.0)
        : rows_(rows), columns_(columns), cell_size_(cell_size)
    {
      initMap(rows, columns);
    }

    NDTMap(const LaserScan &scan, int rows = 20, int columns = 20,
           double cell_size = 1.0)
        : rows_(rows), columns_(columns), cell_size_(cell_size)
    {
      initMap(rows, columns);
      this->addScans(scan);
    }

    NDTMap(const PointCloud2d &pointcloud, int rows = 20, int columns = 20,
           double cell_size = 1.0)
        : rows_(rows), columns_(columns), cell_size_(cell_size)
    {
      initMap(rows, columns);
      this->addPointCloud(pointcloud);
    }

    unsigned int getRows() const { return rows_; }

    unsigned int getColumns() const { return columns_; }

    double getMapXSize() const { return ((double)columns_) * cell_size_; }

    double getMapYSize() const { return ((double)rows_) * cell_size_; }

    double getCellSize() const { return cell_size_; }

    void clear()
    {
      for (int row = 0; row < rows_; row++)
      {
        for (int col = 0; col < columns_; col++)
        {
          this->operator()(row, col).clear();
        }
      }
    }

    const NDTCell &operator()(unsigned int row, unsigned int column) const
    {
      return map_[row][column];
    }

    NDTCell &operator()(unsigned int row, unsigned int column)
    {
      return map_[row][column];
    }

    bool addScan(double x, double y) { return addScan(UVector2d(x, y)); }

    bool addScan(UVector2d point)
    {
      double rows = (double)rows_;
      double cols = (double)columns_;
      double xlimit = cell_size_ * (rows / 2.0);
      double ylimit = cell_size_ * (cols / 2.0);
      if (point[0] > xlimit || point[0] < -xlimit || point[1] > ylimit || point[1] < -ylimit)
      {
        ROS_WARN("Failed to add scan, outside of limits scan coordinates (%.2f, %.2f)",
                 point[0], point[1]);
        return false;
      }

      UVector2d transformed_point = rotation_ * point + translation_;
      transformed_point = transformed_point * (1.0 / cell_size_);
      UVector2d array_coords = Eigen::floor(transformed_point.array());

      ROS_DEBUG("Inserting point (%.2f,%.2f) at position (%d,%d) in array",
                point[0], point[1], (int)array_coords[0], (int)array_coords[1]);

      this->operator()(array_coords[0], array_coords[1]).addScanPoint(point);
      return true;
    }

    bool addScans(const LaserScan &scan)
    {
      ROS_DEBUG("Adding %d scans to ndt", (int)scan.size());

      for (unsigned int i = 0; i < scan.size(); i++)
      {
        this->addScan(scan[i].getCartesianCoordinates());
      }
      return true;
    }

    bool addPointCloud(const PointCloud2d &cloud)
    {
      ROS_DEBUG("Adding %d pointcloud to ndt", (int)cloud.numberOfPoints());

      for(int i = 0; i < cloud.numberOfPoints(); i++)
      {
        this->addScan(cloud.at(i));
      }
      return true;
    }

    void calculateGaussians()
    {
      for (int row = 0; row < rows_; row++)
      {
        for (int col = 0; col < columns_; col++)
        {
          this->operator()(row, col).calculateGaussian();
        }
      }
    }

    std::vector<Gaussian> getGaussianVector()
    {
      std::vector<Gaussian> gaussians;
      for (int row = 0; row < rows_; row++)
      {
        for (int col = 0; col < columns_; col++)
        {
          NDTCell cell = this->operator()(row, col);
          if (cell.hasGaussian())
          {
            gaussians.push_back(cell.getGaussian());
          }
        }
      }
      return gaussians;
    }

    ~NDTMap() {}

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  private:
    void initMap(int rows, int columns)
    {
      this->rotation_ << 0.0, -1.0, 1.0, 0.0;
      this->translation_ = UVector2d(rows, columns) * cell_size_ * 0.5;

      std::vector<NDTCell> column;
      for (int i = 0; i < rows; i++)
      {
        column.push_back(NDTCell());
      }

      map_ = std::vector<std::vector<NDTCell>>();
      for (int i = 0; i < columns; i++)
      {
        map_.push_back(column);
      }

      ROS_DEBUG("NDT map initiated with rows = %d and columns = %d", int(map_.size()), int(map_[0].size()));
    }

    Eigen::Vector3d getCellCenter(int row, int col)
    {
      Eigen::Vector3d return_vec;
      return_vec[0] = (double)(col - columns_ / 2) * cell_size_ + cell_size_ / 2.0;
      return_vec[1] = (double)(rows_ / 2 - row) * cell_size_ - cell_size_ / 2.0;
      return_vec[2] = 0.0;

      return return_vec;
    }
  };
} // namespace ndt2d

#endif