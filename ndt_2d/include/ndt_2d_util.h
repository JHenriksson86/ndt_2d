#ifndef NDT_2D_UTIL_H
#define NDT_2D_UTIL_H

// STL
#include <cmath>
#include <sstream>
#include <string>

// ROS
#include "geometry_msgs/Transform.h"

// Eigen
#include "eigen3/Eigen/Dense"

namespace ndt2d
{

  typedef Eigen::Matrix<double, 2, 1, Eigen::DontAlign> UVector2d;
  typedef Eigen::Matrix<double, 2, 2, Eigen::DontAlign> UMatrix2d;

  class TransformationVector
  {
  private:
    Eigen::Vector3d vector_;

  public:
    TransformationVector()
    {
      vector_ = Eigen::Vector3d::Zero();
    }

    TransformationVector(double x, double y, double angle)
    {
      vector_ = Eigen::Vector3d(x, y, angle);
    }

    TransformationVector(Eigen::Vector3d vector)
    {
      this->vector_ = vector;
    }

    TransformationVector(geometry_msgs::Transform transform_message)
    {
      setTransformation(transform_message);
    }

    TransformationVector(const TransformationVector &trans_vector)
    {
      this->vector_ = trans_vector.vector_;
    }

    double &operator[](unsigned int row)
    {
      return vector_[row];
    }

    const double &operator[](unsigned int row) const
    {
      return vector_[row];
    }

    TransformationVector operator+(const TransformationVector &obj) const
    {
      TransformationVector result;
      result.vector_ = this->vector_ + obj.vector_;
      return result;
    }

    TransformationVector operator-(const TransformationVector &obj) const
    {
      TransformationVector result;
      result.vector_ = this->vector_ - obj.vector_;
      return result;
    }

    TransformationVector calculateAbsoluteDifference(
        const TransformationVector &reference) const
    {
      Eigen::Vector3d result = this->vector_ + reference.getParameterVector();
      result = result.cwiseAbs();
      return TransformationVector(result);
    }

    void setTransformation(const geometry_msgs::Transform msgs)
    {
      vector_[0] = msgs.translation.x;
      vector_[1] = msgs.translation.y;
      vector_[2] = std::asin(msgs.rotation.z) * 2.0;
    }

    void setTransformationParameters(const double *array)
    {
      for (int i = 0; i < vector_.size(); i++)
      {
        vector_[i] = *(array + i);
      }
    }

    geometry_msgs::Transform getTransformMessage() const
    {
      geometry_msgs::Transform msg;
      msg.translation.x = vector_[0];
      msg.translation.y = vector_[1];
      msg.translation.z = 0.0;

      msg.rotation.x = 0.0;
      msg.rotation.y = 0.0;
      msg.rotation.z = std::sin(vector_[2] / 2.0);
      msg.rotation.w = std::cos(vector_[2] / 2.0);

      return msg;
    }

    UMatrix2d getRotationMatrix() const
    {
      double angle = vector_[2];
      UMatrix2d rot_matrix;
      rot_matrix << std::cos(angle), -std::sin(angle), std::sin(angle), std::cos(angle);

      return rot_matrix;
    }

    UVector2d getTranslationVector() const
    {
      return UVector2d(vector_[0], vector_[1]);
    }

    const Eigen::Vector3d &getParameterVector() const
    {
      return vector_;
    }

    void getParameterArray(double *out_array)
    {
      for (int i = 0; i < vector_.size(); i++)
      {
        *(out_array + i) = vector_[i];
      }
    }

    double getX() const { return vector_[0]; }

    double getY() const { return vector_[1]; }

    double getRotation() const { return vector_[2]; }

    std::string to_string() const
    {
      std::stringstream ss;
      ss << vector_[0] << "," << vector_[1] << "," << vector_[2];

      return ss.str();
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  class Gaussian
  {
  private:
    UVector2d mean_;
    UMatrix2d covariance_;

  public:
    Gaussian()
    {
      this->mean_ = UVector2d::Zero();
      this->covariance_ = UMatrix2d::Zero();
    }

    Gaussian(UVector2d mean, UMatrix2d covariance)
    {
      this->mean_ = mean;
      this->covariance_ = covariance;
    }

    Gaussian(const Gaussian &gaussian)
    {
      this->mean_ = gaussian.mean_;
      this->covariance_ = gaussian.covariance_;
    }

    const UVector2d &getMean() const { return mean_; }

    const UMatrix2d &getCovariance() const { return covariance_; }

    void setMean(const UVector2d &mean) { this->mean_ = mean; }

    void setCovariance(const UMatrix2d &covariance) { this->covariance_ = covariance; }

    void transform(const TransformationVector &transformation)
    {
      Gaussian temp = getTransformed(transformation);
      this->mean_ = temp.getMean();
      this->covariance_ = temp.getCovariance();
    }

    Gaussian getTransformed(const TransformationVector &transformation) const
    {
      UVector2d mean = transformation.getRotationMatrix() * this->mean_ +
                       transformation.getTranslationVector();

      UMatrix2d rot_matrix = transformation.getRotationMatrix();
      UMatrix2d covariance = rot_matrix.transpose() * this->covariance_ * rot_matrix;

      return Gaussian(mean, covariance);
    }

    static void matchClosestGaussians(std::vector<Gaussian> *in_out_m1,
                                      std::vector<Gaussian> *in_out_m2)
    {
      int number_of_gaussians;
      std::vector<Gaussian> *m1;
      std::vector<Gaussian> *m2;
      bool m1_smallest;
      if (in_out_m1->size() <= in_out_m2->size())
      {
        number_of_gaussians = in_out_m1->size();
        m1 = in_out_m1;
        m2 = in_out_m2;
        m1_smallest = true;
      }
      else
      {
        number_of_gaussians = in_out_m2->size();
        m1 = in_out_m2;
        m2 = in_out_m1;
        m1_smallest = false;
      }

      std::vector<Gaussian> sorted_gaussians;
      for (int i = 0; i < number_of_gaussians; i++)
      {
        sorted_gaussians.push_back(
            takeClosestGaussian(m1->at(i), m2));
      }

      if (m1_smallest)
        *in_out_m2 = sorted_gaussians;
      else
        *in_out_m1 = sorted_gaussians;
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  private:
    static Gaussian takeClosestGaussian(const Gaussian &gaussian, std::vector<Gaussian> *gaussians)
    {
      double best_score = DBL_MAX;
      int best_index = 0;
      UVector2d fixed_mean = gaussian.getMean();
      //UMatrix2d fixed_cov = gaussian.getCovariance();
      for (unsigned int i = 0; i < gaussians->size(); i++)
      {
        UVector2d movable_mean = gaussians->at(i).getMean();
        //UMatrix2d movable_cov = gaussians->at(i).getCovariance();
        double score = std::sqrt(
            std::pow(fixed_mean[0] - movable_mean[0], 2) +
            std::pow(fixed_mean[1] - movable_mean[1], 2));

        if (score < best_score)
        {
          best_score = score;
          best_index = i;
        }
      }

      Gaussian result = gaussians->at(best_index);
      ROS_DEBUG_STREAM("Closest gaussian result:\n"
                       << "best_score: " << best_score << "\n"
                       << "mean:\n"
                       << result.getMean() << "\n"
                       << gaussian.getMean() << "\n"
                       << "covariance:\n"
                       << result.getCovariance() << "\n"
                       << gaussian.getCovariance());

      gaussians->erase(gaussians->begin() + best_index);
      return result;
    }
  };

}; // namespace ndt2d

#endif