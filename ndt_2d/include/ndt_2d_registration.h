#ifndef NDT_2D_REG_CERES_H
#define NDT_2D_REG_CERES_H

#include <ros/ros.h>
#include "eigen3/Eigen/Dense"
#include "ceres/ceres.h"
#include "glog/logging.h"
#include "ndt_2d_math.h"
#include "ndt_2d_map.h"
#include "ndt_2d_scan.h"
#include <cmath>
#include <cfloat>
#include <iostream>
#include <sstream>
#include <string>

namespace ndt2d
{

  class NDT2DStandardCost : public ceres::FirstOrderFunction
  {
  private:
    NDTMap *fixed_scan_;
    NDTMap *movable_scan_;
    std::vector<Gaussian> fixed_gaussians_;
    std::vector<Gaussian> movable_gaussians_;
    const double d1_;
    const double d2_;

  public:
    NDT2DStandardCost(
        NDTMap *fixed_scan, NDTMap *movable_scan,
        double d1 = 1.0, double d2 = 0.05, double rematch_limit = 0.5)
        : d1_(d1), d2_(d2)
    {
      this->fixed_scan_ = fixed_scan;
      this->movable_scan_ = movable_scan;
      matchGaussians();
    }

    virtual ~NDT2DStandardCost() {}

    virtual bool Evaluate(const double *parameters,
                          double *cost, double *gradient) const
    {
      double tx = parameters[0];
      double ty = parameters[1];
      double rz = parameters[2];
      TransformationVector trans(tx, ty, rz);

      //Check for jacobians
      bool grad = false;
      if (gradient != NULL)
      {
        grad = true;
      }

      double cost_sum = 0.0;
      Eigen::Vector3d grad_sum = Eigen::Vector3d::Zero();
      for (unsigned int i = 0; i < fixed_gaussians_.size(); i++)
      {
        Eigen::Vector2d mean_vector = StandardMath::calcMeanVector(
            movable_gaussians_[i],
            fixed_gaussians_[i],
            trans);

        Eigen::Matrix2d B = StandardMath::calcB(
            movable_gaussians_[i],
            fixed_gaussians_[i],
            trans);

        double cost = StandardMath::calcCost(mean_vector, B, d1_);
        cost_sum += cost;
        //std::cout << "cost: " << cost_sum << "\n";

        if (grad)
        {
          Eigen::Vector3d q_vec = StandardMath::q(
              movable_gaussians_[i],
              trans,
              mean_vector,
              B);

          Eigen::Vector3d gradient = StandardMath::calcGradient(
              mean_vector,
              B,
              q_vec,
              d1_,
              d2_);

          grad_sum += gradient;
        }
      }
      cost[0] = cost_sum;

      if (grad)
      {
        gradient[0] = grad_sum(0);
        gradient[1] = grad_sum(1);
        gradient[2] = grad_sum(2);
      }
      return true;
    }

    virtual int NumParameters() const { return 3; }

  private:
    void matchGaussians()
    {
      this->fixed_gaussians_ = fixed_scan_->getGaussianVector();
      this->movable_gaussians_ = movable_scan_->getGaussianVector();

      ROS_INFO("Before matching: Fixed gaussians size %d, Movable gaussians size %d",
               (int)fixed_gaussians_.size(), (int)movable_gaussians_.size());
      Gaussian::matchClosestGaussians(&fixed_gaussians_, &movable_gaussians_);
      ROS_INFO("After matching: Fixed gaussians size %d, Movable gaussians size %d",
               (int)fixed_gaussians_.size(), (int)movable_gaussians_.size());
    }
  };

  class NDT2DStepCost : public ceres::FirstOrderFunction
  {
  private:
    NDTMap *fixed_scan_;
    NDTMap *movable_scan_;
    std::vector<Gaussian> fixed_gaussians_;
    std::vector<Gaussian> movable_gaussians_;
    const double d1_;
    const double d2_;

  public:
    NDT2DStepCost(
        NDTMap *fixed_scan, NDTMap *movable_scan,
        double d1 = 1.0, double d2 = 0.05, double rematch_limit = 0.5)
        : d1_(d1), d2_(d2)
    {
      this->fixed_scan_ = fixed_scan;
      this->movable_scan_ = movable_scan;
      matchGaussians();
    }

    virtual ~NDT2DStepCost() {}

    virtual bool Evaluate(const double *parameters,
                          double *cost, double *gradient) const
    {
      double tx = parameters[0];
      double ty = parameters[1];
      double rz = parameters[2];
      TransformationVector trans_parameters(tx, ty, rz);
      std::vector<Gaussian> transformed_movable_gaussians = this->movable_gaussians_;

      //Transform gaussians according to new parameters
      //ROS_INFO_STREAM("Movable transformation " << trans_parameters.to_string());
      for (unsigned int i = 0; i < movable_gaussians_.size(); i++)
      {
        transformed_movable_gaussians[i].transform(trans_parameters);
      }

      //Check for jacobians
      bool grad = false;
      if (gradient != NULL)
      {
        grad = true;
      }

      double cost_sum = 0.0;
      Eigen::Vector3d grad_sum = Eigen::Vector3d::Zero();
      for (unsigned int i = 0; i < fixed_gaussians_.size(); i++)
      {
        Eigen::Vector2d mean_vector = StepMath::calcMeanVector(
            transformed_movable_gaussians[i],
            fixed_gaussians_[i]);

        Eigen::Matrix2d B = StepMath::calcB(
            transformed_movable_gaussians[i],
            fixed_gaussians_[i]);

        double cost = StepMath::calcObjectiveFunction(mean_vector, B, d1_, d2_);
        cost_sum += cost;
        //std::cout << "cost: " << cost_sum << "\n";

        if (grad)
        {
          Eigen::Vector3d q_vec = StepMath::q(
              transformed_movable_gaussians[i],
              mean_vector,
              B);

          Eigen::Vector3d gradient = StepMath::gradObjectiveFunction(
              mean_vector,
              B,
              q_vec,
              d1_,
              d2_);

          grad_sum += gradient;
        }
      }
      cost[0] = cost_sum;

      if (grad)
      {
        gradient[0] = grad_sum(0);
        gradient[1] = grad_sum(1);
        gradient[2] = grad_sum(2);
      }
      return true;
    }

    virtual int NumParameters() const { return 3; }

  private:
    void matchGaussians()
    {
      this->fixed_gaussians_ = fixed_scan_->getGaussianVector();
      this->movable_gaussians_ = movable_scan_->getGaussianVector();

      ROS_INFO("Before matching: Fixed gaussians size %d, Movable gaussians size %d",
               (int)fixed_gaussians_.size(), (int)movable_gaussians_.size());
      Gaussian::matchClosestGaussians(&fixed_gaussians_, &movable_gaussians_);
      ROS_INFO("After matching: Fixed gaussians size %d, Movable gaussians size %d",
               (int)fixed_gaussians_.size(), (int)movable_gaussians_.size());
    }
  };

  enum class CostFunction
  {
    Standard = 0,
    Step = 1
  };

  class NDTRegistrationCeres
  {
  private:
    NDTMap *fixed_scan_;
    NDTMap *movable_scan_;
    CostFunction cost_function_;
    TransformationVector parameters_;
    double d1_;
    double d2_;

    //Ceres stuff
    ceres::GradientProblemSolver::Summary summary_;
    ceres::GradientProblemSolver::Options options_;

  public:
    NDTRegistrationCeres(const CostFunction &cost_function, double d1 = 1.0, double d2 = 0.05)
    {
      this->d1_ = d1;
      this->d2_ = d2;
      this->cost_function_ = cost_function;
      this->parameters_ = TransformationVector();
      this->options_.minimizer_progress_to_stdout = false;
    }

    NDTRegistrationCeres(const CostFunction &cost_function, const TransformationVector &initial_guess,
                         double d1 = 1.0, double d2 = 0.05)
        : NDTRegistrationCeres(cost_function, d1, d2)
    {
      this->parameters_ = initial_guess;
    }

    void setMaps(NDTMap *fixed_scan, NDTMap *movable_scan)
    {
      this->parameters_ = TransformationVector();
      this->fixed_scan_ = fixed_scan;
      this->movable_scan_ = movable_scan;
    }

    void setTransformation(TransformationVector vector)
    {
      this->parameters_ = vector;
    }

    TransformationVector getTransformation() const
    {
      return parameters_;
    }

    void setCostFunction(CostFunction cost_function)
    {
      this->cost_function_ = cost_function;
    }

    const CostFunction &getCostFunction() const
    {
      return cost_function_;
    }

    void optimize()
    {
      ROS_DEBUG_STREAM("Initial parameters set to: " << parameters_.to_string());

      double parameters[3];
      parameters_.getParameterArray(parameters);

      ceres::FirstOrderFunction *cost;
      if (this->cost_function_ == CostFunction::Step)
        cost = new NDT2DStepCost(fixed_scan_, movable_scan_);
      else
        cost = new NDT2DStandardCost(fixed_scan_, movable_scan_);

      ceres::GradientProblem problem(cost);

      ceres::Solve(options_, problem, parameters, &summary_);

      parameters_.setTransformationParameters(parameters);
      ;
    }

    //Summary functions
    void printFullReport() const
    {
      std::cout << summary_.FullReport() << "\n";
    }

    double getTimeTaken() const
    {
      return summary_.total_time_in_seconds;
    }

    int getNumberOfIterations() const
    {
      return summary_.iterations.size();
    }

    // Options
    std::string options_to_string() const
    {
      std::stringstream ss;
      std::string error;
      if (!options_.IsValid(&error))
      {
        ss << "Options not valid: " << error << "\n";
      }

      ss << "line_search_type: "
         << options_.line_search_type << "\n"
         << "line_search_direction_type: "
         << options_.line_search_direction_type << "\n"
         << "minimizer_progress_to_stdout: "
         << options_.minimizer_progress_to_stdout << "\n"
         << "max_line_search_step_contraction: "
         << options_.max_line_search_step_contraction << "\n"
         << "min_line_search_step_contraction: "
         << options_.min_line_search_step_contraction << "\n";

      return ss.str();
    }
  };
}; // namespace ndt2d

#endif