#ifndef NDT_2D_REG_NODE_H
#define NDT_2D_REG_NODE_H

#include <ros/ros.h>
#include "eigen3/Eigen/Dense"
#include "ceres/ceres.h"
#include "glog/logging.h"
#include "ndt_2d_map.h"
#include "ndt_2d_scan.h"
#include "ndt_2d_math.h"
#include <cmath>
#include <cfloat>
#include <iostream>

namespace ndt2d
{

    class NDTRegistration
    {
        private:
        NDTMap*                 fixed_scan_;
        NDTMap*                 movable_scan_;
        TransformationVector    trans_vector_;
        Math                    math_;
        double                  d1_;
        double                  d2_;
        unsigned int            max_iterations_;
        double                  step_size_;

        public:

        NDTRegistration(double max_iterations, double step_size = 0.1, 
            double d1 = 1.0, double d2 = 0.05)
        {
            this->d1_ = d1;
            this->d2_ = d2;
            this->math_ = Math(d1, d2);
            this->max_iterations_ = max_iterations;
            this->step_size_ = step_size;
        }

        void setMaps(NDTMap* fixed_scan, NDTMap* movable_scan)
        {
            this->fixed_scan_ = fixed_scan;
            this->movable_scan_ = movable_scan;
        }

        void setTransformation(TransformationVector vector)
        {
            this->trans_vector_ = vector;
        }

        TransformationVector getTransformation()
        {
            return trans_vector_;
        }

        void optimize()
        {
            std::vector<Gaussian> fixed_gaussians = fixed_scan_->getGaussianVector();
            std::vector<Gaussian> movable_gaussians = movable_scan_->getGaussianVector();
            
            //Match closest gaussians
            ROS_DEBUG("Before matching: Fixed gaussians size %d, Movable gaussians size %d",
                (int)fixed_gaussians.size(), (int)movable_gaussians.size());
            matchClosestGaussians(&fixed_gaussians, &movable_gaussians);
            ROS_DEBUG("After matching: Fixed gaussians size %d, Movable gaussians size %d",
                (int)fixed_gaussians.size(), (int)movable_gaussians.size());

            int iterations = 1;
            TransformationVector best_transformation = trans_vector_;
            double best_cost = DBL_MAX;
            while(true)
            {

                //Calculate cost, gradient and hessian
                Eigen::Vector3d gradient;
                Eigen::Matrix3d hessian;
                double cost = math_.calculateMeanCost(fixed_gaussians, movable_gaussians, 
                    &gradient, &hessian);
                if(cost < best_cost && cost != 0.0)
                {
                    best_transformation = trans_vector_;
                    best_cost = cost;
                }
                Eigen::Matrix3d hessian_inverse = Eigen::Matrix3d::Zero();
                bool invertible = false;
                //hessian.computeInverseWithCheck(hessian_inverse, invertible);
                if(!invertible)
                {
                    ROS_WARN("Hessian not invertible, testing pseudoinverse.");
                    Eigen::MatrixXd dynamic_hessian = hessian;
                    Eigen::JacobiSVD<Eigen::MatrixXd> svd(
                        dynamic_hessian, 
                        Eigen::ComputeFullU | Eigen::ComputeFullV);

                    Eigen::Vector3d singular = svd.singularValues();
                    Eigen::Matrix3d u = svd.matrixU();
                    Eigen::Matrix3d u_inv = u.inverse();
                    Eigen::Matrix3d v = svd.matrixV(); 
                    ROS_INFO_STREAM("Singular value decomposition result:\n"
                        << "S:\n" << singular << "\n"
                        << "V:\n" << v << "\n"
                        << "U:\n" << u << "\n"
                        << "U*:\n" << u_inv);

                    Eigen::Matrix3d singular_matrix = Eigen::Matrix3d::Zero();
                    for(int i = 0; i < singular.size(); i++)
                    {
                        if(singular(i) > 0.01)
                            singular_matrix(i,i) = 1.0 / singular(i);
                        else
                            singular_matrix(i,i) = singular(i);
                    }
                    hessian_inverse = v*singular_matrix*u_inv;
                    Eigen::Matrix3d error = hessian * hessian_inverse;
                    ROS_INFO_STREAM("Pseudoinverse result:\n" << hessian_inverse << "\n" 
                        << "Error:\n" << error);

                    if(!invertible)
                    {
                        ROS_ERROR("Pseudo inverse failed also quitting!");
                        break;
                    }
                    hessian_inverse = hessian_inverse * hessian.transpose();
                }

                ROS_INFO("Cost: %.2f", cost);
                ROS_INFO_STREAM("Gradient: \n" << gradient);
                ROS_INFO_STREAM("Hessian: \n" << hessian);
                //ROS_INFO_STREAM("Hessian inverse: \n" << hessian_inverse);

                Eigen::Vector3d newton_result = hessian_inverse * gradient;
                //Eigen::Vector3d newton_result = hessian.fullPivHouseholderQr().solve(gradient);
                double relative_error = (hessian * newton_result - gradient).norm() / gradient.norm();
                ROS_INFO_STREAM("Newton result:\n" << newton_result << "\n" 
                    << "Relative error: " << relative_error);

                Eigen::Vector3d scaled_transform = newton_result * (-1.0) * step_size_;
                TransformationVector transformation(scaled_transform);
                
                //Apply newton result
                for(int i = 0; i < movable_gaussians.size(); i++)
                {
                    movable_gaussians[i].transform(transformation);
                }

                trans_vector_ = trans_vector_ + transformation;

                ROS_INFO_STREAM("Optimization iteration nr: " << iterations 
                    << " result: \n" << newton_result);

                iterations++;
                if(iterations > max_iterations_)
                    break;
                if(newton_result[0] > 100.0 || newton_result[0] < -100.0)
                    break;
                if(newton_result[1] > 100.0 || newton_result[1] < -100.0)
                    break;
                if(newton_result[2] > 100.0 || newton_result[2] < -100.0)
                    break;
            }

            trans_vector_ = best_transformation;
        }

        void matchClosestGaussians(std::vector<Gaussian>* in_out_m1, 
            std::vector<Gaussian>* in_out_m2)
        {
            int number_of_gaussians;
            std::vector<Gaussian>* m1;
            std::vector<Gaussian>* m2;
            bool m1_smallest;
            if(in_out_m1->size() <= in_out_m2->size())
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
            for(int i = 0; i < number_of_gaussians; i++)
            {
                sorted_gaussians.push_back(
                    takeClosestGaussian(m1->at(i), m2)
                );
            }

            if(m1_smallest)
                *in_out_m2 = sorted_gaussians;
            else
                *in_out_m1 = sorted_gaussians;
             
        }

        private:

        Gaussian takeClosestGaussian(const Gaussian& gaussian, std::vector<Gaussian>* gaussians)
        {
            double best_score = DBL_MAX;
            int best_index = 0;
            Eigen::Vector2d fixed_mean = gaussian.getMean();
            Eigen::Matrix2d fixed_cov = gaussian.getCovariance();
            for(int i = 0; i < gaussians->size(); i++)
            {
                Eigen::Vector2d movable_mean = gaussians->at(i).getMean();
                Eigen::Matrix2d movable_cov = gaussians->at(i).getCovariance();
                double score = std::sqrt(
                    std::pow(fixed_mean[0]-movable_mean[0],2) + 
                    std::pow(fixed_mean[1]-movable_mean[1],2)
                );
                
                if(score < best_score)
                {
                    best_score = score;
                    best_index = i;
                }
            }

            Gaussian result = gaussians->at(best_index);
            ROS_DEBUG_STREAM("Closest gaussian result:\n" <<
                "best_score: " << best_score << "\n" <<
                "mean:\n" << result.getMean() << "\n" << gaussian.getMean() << "\n" <<
                "covariance:\n" << result.getCovariance() << "\n" << gaussian.getCovariance()
            );

            gaussians->erase(gaussians->begin() + best_index);
            return result;
        }
    };
};

#endif