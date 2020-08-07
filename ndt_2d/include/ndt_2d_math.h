#ifndef NDT_2D_MATH_H
#define NDT_2D_MATH_H

#include <ros/ros.h>
#include "ndt_2d_util.h"
#include "eigen3/Eigen/Dense"
#include <cmath>
#include <cfloat>
#include <iostream>

namespace ndt2d
{
    struct StepMath
    {
        static Eigen::Vector2d calcMeanVector(const Gaussian& movable_point, 
            const Gaussian& fixed_point)
        {
            return movable_point.getMean() - fixed_point.getMean();
        }

        static Eigen::Matrix<double,2,3> gradMeanVector(const Gaussian& movable_point)
        { 
            Eigen::Vector2d mean = movable_point.getMean();
            Eigen::Matrix<double,2,3> result;
            result << 1.0, 0.0, -mean[1],
                0.0, 1.0, mean[0];
            return result;
        }

        static Eigen::Matrix<double,6,3> hessianMeanVector(const Gaussian& movable_point)
        {
            Eigen::Vector2d mean = movable_point.getMean();
            Eigen::Matrix<double,6,3> result = Eigen::Matrix<double,6,3>::Zero();
            result(4,2) = -mean[0];
            result(5,2) = -mean[1];
            return result;
        }

        static Eigen::Matrix2d calcB(const Gaussian& movable_point, 
            const Gaussian& fixed_point) 
        {
            Eigen::Matrix2d cov_sum = 
                        movable_point.getCovariance() + 
                        fixed_point.getCovariance();
            
            Eigen::Matrix2d result;
            bool invertible;
            cov_sum.computeInverseWithCheck(result, invertible);
            if(!invertible)
                ROS_ERROR("B matrix not invertible!");

            return result;
        }

        static Eigen::Matrix<double,2,6> gradB(const Gaussian& movable_point)
        {
            Eigen::Matrix2d cov = movable_point.getCovariance();
            double xx = cov(0,0);
            double xy = cov(0,1);
            double yy = cov(1,1);
            Eigen::Matrix<double,2,6> result = Eigen::Matrix<double,2,6>::Zero();
            result(0,4) = 2.0*xy;
            result(0,5) = yy-xx;
            result(1,4) = result(0,5);
            result(1,5) = -2.0*xy;

            return result;
        }

        static Eigen::Matrix<double,6,6> hessianB(const Gaussian& movable_point)
        {
            Eigen::Matrix2d cov = movable_point.getCovariance();
            double xx = cov(0,0);
            double xy = cov(0,1);
            double yy = cov(1,1);
            Eigen::Matrix<double,6,6> result = Eigen::Matrix<double,6,6>::Zero();
            result(4,4) = 2.0*(yy-xx);
            result(4,5) = -4.0*xy;
            result(5,4) = result(4,5);
            result(5,5) = 2.0*(xx-yy);
            
            return result;
        }

        static Eigen::Vector3d q(const Gaussian& movable_point, 
            const Eigen::Vector2d& mean_vector, const Eigen::Matrix2d& B)
        {
            Eigen::Matrix<double,2,3> J = gradMeanVector(movable_point);
            Eigen::Matrix<double,2,6> Z = gradB(movable_point);

            ROS_DEBUG_STREAM("q input:\n" << 
                "mean_vector:\n" << mean_vector << std::endl <<
                "B:\n" << B << std::endl <<
                "J:\n" << J << std::endl <<
                "Z:\n" << Z
            );

            Eigen::Vector3d result(0.0, 0.0, 0.0);
            for(int i = 0; i < result.size(); i++)
            {
                Eigen::Vector2d J_temp = J.block<2,1>(0,i);
                Eigen::Matrix2d Z_temp = Z.block<2,2>(0,i*2);
                double exp1 = mean_vector.transpose()*B*J_temp;
                double exp2 = mean_vector.transpose()*B*Z_temp*B*mean_vector;
                result[i] = 2.0*exp1 - exp2;
            } 

            ROS_DEBUG_STREAM("q result: \n" << result);
            return result;
        }

        static double calculateMeanCost(std::vector<Gaussian>& fixed_gaussians, 
            std::vector<Gaussian>& movable_gaussians, Eigen::Vector3d* out_mean_gradient,
            Eigen::Matrix3d* out_mean_hessian)
        {
            if(fixed_gaussians.size() != movable_gaussians.size())
            {
                ROS_ERROR("Fixed (%d) and movable (%d) gaussians are not equal",
                    (int)fixed_gaussians.size(), (int)movable_gaussians.size());
                return 0.0;
            }

            double total_cost = 0.0;
            Eigen::Vector3d cost_gradient = Eigen::Vector3d::Zero();
            Eigen::Matrix3d cost_hessian = Eigen::Matrix3d::Zero();
            for(unsigned int i = 0; i < fixed_gaussians.size(); i++)
            {
                Gaussian fixed = fixed_gaussians[i];
                Gaussian movable = movable_gaussians[i];

                Eigen::Vector2d mean_vector = calcMeanVector(movable, fixed);
                Eigen::Matrix2d B = calcB(movable, fixed);
                total_cost += calcObjectiveFunction(mean_vector, B);

                Eigen::Vector3d q_ = q(movable, mean_vector, B);
                cost_gradient += gradObjectiveFunction(mean_vector, B, q_);

                cost_hessian += hessianObjectiveFunction(movable, 
                    fixed, mean_vector, B, q_);
            }

            double N = (double)fixed_gaussians.size();
            *out_mean_gradient = cost_gradient * (1.0/N);
            *out_mean_hessian = cost_hessian * (1.0/N); 
            return total_cost * (1.0/N);
        }

        static double calcObjectiveFunction(const Eigen::Vector2d& mean_vector, 
            const Eigen::Matrix2d& B, double d1 = 1.0, double d2 = 0.05)
        {
            return -d1*std::exp(
                (-d2/2.0)*mean_vector.transpose()*B*mean_vector
            );
        }

        static Eigen::Vector3d gradObjectiveFunction(const Eigen::Vector2d& mean_vector, 
            const Eigen::Matrix2d& B, const Eigen::Vector3d& q, 
            double d1 = 1.0, double d2 = 0.05)
        {
            double exponent = std::exp(
                (-d2/2.0)*mean_vector.transpose()*B*mean_vector
            );
            double scaling = d1*d2/2.0;
            
            Eigen::Vector3d result = scaling * q * exponent; 
            ROS_DEBUG_STREAM("gradObjectiveFunction result\n" <<
                result);
            return result;
        }

        static Eigen::Matrix3d hessianObjectiveFunction(const Gaussian& movable_point, 
            const Gaussian& fixed_point, const Eigen::Vector2d& mean_vector, 
            const Eigen::Matrix2d& B, const Eigen::Vector3d& q, 
            double d1 = 1.0, double d2 = 0.05)
        {
            Eigen::Matrix<double,6,3> Hab = hessianMeanVector(movable_point);
            Eigen::Matrix<double,6,6> Zab = hessianB(movable_point);
            Eigen::Matrix<double,2,3> Ja = gradMeanVector(movable_point);
            Eigen::Matrix<double,2,6> Za = gradB(movable_point);
            Eigen::Matrix<double,2,6> Zb = gradB(fixed_point);
            Eigen::Matrix<double,1,2> mean_transpose = mean_vector.transpose();
            const double exponent = std::exp(
                (-d2/2.0)*mean_transpose*B*mean_vector
            );

            ROS_DEBUG_STREAM("hessianObjectiveFunction\n" <<
                "exponent: " << exponent << "\n" <<
                "mean:\n" << mean_vector << "\n" <<
                "B:\n" << B << "\n" <<
                "Ja:\n" << Ja << "\n" <<
                "Za:\n" << Za << "\n" <<
                "Zb:\n" << Zb << "\n" <<
                "Hab:\n" << Hab << "\n" <<
                "Zab:\n" << Zab
            );

            Eigen::Matrix3d result = Eigen::Matrix3d::Zero();
            for(int row = 0; row < result.rows(); row++)
            {
                Eigen::Vector2d temp_Ja = Ja.block<2,1>(0,row);
                Eigen::Matrix2d temp_Za = Za.block<2,2>(0,row*2);
                double exp1 = temp_Ja.transpose() * B * temp_Ja;
                exp1 -= 2.0 * mean_transpose * B * temp_Za * temp_Ja;
                
                for(int col = 0; col < result.cols(); col++)
                {
                    double exp2 = exp1;
                    Eigen::Vector2d temp_Hab = Hab.block<2,1>(row*2, col);
                    exp2 += mean_transpose * B * temp_Hab;
                    Eigen::Matrix2d temp_Zb = Zb.block<2,2>(0, col*2);
                    exp2 -= mean_transpose * B * temp_Za * B * temp_Zb * B *
                        mean_vector;
                    Eigen::Matrix2d temp_Zab = Zab.block<2,2>(row*2, col*2);
                    exp2 -= 0.5 * mean_transpose * B * temp_Zab * B * mean_vector;
                    exp2 -= (d2/4.0) * q.transpose() * q;
                    result(row,col) = d1*d2* exp2 * exponent;
                }
            }
            
            ROS_DEBUG_STREAM("hessianObjectiveFunction result\n" <<
                result);
            return result;
        }
    };

    struct StandardMath
    {
        static Eigen::Vector2d calcMeanVector(const Gaussian& movable_point, 
            const Gaussian& fixed_point, const TransformationVector& trans)
        {
            Gaussian temp_move = movable_point.getTransformed(trans);
            return temp_move.getMean() - fixed_point.getMean();
        }

        static Eigen::Matrix2d calcB(const Gaussian& movable_point, 
            const Gaussian& fixed_point, const TransformationVector& trans)
        {
            Gaussian temp_move = movable_point.getTransformed(trans);
            Eigen::Matrix2d cov_sum = temp_move.getCovariance() 
                + fixed_point.getCovariance();
            
            Eigen::Matrix2d result;
            bool invertible;
            cov_sum.computeInverseWithCheck(result, invertible);
            if(!invertible)
                ROS_ERROR("B matrix not invertible!");

            ROS_DEBUG_STREAM("calcB:\n"
                << "result*cov_sum:\n" << result*cov_sum
            );
            return result;
        }

        static Eigen::Matrix<double,2,3> gradMeanVector(const Gaussian& movable_point,
            const TransformationVector& trans)
        { 
            double s_rz = std::sin(trans.getRotation());
            double c_rz = std::cos(trans.getRotation());
            Eigen::Vector2d mean = movable_point.getMean();

            Eigen::Matrix<double,2,3> result;
            result << 1.0, 0.0, -mean[0]*s_rz -mean[1]*c_rz,
                0.0, 1.0, mean[0]*c_rz-mean[1]*s_rz;
            
            ROS_DEBUG_STREAM("gradMeanVector:\n" 
                << "mean:\n" << mean << "\n"
                << "rz:" << trans.getRotation() << "\n"
                << "result:\n" << result
            );
            return result;
        }

        static Eigen::Matrix<double,2,6> gradB(const Gaussian& movable_point,
            const TransformationVector& trans)
        {
            double s_rz = std::sin(trans.getRotation());
            double c_rz = std::cos(trans.getRotation());
            Eigen::Matrix2d rot_matrix = trans.getRotationMatrix();
            Eigen::Matrix2d d_rot_matrix;
            d_rot_matrix << -s_rz, -c_rz, c_rz, -s_rz; 
            
            Eigen::Matrix2d cov = movable_point.getCovariance();
            Eigen::Matrix<double,2,6> result = Eigen::Matrix<double,2,6>::Zero();
            result.block<2,2>(0,4) = d_rot_matrix.transpose()*cov*rot_matrix +
                rot_matrix.transpose()*cov*d_rot_matrix;

            ROS_DEBUG_STREAM("gradB:\n" << result);
            return result;
        }

        static Eigen::Vector3d q(const Gaussian& movable_point, const TransformationVector& trans, 
            const Eigen::Vector2d& mean_vector, const Eigen::Matrix2d& B)
        {
            Eigen::Matrix<double,2,3> J = gradMeanVector(movable_point, trans);
            Eigen::Matrix<double,2,6> Z = gradB(movable_point, trans);

            ROS_DEBUG_STREAM("q input:\n" << 
                "mean_vector:\n" << mean_vector << std::endl <<
                "B:\n" << B << std::endl <<
                "J:\n" << J << std::endl <<
                "Z:\n" << Z
            );

            Eigen::Vector3d result(0.0, 0.0, 0.0);
            for(int i = 0; i < result.size(); i++)
            {
                Eigen::Vector2d J_temp = J.block<2,1>(0,i);
                Eigen::Matrix2d Z_temp = Z.block<2,2>(0,i*2);
                double exp1 = mean_vector.transpose()*B*J_temp;
                double exp2 = mean_vector.transpose()*B*Z_temp*B*mean_vector;
                ROS_DEBUG_STREAM("q loop" << i << ":\n"
                "J_temp:\n" << J_temp << std::endl <<
                "Z_temp:\n" << Z_temp << std::endl <<
                "exp1:" << exp1 << std::endl <<
                "exp2:" << exp2
            );
                result[i] = 2.0*exp1 - exp2;
            } 

            ROS_DEBUG_STREAM("q result: \n" << result);
            return result;
        }

        static double calcLikelyhood(const Eigen::Vector2d& mean_vector, 
            const Eigen::Matrix2d& B, double d1 = 1.0, double d2 = 0.05)
        {
            //Eigen::Matrix<double,1,2> mean_t = mean_vector.transpose();
            return std::exp((-d2/2.0)*mean_vector.dot(B*mean_vector));
        }

        static double calcCost(const Eigen::Vector2d& mean_vector, 
            const Eigen::Matrix2d& B, double d1 = 1.0)
        {
            return -d1*calcLikelyhood(mean_vector, B);
        }

        static Eigen::Vector3d calcGradient(const Eigen::Vector2d& mean_vector, 
            const Eigen::Matrix2d& B, const Eigen::Vector3d& q, 
            double d1 = 1.0, double d2 = 0.05)
        {
            double scaling = d1*d2/2.0;
            
            Eigen::Vector3d result = scaling * q * calcLikelyhood(mean_vector, B); 
            ROS_DEBUG_STREAM("gradObjectiveFunction result\n" <<
                result);
            return result;
        }
    };
};

#endif