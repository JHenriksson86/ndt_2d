#ifndef NDT_2D_MAP_CELL_H
#define NDT_2D_MAP_CELL_H

#include "eigen3/Eigen/Dense"
#include <vector>
#include <iostream>
#include <numeric>
#include <cmath>
#include "ndt_2d_util.h"

namespace ndt2d{
 
    class NDTCell {
        protected:

        std::vector<UVector2d>          scan_points_;
        Gaussian                        gaussian_;
        unsigned int                    total_number_of_scps_;
        const unsigned int              min_scan_points_;
        bool                            has_gaussian_;

        public:
        NDTCell(unsigned int min_scan_points = 3) 
            : min_scan_points_(min_scan_points)
        {
            init();
        }

        NDTCell(const NDTCell& cell)
            : min_scan_points_(cell.min_scan_points_)
        {
            scan_points_ = cell.scan_points_;
            gaussian_ = cell.gaussian_;
            total_number_of_scps_ = cell.total_number_of_scps_;
            has_gaussian_ = cell.has_gaussian_;
        }

        void clear()
        {
            init();
        }

        unsigned int getUnregisteredSCPS() const { return scan_points_.size(); }

        unsigned int getRegisteredSCPS() const { return total_number_of_scps_; }

        bool hasGaussian() const { return has_gaussian_; }

        UVector2d getMean() const { return gaussian_.getMean(); }

        UMatrix2d getCovariance() const { return gaussian_.getCovariance(); }

        Gaussian getGaussian() const { return gaussian_; }

        void addScanPoint(const UVector2d& scan_point)
        {
            UVector2d point(scan_point[0], scan_point[1]);
            scan_points_.push_back(point);
        }

        virtual bool calculateGaussian()
        {
            if(scan_points_.size() < min_scan_points_)
            {
                init();
                return false;
            }
                
            double n = (double)scan_points_.size();

            UVector2d mean = UVector2d::Zero();
            for(unsigned int i = 0; i < scan_points_.size(); i++)
            {
                mean += scan_points_[i];
            }
            mean = (1.0/n)*mean;

            UMatrix2d covariance_sum = UMatrix2d::Zero();
            for(unsigned int i = 0; i < scan_points_.size(); i++)
            {
                UVector2d scan_mean_diff = scan_points_[i] - mean;
                covariance_sum += scan_mean_diff*scan_mean_diff.transpose();
            }
            
            gaussian_.setMean(mean);
            gaussian_.setCovariance((1.0/(n-1.0))*covariance_sum);
            has_gaussian_ = true;

            total_number_of_scps_ = scan_points_.size();
            scan_points_.clear();

            return true;
        }

        ~NDTCell()
        {
            scan_points_.clear();
        }

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        
        private:

        virtual void init()
        {
            scan_points_.clear();
            has_gaussian_ = false;
            total_number_of_scps_ = 0;
        }

        std::vector<double> makeCovMatrixMessage() const
        {
            UMatrix2d cov = gaussian_.getCovariance();
            double cov_array[] = {
                cov(0,0), cov(0,1), 0.0,
                cov(1,0), cov(1,1), 0.0,
                0.0, 0.0, 0.0
            };

            std::vector<double> msg(cov_array, cov_array + 9);
            return msg;
        }
    };

    class NDTOMCell : public NDTCell
    {
        private:
        UVector2d T_; //Unnormalized mean
        UMatrix2d S_; //Unnormalized covariance

        public:
        NDTOMCell(unsigned int min_scan_points = 3)
            : NDTCell(min_scan_points)
        {
            init();
        }

        ~NDTOMCell()
        {
            scan_points_.clear();
        }

        virtual bool calculateGaussian()
        {
            if(scan_points_.size() < min_scan_points_)
                return false;

            double n = (double)scan_points_.size();
            double m = (double)total_number_of_scps_;

            UVector2d T = calculateT();
            UMatrix2d S = calculateS(T, n);

            if(has_gaussian_)
            {
                double S_norm = m/(n*(m+n));
                UVector2d T_diff = ((n/m)*T_-T);
                S_ += S + S_norm*(T_diff*T_diff.transpose());
                T_ += T;
            }
            else
            {
                S_ = S;
                T_ = T;
                has_gaussian_ = true;
            }

            total_number_of_scps_ += scan_points_.size();
            scan_points_.clear();

            gaussian_.setMean(T_ * (1.0/(n+m)));
            gaussian_.setCovariance(S_ * (1.0/(n+m-1)));

            return true;
        }

        private:

        virtual void init()
        {
            scan_points_.clear();
            total_number_of_scps_ = 0;
            has_gaussian_ = false;
            T_ = UVector2d::Zero();
            S_ = UMatrix2d::Zero();
        }

        UVector2d calculateT()
        {
            UVector2d sum = UVector2d::Zero();
            for(unsigned int i = 0; i < scan_points_.size(); i++)
                sum += scan_points_[i];
                
            return sum;
        }

        UMatrix2d calculateS(const UVector2d& T, double number_of_scan_points)
        {
            UMatrix2d S = UMatrix2d::Zero();
            UVector2d norm_T = (1.0/number_of_scan_points) * T;

            for(unsigned int i = 0; i < scan_points_.size(); i++)
            {
                UVector2d temp_vector = scan_points_[i]-norm_T;
                S += temp_vector*temp_vector.transpose();
            }
            return S;
        }
    };
};

#endif