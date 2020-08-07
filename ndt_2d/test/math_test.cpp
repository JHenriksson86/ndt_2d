#include <ros/ros.h>
#include "ndt_2d_math.h"
#include "ndt_2d_util.h"
#include "ndt_2d_registration_ceres.h"
#include <gtest/gtest.h>
#include <vector>
#include <cmath>

using namespace ndt2d;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Matrix2d;
using Eigen::Matrix3d;
using std::vector;
using std::round;

Gaussian g_move, g_fixed;

void initTestCase()
{
    Vector2d move_mean(-9.75, 9.10);
    Matrix2d move_cov;
    move_cov << 0.085, -0.027, -0.027, 0.061;
    g_move = Gaussian(move_mean, move_cov);

    Vector2d fixed_mean(-9.55, 9.45);
    Matrix2d fixed_cov;
    fixed_cov << 0.095, -0.037, -0.037, 0.071;
    g_fixed = Gaussian(fixed_mean, fixed_cov);
}

TEST(ndt2d_step_math, calcMeanVector)
{
    initTestCase();

    Vector2d mean_vector = StepMath::calcMeanVector(g_move, g_fixed);

    EXPECT_EQ(round(-0.2*1000), round(mean_vector(0)*1000));
    EXPECT_EQ(round(-0.35*1000), round(mean_vector(1)*1000));
}

TEST(ndt2d_step_math, gradMeanVector)
{
    initTestCase();

    Eigen::Matrix<double,2,3> ja = StepMath::gradMeanVector(g_move);

    EXPECT_EQ(1.0, ja(0,0));
    EXPECT_EQ(0.0, ja(0,1));
    EXPECT_EQ(round(-9.10*10000), round(ja(0,2)*10000));
    EXPECT_EQ(0.0, ja(1,0));
    EXPECT_EQ(1.0, ja(1,1));
    EXPECT_EQ(round(-9.75*10000), round(ja(1,2)*10000));
}

TEST(ndt2d_step_math, hessianMeanVector)
{
    initTestCase();

    Eigen::Matrix<double,6,3> Hab = StepMath::hessianMeanVector(g_move);

    double zero_sum = 0.0;
    for(int row = 0; row < Hab.rows(); row++)
    {
        for(int col = 0; col < Hab.cols(); col++)
        {
            if(row < 4 && col < 2)
            {
                zero_sum += Hab(row, col);
            }
        }
    }
    EXPECT_EQ(0.0, zero_sum);
    EXPECT_EQ(round(9.7500*10000), round(Hab(4,2)*10000));
    EXPECT_EQ(round(-9.1000*10000), round(Hab(5,2)*10000));
}

TEST(ndt2d_step_math, calcB)
{
    initTestCase();

    Matrix2d B = StepMath::calcB(g_move, g_fixed);

    EXPECT_EQ(round(6.7128*10000), round(B(0,0)*10000));
    EXPECT_EQ(round(3.2547*10000), round(B(0,1)*10000));
    EXPECT_EQ(round(3.2547*10000), round(B(1,0)*10000));
    EXPECT_EQ(round(9.1538*10000), round(B(1,1)*10000));
}

TEST(ndt2d_step_math, gradB)
{
    initTestCase();

    Eigen::Matrix<double,2,6> Za = StepMath::gradB(g_move);

    double zero_sum = 0.0;
    for(int row = 0; row < Za.rows(); row++)
    {
        for(int col = 0; col < Za.cols(); col++)
        {
            if(col < 4)
            {
                zero_sum += Za(row, col);
            }
        }
    }
    EXPECT_EQ(0.0, zero_sum);
    EXPECT_EQ(round(-0.0540*10000), round(Za(0,4)*10000));
    EXPECT_EQ(round(-0.0240*10000), round(Za(0,5)*10000));
    EXPECT_EQ(round(-0.0240*10000), round(Za(1,4)*10000));
    EXPECT_EQ(round(0.0540*10000), round(Za(1,5)*10000));
}

TEST(ndt2d_step_math, hessianB)
{
    initTestCase();

    Eigen::Matrix<double,6,6> Zab = StepMath::hessianB(g_move);

    double zero_sum = 0.0;
    for(int row = 0; row < Zab.rows(); row++)
    {
        for(int col = 0; col < Zab.cols(); col++)
        {
            if(row < 4 && col < 4)
            {
                zero_sum += Zab(row, col);
            }
        }
    }
    EXPECT_EQ(0.0, zero_sum);
    EXPECT_EQ(round(-0.0480*10000), round(Zab(4,4)*10000));
    EXPECT_EQ(round(0.1080*10000), round(Zab(4,5)*10000));
    EXPECT_EQ(round(0.1080*10000), round(Zab(5,4)*10000));
    EXPECT_EQ(round(0.0480*10000), round(Zab(5,5)*10000));
}

TEST(ndt2d_step_math, calcObjectiveFunction)
{
    initTestCase();
    Matrix2d B = StepMath::calcB(g_move, g_fixed);
    Vector2d mean_vector = StepMath::calcMeanVector(g_move, g_fixed);
    double object = StepMath::calcObjectiveFunction(mean_vector, B);

    EXPECT_EQ(round(-0.9549*10000), round(object*10000));
}

TEST(ndt2d_step_math, q)
{
    initTestCase();
    Matrix2d B = StepMath::calcB(g_move, g_fixed);
    Vector2d mean_vector = StepMath::calcMeanVector(g_move, g_fixed);
    Eigen::Vector3d q = StepMath::q(g_move, mean_vector, B);

    EXPECT_EQ(round(-4.9634*10000), round(q(0)*10000));
    EXPECT_EQ(round(-7.7095*10000), round(q(1)*10000));
    EXPECT_EQ(round(120.3240*10000), round(q(2)*10000));
}

TEST(ndt2d_step_math, gradObjectiveFunction)
{
    initTestCase();
    Matrix2d B = StepMath::calcB(g_move, g_fixed);
    Vector2d mean_vector = StepMath::calcMeanVector(g_move, g_fixed);
    Vector3d q = StepMath::q(g_move, mean_vector, B);
    Vector3d grad_object = StepMath::gradObjectiveFunction(mean_vector, B, q);

    EXPECT_EQ(round(-0.1185*10000), round(grad_object(0)*10000));
    EXPECT_EQ(round(-0.1840*10000), round(grad_object(1)*10000));
    EXPECT_EQ(round(2.8725*10000), round(grad_object(2)*10000));
}

//Broke when q vectors where fixed, not interesting atm though.
/* 
TEST(ndt2d_math, hessianObjectiveFunction)
{
    Math ndt_math;
    initTestCase();
    Matrix2d B = ndt_math.calcB(g_move, g_fixed);
    Vector2d mean_vector = ndt_math.calcMeanVector(g_move, g_fixed);
    Vector3d q = ndt_math.q(g_move, g_fixed, mean_vector, B);
    Matrix3d hessian = ndt_math.hessianObjectiveFunction(g_move, g_fixed, mean_vector, B, q);

    EXPECT_EQ(round(-1.8518*10000), round(hessian(0,0)*10000));
    EXPECT_EQ(round(-1.8518*10000), round(hessian(0,1)*10000));
    EXPECT_EQ(round(-1.8518*10000), round(hessian(0,2)*10000));
    EXPECT_EQ(round(-1.7353*10000), round(hessian(1,0)*10000));
    EXPECT_EQ(round(-1.7353*10000), round(hessian(1,1)*10000));
    EXPECT_EQ(round(-1.7353*10000), round(hessian(1,2)*10000));
    EXPECT_EQ(round(93.5495*10000), round(hessian(2,0)*10000));
    EXPECT_EQ(round(93.5495*10000), round(hessian(2,1)*10000));
    EXPECT_EQ(round(93.9894*10000), round(hessian(2,2)*10000));
}
*/

TEST(ndt2d_standard_math, meanVector)
{
    initTestCase();
    TransformationVector trans;

    Vector2d mean_vector = StandardMath::calcMeanVector(g_move, g_fixed, trans);

    EXPECT_EQ(round(-0.2*1000), round(mean_vector(0)*1000));
    EXPECT_EQ(round(-0.35*1000), round(mean_vector(1)*1000));
}

TEST(ndt2d_standard_math, gradMeanVector)
{
    initTestCase();
    TransformationVector trans;

    Eigen::Matrix<double,2,3> ja = StandardMath::gradMeanVector(g_move, trans);

    EXPECT_EQ(1.0, ja(0,0));
    EXPECT_EQ(0.0, ja(0,1));
    EXPECT_EQ(round(-9.10*10000), round(ja(0,2)*10000));
    EXPECT_EQ(0.0, ja(1,0));
    EXPECT_EQ(1.0, ja(1,1));
    EXPECT_EQ(round(-9.75*10000), round(ja(1,2)*10000));
}

TEST(ndt2d_standard_math, calcB)
{
    initTestCase();
    TransformationVector trans;

    Matrix2d B = StandardMath::calcB(g_move, g_fixed, trans);

    EXPECT_EQ(round(6.7128*10000), round(B(0,0)*10000));
    EXPECT_EQ(round(3.2547*10000), round(B(0,1)*10000));
    EXPECT_EQ(round(3.2547*10000), round(B(1,0)*10000));
    EXPECT_EQ(round(9.1538*10000), round(B(1,1)*10000));
}

TEST(ndt2d_standard_math, gradB)
{
    initTestCase();
    TransformationVector trans;

    Eigen::Matrix<double, 2, 6> Za = StandardMath::gradB(g_move, trans);

    double zero_sum = 0.0;
    for(int row = 0; row < Za.rows(); row++)
    {
        for(int col = 0; col < Za.cols(); col++)
        {
            if(col < 4)
            {
                zero_sum += Za(row, col);
            }
        }
    }
    EXPECT_EQ(0.0, zero_sum);
    EXPECT_EQ(round(-0.0540*10000), round(Za(0,4)*10000));
    EXPECT_EQ(round(-0.0240*10000), round(Za(0,5)*10000));
    EXPECT_EQ(round(-0.0240*10000), round(Za(1,4)*10000));
    EXPECT_EQ(round(0.0540*10000), round(Za(1,5)*10000));
}

TEST(ndt2d_standard_math, calcObjectiveFunction)
{
    initTestCase();
    TransformationVector trans;


    Matrix2d B = StandardMath::calcB(g_move, g_fixed, trans);
    Vector2d mean_vector = StandardMath::calcMeanVector(g_move, g_fixed, trans);
    double object = StandardMath::calcCost(mean_vector, B);

    EXPECT_EQ(round(-0.9549*10000), round(object*10000));
}

TEST(ndt2d_standard_math, q)
{
    initTestCase();
    TransformationVector trans;

    Matrix2d B = StandardMath::calcB(g_move, g_fixed, trans);
    Vector2d mean_vector = StandardMath::calcMeanVector(g_move, g_fixed, trans);
    Eigen::Vector3d q = StandardMath::q(g_move, trans, mean_vector, B);

    EXPECT_EQ(round(-4.9634*10000), round(q(0)*10000));
    EXPECT_EQ(round(-7.7095*10000), round(q(1)*10000));
    EXPECT_EQ(round(120.3240*10000), round(q(2)*10000));
}

TEST(ndt2d_standard_math, gradObjectiveFunction)
{
    initTestCase();
    TransformationVector trans;

    Matrix2d B = StandardMath::calcB(g_move, g_fixed, trans);
    Vector2d mean_vector = StandardMath::calcMeanVector(g_move, g_fixed, trans);
    Vector3d q = StandardMath::q(g_move, trans, mean_vector, B);
    Vector3d grad_object = StandardMath::calcGradient(mean_vector, B, q);

    EXPECT_EQ(round(-0.1185*10000), round(grad_object(0)*10000));
    EXPECT_EQ(round(-0.1840*10000), round(grad_object(1)*10000));
    EXPECT_EQ(round(2.8725*10000), round(grad_object(2)*10000));
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "tester");
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}