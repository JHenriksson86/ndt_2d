#include <ros/ros.h>
#include "ndt_2d_map.h"
#include <gtest/gtest.h>
#include <iostream>
#include <cmath>

using namespace ndt2d;
using Eigen::Vector2d;
using Eigen::Matrix2d;
using Eigen::Vector3d;
using Eigen::Matrix3d;
using std::round;
using std::cos;
using std::sin;


//Test map cells

TEST(MapCell, initClass)
{
  NDTCell cell;
  EXPECT_FALSE(cell.hasGaussian());
  EXPECT_FALSE(cell.calculateGaussian());
}

TEST(MapCell, testCalcGaussian)
{
  NDTCell cell;
  Vector2d p1(3.0,1.5), p2(2.0, 3.0), p3(1.0, -3.0);
  cell.addScanPoint(p1);
  cell.addScanPoint(p2);
  cell.addScanPoint(p3);

  EXPECT_EQ(3, cell.getUnregisteredSCPS());
  EXPECT_FALSE(cell.hasGaussian());
  EXPECT_TRUE(cell.calculateGaussian());
  EXPECT_EQ(3, cell.getRegisteredSCPS());
  EXPECT_EQ(0, cell.getUnregisteredSCPS());

  Vector2d mean =  cell.getMean();
  Matrix2d cov = cell.getCovariance();
  
  //Test mean
  EXPECT_EQ(2.0, mean[0]);
  EXPECT_EQ(0.5, mean[1]);

  //Test covariance
  EXPECT_EQ(1.0, cov(0,0));
  EXPECT_EQ(2.25, cov(0,1));
  EXPECT_EQ(2.25, cov(1,0));
  EXPECT_EQ(9.75, cov(1,1));

  EXPECT_TRUE(cell.hasGaussian());
}

TEST(MapCellOM, testTwoRoundsCalcGaussian)
{
  NDTOMCell cell;
  Vector2d p1(3.0,1.5), p2(2.0, 3.0), p3(1.0, -3.0), p4(2.0, 2.0), p5(1.0, 2.0), p6(0.0, -1.0);
  cell.addScanPoint(p1);
  cell.addScanPoint(p2);
  cell.addScanPoint(p3);

  EXPECT_TRUE(cell.calculateGaussian());
  EXPECT_EQ(3, cell.getRegisteredSCPS());

  cell.addScanPoint(p4);
  cell.addScanPoint(p5);
  cell.addScanPoint(p6);

  EXPECT_TRUE(cell.calculateGaussian());
  EXPECT_EQ(6, cell.getRegisteredSCPS());

  Vector2d mean =  cell.getMean();
  Matrix2d cov = cell.getCovariance();
  
  //Test mean
  EXPECT_EQ(1.5, mean[0]);
  EXPECT_EQ(0.75, mean[1]);

  //Test covariance
  EXPECT_EQ(1.1, cov(0,0));
  EXPECT_EQ(1.35, cov(0,1));
  EXPECT_EQ(1.35, cov(1,0));
  EXPECT_EQ(round(5.175*1000.0), round(cov(1,1)*1000.0));
}

//Test map

TEST(Map, testInit)
{
  NDTMap map;
  EXPECT_EQ(20, map.getRows());
  EXPECT_EQ(20, map.getColumns());
  EXPECT_EQ(1.0, map.getCellSize());
}

TEST(Map, testInitSize)
{
  NDTMap map1(12, 12, 1.0);
  EXPECT_EQ(12, map1.getRows());
  EXPECT_EQ(12, map1.getColumns());
  EXPECT_EQ(1.0, map1.getCellSize());

  NDTMap map2(80, 80, 0.25);
  EXPECT_EQ(80, map2.getRows());
  EXPECT_EQ(80, map2.getColumns());
  EXPECT_EQ(0.25, map2.getCellSize());

  NDTMap map3(12, 12, 2.0);
  EXPECT_EQ(12, map3.getRows());
  EXPECT_EQ(12, map3.getColumns());
  EXPECT_EQ(2.0, map3.getCellSize());
}

TEST(Map, testCellInit)
{
  NDTMap map;

  for(int row = 0; row < map.getRows(); row++)
  {
    for(int col = 0; col < map.getColumns(); col++)
    {
      EXPECT_EQ(0, map(row, col).getRegisteredSCPS());
      EXPECT_EQ(0, map(row, col).getUnregisteredSCPS());
      EXPECT_FALSE(map(row, col).hasGaussian());
    }
  }
}

TEST(Map, testCellInitBiggersize)
{
  NDTMap map(12, 12, 2.5);

  for(int row = 0; row < map.getRows(); row++)
  {
    for(int col = 0; col < map.getColumns(); col++)
    {
      EXPECT_EQ(0, map(row, col).getRegisteredSCPS());
      EXPECT_EQ(0, map(row, col).getUnregisteredSCPS());
      EXPECT_FALSE(map(row, col).hasGaussian());
    }
  }
}

TEST(Map, testScanReg)
{
  NDTMap map;

  EXPECT_TRUE(map.addScan(2.3,2.1));

  for(int row = 0; row < map.getRows(); row++)
  {
    for(int col = 0; col < map.getColumns(); col++)
    {
      if(row == 7 && col == 12)
        EXPECT_EQ(1, map(row, col).getUnregisteredSCPS()) << "Map cell (" << row << "," << col << ")\n";
      else
        EXPECT_EQ(0, map(row, col).getUnregisteredSCPS()) << "Map cell (" << row << "," << col << ")\n";
    }
  }
}

TEST(Map, testScanRegLimits)
{
  NDTMap map;

  EXPECT_FALSE(map.addScan(2.3,10.1));
  EXPECT_FALSE(map.addScan(10.3,2.1));
  EXPECT_FALSE(map.addScan(10.3,10.1));
  EXPECT_FALSE(map.addScan(-10.3,2.1));
  EXPECT_FALSE(map.addScan(2.0,-10.1));
  EXPECT_FALSE(map.addScan(-10.2,-10.1));

  EXPECT_TRUE(map.addScan(9.9,9.9));
  EXPECT_TRUE(map.addScan(9.9,-9.9));
  EXPECT_TRUE(map.addScan(-9.9,9.9));
  EXPECT_TRUE(map.addScan(-9.9,-9.9));

  for(int row = 0; row < map.getRows(); row++)
  {
    for(int col = 0; col < map.getColumns(); col++)
    {
      if(row == 0 && col == 19)
        EXPECT_EQ(1, map(row, col).getUnregisteredSCPS())  << "Map cell (" << row << "," << col << ")\n";
      else if(row == 19 && col == 19)
        EXPECT_EQ(1, map(row, col).getUnregisteredSCPS())  << "Map cell (" << row << "," << col << ")\n";
      else if(row == 0 && col == 0)
        EXPECT_EQ(1, map(row, col).getUnregisteredSCPS())  << "Map cell (" << row << "," << col << ")\n";
      else if(row == 19 && col == 0)
        EXPECT_EQ(1, map(row, col).getUnregisteredSCPS())  << "Map cell (" << row << "," << col << ")\n";
      else
        EXPECT_EQ(0, map(row, col).getUnregisteredSCPS())  << "Map cell (" << row << "," << col << ")\n";
    }
  }
}

TEST(Map, testScanRegCellSize2meters)
{
  NDTMap map(12, 12, 2.0);

  EXPECT_TRUE(map.addScan(9.9,9.9));
  EXPECT_TRUE(map.addScan(9.9,-9.9));
  EXPECT_TRUE(map.addScan(-9.9,9.9));
  EXPECT_TRUE(map.addScan(-9.9,-9.9));
  EXPECT_TRUE(map.addScan(2.3, 2.1));
  
  for(int row = 0; row < map.getRows(); row++)
  {
    for(int col = 0; col < map.getColumns(); col++)
    {
      if(row == 1 && col == 10)
        EXPECT_EQ(1, map(row, col).getUnregisteredSCPS())  << "Map cell (" << row << "," << col << ")\n";
      else if(row == 10 && col == 10)
        EXPECT_EQ(1, map(row, col).getUnregisteredSCPS())  << "Map cell (" << row << "," << col << ")\n";
      else if(row == 1 && col == 1)
        EXPECT_EQ(1, map(row, col).getUnregisteredSCPS())  << "Map cell (" << row << "," << col << ")\n";
      else if(row == 10 && col == 1)
        EXPECT_EQ(1, map(row, col).getUnregisteredSCPS())  << "Map cell (" << row << "," << col << ")\n";
      else if(row == 4 && col == 7)
        EXPECT_EQ(1, map(row, col).getUnregisteredSCPS())  << "Map cell (" << row << "," << col << ")\n";
      else
        EXPECT_EQ(0, map(row, col).getUnregisteredSCPS())  << "Map cell (" << row << "," << col << ")\n";
    }
  }
}

TEST(Map, testGaussian)
{
  NDTMap map;
  Vector2d p1(-9.9, 9.9), p2(-9.8, 9.4), p3(-9.1, 9.5), p4(-9.3, 9.3), p5(-9.7, 9.5), p6(-9.5, 9.1);

  EXPECT_TRUE(map.addScan(p1));
  EXPECT_TRUE(map.addScan(p2));
  EXPECT_TRUE(map.addScan(p3));
  EXPECT_TRUE(map.addScan(p4));
  EXPECT_TRUE(map.addScan(p5));
  EXPECT_TRUE(map.addScan(p6));

  EXPECT_EQ(6, map(0,0).getUnregisteredSCPS());
  EXPECT_EQ(0, map(0,0).getRegisteredSCPS());
  EXPECT_TRUE(map(0,0).calculateGaussian());
  EXPECT_EQ(0, map(0,0).getUnregisteredSCPS());
  EXPECT_EQ(6, map(0,0).getRegisteredSCPS());

  Vector2d mean =  map(0,0).getMean();
  Matrix2d cov = map(0,0).getCovariance();

  //Test mean
  EXPECT_EQ(round(-9.55*1000), round(mean[0]*1000));
  EXPECT_EQ(round(9.45*1000), round(mean[1]*1000));

  //Test covariance
  EXPECT_EQ(round(0.095*1000), round(cov(0,0)*1000));
  EXPECT_EQ(round(-0.037*1000), round(cov(0,1)*1000));
  EXPECT_EQ(round(-0.037*1000), round(cov(1,0)*1000));
  EXPECT_EQ(round(0.071*1000), round(cov(1,1)*1000));
}

TEST(Map, testGaussianTransformation)
{
  NDTMap map;
  Vector2d p1(-9.9, 9.9), p2(-9.8, 9.4), p3(-9.1, 9.5), p4(-9.3, 9.3), p5(-9.7, 9.5), p6(-9.5, 9.1);

  EXPECT_TRUE(map.addScan(p1));
  EXPECT_TRUE(map.addScan(p2));
  EXPECT_TRUE(map.addScan(p3));
  EXPECT_TRUE(map.addScan(p4));
  EXPECT_TRUE(map.addScan(p5));
  EXPECT_TRUE(map.addScan(p6));

  EXPECT_EQ(6, map(0,0).getUnregisteredSCPS());
  EXPECT_EQ(0, map(0,0).getRegisteredSCPS());
  EXPECT_TRUE(map(0,0).calculateGaussian());
  EXPECT_EQ(0, map(0,0).getUnregisteredSCPS());
  EXPECT_EQ(6, map(0,0).getRegisteredSCPS());

  // TODO
}

//Test transformation vector

TEST(Transformation_Vector, init)
{
  TransformationVector vec1;
  TransformationVector vec2(1.1, 2.1, 0.4);
  TransformationVector vec3(Vector3d(-1.1, -2.1, -0.4));

  EXPECT_EQ(0.0, vec1.getX());
  EXPECT_EQ(0.0, vec1.getY());
  EXPECT_EQ(0.0, vec1.getRotation());

  EXPECT_EQ(1.1, vec2.getX());
  EXPECT_EQ(2.1, vec2.getY());
  EXPECT_EQ(0.4, vec2.getRotation());

  EXPECT_EQ(-1.1, vec3.getX());
  EXPECT_EQ(-2.1, vec3.getY());
  EXPECT_EQ(-0.4, vec3.getRotation());
}

TEST(Transformation_Vector, translation_vector)
{
  TransformationVector vec1(1.1, 2.1, 0.4);
  TransformationVector vec2(Vector3d(-1.1, -2.1, -0.4));

  Vector2d translation1(1.1, 2.1);
  Vector2d translation2(-1.1, -2.1);

  EXPECT_EQ(translation1, vec1.getTranslationVector());
  EXPECT_EQ(translation2, vec2.getTranslationVector());
}

TEST(Transformation_Vector, rotation_matrix)
{
  TransformationVector vec1(1.1, 2.1, 0.4);
  TransformationVector vec2(Vector3d(-1.1, -2.1, -0.4));

  Matrix2d rot1;
  rot1 << cos(0.4), -sin(0.4), sin(0.4), cos(0.4);
  Matrix2d rot2;
  rot2 << cos(-0.4), -sin(-0.4), sin(-0.4), cos(-0.4);

  EXPECT_EQ(rot1, vec1.getRotationMatrix());
  EXPECT_EQ(rot2, vec2.getRotationMatrix());
}

TEST(Transformation_Vector, operator_plus)
{
  TransformationVector vec1(1.1, 2.1, 0.4);
  TransformationVector vec2(Vector3d(-1.1, -2.1, -0.4));

  TransformationVector vec3 = vec1 + vec2;

  EXPECT_EQ(0.0, vec3.getX());
  EXPECT_EQ(0.0, vec3.getY());
  EXPECT_EQ(0.0, vec3.getRotation());
}

TEST(Transformation_Vector, operator_minus)
{
  TransformationVector vec1(1.1, 2.1, 0.4);
  TransformationVector vec2(Vector3d(-1.1, -2.1, -0.4));

  TransformationVector vec3 = vec1 - vec2;

  EXPECT_EQ(2.2, vec3.getX());
  EXPECT_EQ(4.2, vec3.getY());
  EXPECT_EQ(0.8, vec3.getRotation());
}

TEST(Transformation_Vector, get_array)
{
  TransformationVector vec1(1.1, 2.1, 0.4);

  double param[3];
  vec1.getParameterArray(param);

  EXPECT_EQ(1.1, param[0]);
  EXPECT_EQ(2.1, param[1]);
  EXPECT_EQ(0.4, param[2]);
}

TEST(Transformation_Vector, set_array)
{
  TransformationVector vec1(1.1, 2.1, 0.4);
  TransformationVector vec2;

  double param[3];
  vec1.getParameterArray(param);
  vec2.setTransformationParameters(param);

  EXPECT_EQ(1.1, vec2.getX());
  EXPECT_EQ(2.1, vec2.getY());
  EXPECT_EQ(0.4, vec2.getRotation());
}

//Test gaussian
TEST(gaussian_class, init)
{
  Gaussian gau1;

  Vector2d mean2(1.5, 2.0);
  Matrix2d cov2;
  cov2 << 1.0, 0.1, 0.1, 0.9;
  Gaussian gau2(mean2, cov2);

  EXPECT_EQ(Vector2d::Zero(), gau1.getMean());
  EXPECT_EQ(Matrix2d::Zero(), gau1.getCovariance());
  EXPECT_EQ(mean2, gau2.getMean());
  EXPECT_EQ(cov2, gau2.getCovariance());
}

TEST(gaussian_class, copy)
{
  Vector2d mean(1.5, 2.0);
  Matrix2d cov;
  cov << 1.0, 0.1, 0.1, 0.9;
  Gaussian gau1(mean, cov);

  Gaussian gau2 = gau1;

  EXPECT_EQ(mean, gau1.getMean());
  EXPECT_EQ(cov, gau1.getCovariance());
  EXPECT_EQ(mean, gau2.getMean());
  EXPECT_EQ(cov, gau2.getCovariance());
}

TEST(gaussian_class, translation_transformation)
{
  Vector2d mean(1.5, 2.0);
  Matrix2d cov;
  cov << 1.0, 0.1, 0.1, 0.9;
  Gaussian gau1(mean, cov);

  TransformationVector trans1(0.1, 1.0, 0.0);
  Gaussian gau2 = gau1.getTransformed(trans1);
  Vector2d mean_result2(1.6, 3.0);

  TransformationVector trans2(-0.1, -1.0, 0.0);
  gau1.transform(trans2);
  Vector2d mean_result1(1.4, 1.0);

  EXPECT_EQ(mean_result1, gau1.getMean());
  EXPECT_EQ(cov, gau1.getCovariance());
  EXPECT_EQ(mean_result2, gau2.getMean());
  EXPECT_EQ(cov, gau2.getCovariance());
}

TEST(gaussian_class, rotation_transformation)
{
  Vector2d mean(1.5, 2.0);
  Matrix2d cov;
  cov << 1.0, 0.1, 0.1, 0.9;
  Gaussian gau1(mean, cov);

  TransformationVector trans2(0.1, 1.0, 2.0);
  Gaussian gau2 = gau1.getTransformed(trans2);
  Vector2d mean_result2(-2.3428, 1.5317);
  Matrix2d cov_result2;
  cov_result2 << 0.8416, -0.0275, -0.0275, 1.0584;

  TransformationVector trans1(-0.1, -1.0, -2.0);
  gau1.transform(trans1);  
  Vector2d mean_result1(1.0944, -3.1962);
  Matrix2d cov_result1;
  cov_result1 << 0.9930, -0.1032, -0.1032, 0.9070;
  
  EXPECT_EQ(round(mean_result1[0]*10000.0), round((gau1.getMean())[0]*10000.0)); 
  EXPECT_EQ(round(mean_result1[1]*10000.0), round((gau1.getMean())[1]*10000.0));

  EXPECT_EQ(round(mean_result2[0]*10000.0), round((gau2.getMean())[0]*10000.0)); 
  EXPECT_EQ(round(mean_result2[1]*10000.0), round((gau2.getMean())[1]*10000.0));
  
  EXPECT_EQ(round(cov_result1(0,0)*10000.0), round((gau1.getCovariance())(0,0)*10000.0)); 
  EXPECT_EQ(round(cov_result1(0,1)*10000.0), round((gau1.getCovariance())(0,1)*10000.0));
  EXPECT_EQ(round(cov_result1(1,0)*10000.0), round((gau1.getCovariance())(1,0)*10000.0)); 
  EXPECT_EQ(round(cov_result1(1,1)*10000.0), round((gau1.getCovariance())(1,1)*10000.0));

  EXPECT_EQ(round(cov_result2(0,0)*10000.0), round((gau2.getCovariance())(0,0)*10000.0)); 
  EXPECT_EQ(round(cov_result2(0,1)*10000.0), round((gau2.getCovariance())(0,1)*10000.0));
  EXPECT_EQ(round(cov_result2(1,0)*10000.0), round((gau2.getCovariance())(1,0)*10000.0)); 
  EXPECT_EQ(round(cov_result2(1,1)*10000.0), round((gau2.getCovariance())(1,1)*10000.0));

}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "tester");
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}