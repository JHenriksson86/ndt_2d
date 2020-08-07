#include <ros/ros.h>
#include "ndt_2d_scan.h"
#include "ndt_2d_util.h"
#include <gtest/gtest.h>
#include <iostream>
#include <cmath>

using namespace ndt2d;
using std::round;
using std::sqrt;
using std::cos;
using std::sin;

TEST(LaserScanPoint, initClass)
{
  LaserScanPoint point;
  LaserScanPoint point1(2.9, M_PI_2);
  LaserScanPoint point2(point1);

  EXPECT_EQ(0.0, point.getDistance());
  EXPECT_EQ(0.0, point.getAngle());
  EXPECT_EQ(2.9, point1.getDistance());
  EXPECT_EQ(M_PI_2, point1.getAngle());
  EXPECT_EQ(point1.getDistance(), point2.getDistance());
  EXPECT_EQ(point1.getAngle(), point2.getAngle());
}

TEST(LaserScanPoint, testConversion)
{
  LaserScanPoint point1;
  LaserScanPoint point2;

  point1.setCartesianCoordinates(1.0, 1.0);
  point2.setPolarCoordinates(sqrt(2.0), M_PI_4);

  EXPECT_DOUBLE_EQ(point1.getX(), point2.getX());
  EXPECT_DOUBLE_EQ(point1.getY(), point2.getY());
  EXPECT_DOUBLE_EQ(point1.getDistance(), point2.getDistance());
  EXPECT_DOUBLE_EQ(point1.getAngle(), point2.getAngle());
}

TEST(LaserScanPoint, testAngleTransformation)
{
  LaserScanPoint point1;
  LaserScanPoint point2;
  LaserScanPoint point3;
  TransformationVector trans1(0.0, 0.0, M_PI);

  point1.setCartesianCoordinates(sqrt(2.0), 0.0);
  point2 = point1.getTransformed(trans1);
  point3 = point2;
  point3.transform(trans1);

  EXPECT_DOUBLE_EQ(sqrt(2.0), point2.getDistance());
  EXPECT_DOUBLE_EQ(M_PI, point2.getAngle());
  EXPECT_DOUBLE_EQ(sqrt(2.0), point3.getDistance());
  EXPECT_DOUBLE_EQ(0.0, round(point3.getAngle()*10000.0));
}

TEST(LaserScanPoint, testTranslationTransformation)
{
  LaserScanPoint point1;
  LaserScanPoint point2;
  LaserScanPoint point3;
  TransformationVector trans1(1.0, 1.0, 0.0);
  TransformationVector trans2(-10.0, 3.0, 0.0);

  point1.setCartesianCoordinates(2.0, 0.0);
  point2 = point1.getTransformed(trans1);
  point3 = point1;
  point3.transform(trans2);

  EXPECT_DOUBLE_EQ(2.0, point1.getX());
  EXPECT_DOUBLE_EQ(0.0, point1.getY());
  EXPECT_DOUBLE_EQ(3.0, point2.getX());
  EXPECT_DOUBLE_EQ(1.0, point2.getY());
  EXPECT_DOUBLE_EQ(-8.0, point3.getX());
  EXPECT_DOUBLE_EQ(3.0, point3.getY());
}

TEST(LaserScanPoint, testTransformation)
{
  LaserScanPoint point1;
  LaserScanPoint point2;
  LaserScanPoint point3;
  TransformationVector trans1(1.0, 0.0, M_PI_2);
  TransformationVector trans2(1.0, 0.0, M_PI);

  point1.setCartesianCoordinates(2.0, 2.0);
  point2 = point1.getTransformed(trans1);
  point3 = point1;
  point3.transform(trans2);

  EXPECT_DOUBLE_EQ(2.0, point1.getX());
  EXPECT_DOUBLE_EQ(2.0, point1.getY());
  EXPECT_DOUBLE_EQ(-1.0, point2.getX());
  EXPECT_DOUBLE_EQ(2.0, point2.getY());
  EXPECT_DOUBLE_EQ(-1.0, point3.getX());
  EXPECT_DOUBLE_EQ(-2.0, point3.getY());
}

TEST(LaserScan, testPushBack)
{
  LaserScanPoint point1(1.5, M_PI);
  LaserScanPoint point2(2.5, 0.0);
  LaserScanPoint point3(5.0, M_PI_2);

  LaserScan scan;
  scan.push_back(point1);
  scan.push_back(point2);
  scan.push_back(point3);

  
  EXPECT_DOUBLE_EQ(-1.5, scan[0].getX());
  EXPECT_DOUBLE_EQ(0.0, round(scan[0].getY()*10000.0));
  EXPECT_DOUBLE_EQ(2.5, scan[1].getX());
  EXPECT_DOUBLE_EQ(0.0, scan[1].getY());
  EXPECT_DOUBLE_EQ(0.0, round(scan[2].getX()*10000.0));
  EXPECT_DOUBLE_EQ(5.0, scan[2].getY());
}

TEST(LaserScan, testTransform)
{
  LaserScanPoint point1(1.5, M_PI);
  LaserScanPoint point2(2.5, 0.0);
  LaserScanPoint point3(5.0, M_PI_2);

  LaserScan scan;
  scan.push_back(point1);
  scan.push_back(point2);
  scan.push_back(point3);

  TransformationVector trans1(1.0, 1.0, M_PI);
  scan.transform(trans1);
  
  EXPECT_DOUBLE_EQ(2.5, scan[0].getX());
  EXPECT_DOUBLE_EQ(1.0, scan[0].getY());
  EXPECT_DOUBLE_EQ(-1.5, scan[1].getX());
  EXPECT_DOUBLE_EQ(1.0, scan[1].getY());
  EXPECT_DOUBLE_EQ(round(1.0*10000.0), round(scan[2].getX()*10000.0));
  EXPECT_DOUBLE_EQ(-4.0, scan[2].getY());
}

TEST(LaserScan, testgetTransform)
{
  LaserScanPoint point1(1.5, M_PI);
  LaserScanPoint point2(2.5, 0.0);
  LaserScanPoint point3(5.0, M_PI_2);

  LaserScan scan;
  scan.push_back(point1);
  scan.push_back(point2);
  scan.push_back(point3);

  TransformationVector trans1(1.0, 1.0, M_PI);
  LaserScan scan2 = scan.getTransformed(trans1);
  
  EXPECT_DOUBLE_EQ(2.5, scan2[0].getX());
  EXPECT_DOUBLE_EQ(1.0, scan2[0].getY());
  EXPECT_DOUBLE_EQ(-1.5, scan2[1].getX());
  EXPECT_DOUBLE_EQ(1.0, scan2[1].getY());
  EXPECT_DOUBLE_EQ(round(1.0*10000.0), round(scan2[2].getX()*10000.0));
  EXPECT_DOUBLE_EQ(-4.0, scan2[2].getY());
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "tester");
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}