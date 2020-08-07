#include <ros/ros.h>
#include "ndt_eval/registration_evaluation.h"

using namespace ndt2d;

int main(int argc, char *argv[])
{
  // Init the connection with the ROS system.
  ros::init(argc, argv, "ndt_eval_node");
  ros::NodeHandle nh;

  // Setting debugging
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
  ros::console::notifyLoggerLevelsChanged();

  // Wait for other applications to start up
  ros::Duration(1.0).sleep();

  RegistrationEvaluation reg(&nh);
  
  double start_time = ros::Time::now().toSec();
  reg.run();
  double stop_time = ros::Time::now().toSec();

  ROS_INFO("Time taken %2.f seconds.", stop_time - start_time);
  
  return 0;
}