#include <ros/ros.h>
#include <ros/console.h>
#include <ndt_2d_service.h>

using namespace ndt2d;

int main(int argc, char **argv)
{
    // Init the connection with the ROS system.
   ros::init(argc, argv, "ndt_service_node");
   ros::NodeHandle nh;

   ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
   ros::console::notifyLoggerLevelsChanged();
   
   // Creating class instance
   NDT2dService ndt;
   ndt.subscribeAndAdvertise(&nh);

   // ROS loop
   ros::spin();

   return 0;
}

