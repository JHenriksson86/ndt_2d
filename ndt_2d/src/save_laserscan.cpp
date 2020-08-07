#include <ros/ros.h>
#include <ros/console.h>
#include <ndt_2d.h>

using namespace ndt2d;

LaserScan g_scan;
bool laser_scan_saved = false;

void laser_scan_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    ROS_INFO("Saving laser scan!");
    g_scan.addScanMessage(msg);
    g_scan.saveLaserScan("test");
    laser_scan_saved = true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ndt_2d_save_laserscan_node");
    ros::NodeHandle nh;

    ros::Subscriber laser_sub_ = nh.subscribe("/scan", 100, &laser_scan_callback);

    ros::Rate loop_rate(10);
    while(ros::ok() && !laser_scan_saved)
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    g_scan.loadLaserScan("test");
    ROS_INFO("Save successfull, shuting down.");

    return 0;
}