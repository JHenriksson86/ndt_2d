#include "ndt_eval/pointcloud_handler.h"

namespace ndt2d
{
  PointCloudHandler::PointCloudHandler(std::string frame) :
    cloud_(new pcl::PointCloud<pcl::PointXYZ>)
  {
    this->frame_ = frame;
    this->stamp_ = "";
  }

  PointCloudHandler::PointCloudHandler(
      const sensor_msgs::PointCloud2& pc2_message, 
      std::string frame) : 
    cloud_(new pcl::PointCloud<pcl::PointXYZ>)
  {
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(pc2_message, pcl_pc2);
    pcl::fromPCLPointCloud2(pcl_pc2, *cloud_);

    this->frame_ = frame;
    this->stamp_ = "";
  }  

  PointCloudHandler::~PointCloudHandler(){}

  int PointCloudHandler::getNumberOfPoints() const
  {
    return cloud_->size();
  }

  void PointCloudHandler::loadPCD(std::string pcd_path)
  {
    cloud_->clear();
    pcl::PCDReader reader;
    ROS_DEBUG_STREAM("Loading pcd from: " << pcd_path);
    if (reader.read(pcd_path, *cloud_) < 0)
    {
      PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    }
  }

  void PointCloudHandler::setStamp(const std::string& stamp)
  {
    this->stamp_ = stamp;
  }

  const std::string& PointCloudHandler::getStamp() const
  {
    return stamp_;
  }

  sensor_msgs::PointCloud2 PointCloudHandler::getPointCloud2Message(
    bool reduced, double grid_size) const
  {
    ROS_DEBUG("PointCloudHandler::getPointCloud2Message(reduced = %d, grid_size = %.2f)",
      reduced, grid_size);
    if(!reduced)
    {
      return pclToPC2(*cloud_);
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    downsampleCloud(grid_size, cloud_, *filtered_cloud);

    return pclToPC2(*filtered_cloud);
  }

  sensor_msgs::PointCloud2 PointCloudHandler::getPointCloud2Message(
    const geometry_msgs::Transform& transform, bool reduced, double grid_size) const
  {
    ROS_DEBUG("PointCloudHandler::getPointCloud2Message(const geometry_msgs::Transform& transform)");
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    if(reduced)
    {
      cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
      downsampleCloud(grid_size, cloud_, *cloud);
    }
    else
    {
      cloud = cloud_;
    }
    
    Eigen::Affine3d transformation = Eigen::Affine3d::Identity();
    TransformHandler::transformMessageToEigenTransform(transform, transformation);
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*cloud, *transformed_cloud, transformation);

    ROS_DEBUG_STREAM("Transforming point cloud with translation:\n" 
      << transformation.translation() << "\nrotation:\n" 
      << transformation.rotation());

    return pclToPC2(*transformed_cloud);
  }

  Eigen::MatrixXd PointCloudHandler::getPointMatrix(bool reduced, double grid_size) const
  {
    if(!reduced)
    {
      return cloud_->getMatrixXfMap(3,4,0).cast<double>();
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    downsampleCloud(grid_size, cloud_, *filtered_cloud);
    
    return filtered_cloud->getMatrixXfMap(3,4,0).cast<double>();
  }

  sensor_msgs::PointCloud2 PointCloudHandler::pclToPC2(const PointCloud& pcl_cloud) const
  {
    sensor_msgs::PointCloud2 ros_cloud;
    pcl::toROSMsg(pcl_cloud, ros_cloud);
    ros_cloud.header.frame_id = frame_;

    ros::Time time_stamp;
    if(stamp_.compare("") != 0)
      time_stamp.fromSec(std::stod(stamp_));
    else 
      time_stamp.fromSec(0.0);
    ros_cloud.header.stamp = time_stamp;
    return ros_cloud;
  }

  void PointCloudHandler::downsampleCloud(double grid_size,PointCloud::Ptr input_cloud, PointCloud& output_cloud) const
  {
    pcl::VoxelGrid<pcl::PointXYZ> filter;
    filter.setInputCloud(input_cloud);
    filter.setLeafSize(grid_size, grid_size, grid_size);
    filter.filter(output_cloud);
  }
};