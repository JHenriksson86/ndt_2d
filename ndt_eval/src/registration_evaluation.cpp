#include "ndt_eval/registration_evaluation.h"

using std::string;

namespace ndt2d
{
  RegistrationEvaluation::RegistrationEvaluation(ros::NodeHandle* nh)
  {
    this->nh_ = nh;
    this->pcd_path_ = "";
    this->pcd_name_ = "";
    this->odometry_file_ = "";
    this->output_file_ = "";
    this->pcd_start_ = 0;
    this->pcd_stop_ = 0;
    this->pcd_step_size_ = 1;
    this->initial_guess_ = false;
    this->fixed_cloud_ = new PointCloudHandler();
    this->movable_cloud_ = new PointCloudHandler();
    this->registration_client_ = nh->serviceClient<ndt_msgs::Registration>("registration");
    this->fixed_cloud_pub_ = nh_->advertise<sensor_msgs::PointCloud2>("fixed_cloud", 10);
    this->movable_cloud_pub_ = nh_->advertise<sensor_msgs::PointCloud2>("movable_cloud", 10);
    this->result_cloud_pub_ = nh_->advertise<sensor_msgs::PointCloud2>("result_cloud", 10);
    getRegistrationParameters();
    getPCDParameters();
    this->robot_pose_ = TransformHandler(getScannerTransform());

    // Create or wipe file
    std::ofstream ofs;
    ofs.open (output_file_, std::ofstream::out | std::ofstream::trunc);
    ofs.close();
  }

  RegistrationEvaluation::~RegistrationEvaluation()
  {
    delete fixed_cloud_;
    delete movable_cloud_;
  }

  void RegistrationEvaluation::run()
  {
    ndt_msgs::Registration srv;
    srv.request.parameters = registration_parameters_;
    
    string file = pcd_path_ + pcd_name_;
    robot_pose_.loadOdometry(odometry_file_,pcd_start_);
    robot_pose_.broadcastTransform();
    movable_cloud_->loadPCD(file + std::to_string(pcd_start_) + ".pcd");
    movable_cloud_->setStamp(robot_pose_.getStamp());

    for(int i = pcd_start_+pcd_step_size_; i <= pcd_stop_; i += pcd_step_size_)
    {
      delete fixed_cloud_;
      fixed_cloud_ = movable_cloud_;
      robot_pose_.loadOdometry(odometry_file_, i);
      movable_cloud_ = new PointCloudHandler();
      movable_cloud_->loadPCD(file + std::to_string(i) + ".pcd");
      movable_cloud_->setStamp(robot_pose_.getStamp());
      fixed_cloud_->setStamp(robot_pose_.getStamp());

      geometry_msgs::Transform initial_guess;
      if(initial_guess_)
        initial_guess = robot_pose_.getInitialGuessMsg();
      else
        initial_guess = robot_pose_.getInitialGuessMsg(0.0,0.0,0.0,0.0,0.0,0.0);
      
      geometry_msgs::Transform result = callRegistrationService(
        srv, fixed_cloud_, movable_cloud_, initial_guess
      );

      robot_pose_.addRegistrationResult(result);
      ROS_INFO_STREAM("Accumulated transformation:\n" << robot_pose_.to_string());
      robot_pose_.broadcastTransform();

      saveEstimate();

      fixed_cloud_pub_.publish(fixed_cloud_->getPointCloud2Message());
      movable_cloud_pub_.publish(movable_cloud_->getPointCloud2Message());
      result_cloud_pub_.publish(movable_cloud_->getPointCloud2Message(robot_pose_.getTransformMsg()));
    } 
  }

  geometry_msgs::Transform RegistrationEvaluation::callRegistrationService(
    ndt_msgs::Registration& srv, PointCloudHandler* fixed_cloud, 
    PointCloudHandler* movable_cloud, const geometry_msgs::Transform& initial_guess)
  {
    sensor_msgs::PointCloud2 fixed_cloud_msg = fixed_cloud_->getPointCloud2Message();
    sensor_msgs::PointCloud2 movable_cloud_msg = movable_cloud_->getPointCloud2Message();
    srv.request.fixed_cloud = fixed_cloud_msg;
    srv.request.movable_cloud = movable_cloud_msg;
    srv.request.initial_guess = initial_guess;
    ROS_INFO_STREAM("Initial guess:\n" << initial_guess);
    
    registration_client_.waitForExistence();
    geometry_msgs::Transform result;
    if(registration_client_.call(srv))
    {
      ROS_INFO("Service call was successful!");
      result = srv.response.result;
      ROS_INFO("Result");
      ROS_INFO("Translation: [%.2f,%.2f,%.2f]", 
        result.translation.x, result.translation.y, result.translation.z);
      ROS_INFO("Rotation: [%.4f,%.4f,%.4f,%.4f]", 
        result.rotation.x, result.rotation.y, 
        result.rotation.z, result.rotation.w);
    }
    else
    {
      ROS_ERROR("Service call failed!");
    }
    return result;
  }

  void RegistrationEvaluation::saveEstimate() const
  {
    std::ofstream output;
    ROS_DEBUG("Writing data to %s", output_file_.c_str());
    output.open(output_file_.c_str(),  std::ofstream::out | std::ofstream::app);

    if(output.is_open())
    {
      output <<  robot_pose_.to_string() << "\n";
    }
    else
    {
      ROS_ERROR("Failed to open output stream.");
    }

    output.close();
  }

  void RegistrationEvaluation::getPCDParameters()
  {
    ros::param::get("ndt_eval_node/pcd_path", pcd_path_);
    ROS_INFO_STREAM("Pcd path is set to: " << pcd_path_);

    ros::param::get("ndt_eval_node/pcd_name", pcd_name_);
    ROS_INFO_STREAM("Pcd name is set to: " << pcd_name_);

    ros::param::get("ndt_eval_node/pcd_start_number", pcd_start_);
    ROS_INFO_STREAM("Pcd start number is set to: " << pcd_start_);

    ros::param::get("ndt_eval_node/pcd_stop_number", pcd_stop_);
    ROS_INFO_STREAM("Pcd stop number is set to: " << pcd_stop_);

    ros::param::get("ndt_eval_node/pcd_step_size", pcd_step_size_);
    ROS_INFO_STREAM("Pcd stop number is set to: " << pcd_step_size_);

    ros::param::get("ndt_eval_node/odometry_file", odometry_file_);
    ROS_INFO_STREAM("Odometry file is set to: " << odometry_file_);

    ros::param::get("ndt_eval_node/output_file", output_file_);
    ROS_INFO_STREAM("Output file is set to: " << output_file_);

    ros::param::get("ndt_eval_node/initial_guess", initial_guess_);
    ROS_INFO_STREAM("Initial guess is set to: " << initial_guess_);

  }

  void RegistrationEvaluation::getRegistrationParameters()
  {
    double map_cell_size = 1.0;
    ros::param::get("ndt_eval_node/map_cell_size", map_cell_size);
    ROS_INFO_STREAM("Map cell size: " << map_cell_size);
    registration_parameters_.map_cell_size = map_cell_size;

    int map_cell_count = 80;
    ros::param::get("ndt_eval_node/map_cell_count", map_cell_count);
    ROS_INFO_STREAM("Number of map cells: " << map_cell_count);
    registration_parameters_.map_cell_count = map_cell_count;

  }

  Eigen::Affine3d RegistrationEvaluation::getScannerTransform() const
  {
    Eigen::Vector3d translation = Eigen::Vector3d::Zero();
    ros::param::get("ndt_eval_node/scanner_posx", translation[0]);
    ros::param::get("ndt_eval_node/scanner_posy", translation[1]);
    ros::param::get("ndt_eval_node/scanner_posz", translation[2]);
    ROS_INFO("Scanner translation set to: [%.2f,%.2f,%.2f]", 
      translation[0], translation[1], translation[2]);

    double rotx = 0.0, roty = 0.0, rotz = 0.0;
    ros::param::get("ndt_eval_node/scanner_rotx", rotx);
    ros::param::get("ndt_eval_node/scanner_roty", roty);
    ros::param::get("ndt_eval_node/scanner_rotz", rotz);
    ROS_INFO("Scanner translation set to: [%.2f,%.2f,%.2f]", 
      rotx, roty, rotz);

    Eigen::Matrix3d rot_mat;
    rot_mat = Eigen::AngleAxisd(rotz, Eigen::Vector3d::UnitZ())
      * Eigen::AngleAxisd(roty, Eigen::Vector3d::UnitY())
      * Eigen::AngleAxisd(rotx, Eigen::Vector3d::UnitZ());

    Eigen::Affine3d robot_to_laser = Eigen::Affine3d(rot_mat);
    robot_to_laser.translation() = translation;

    return robot_to_laser;
  }
};