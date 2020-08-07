#include "ndt_eval/transform_handler.h"

namespace ndt2d
{
  TransformHandler::TransformHandler()
  {
    init();
  }

  TransformHandler::TransformHandler(const Eigen::Affine3d& robot_to_laser)
  { 
    init();
    this->robot_to_laser_ = robot_to_laser;
  }

  TransformHandler::TransformHandler(const TransformHandler& transform)
  {
    this->registration_transform_ = transform.registration_transform_;
    this->robot_pose_ = transform.robot_pose_;
    this->robot_to_laser_ = transform.robot_to_laser_;
    this->odometry_ = transform.odometry_;
    this->old_odometry_ = transform.old_odometry_;
    this->stamp_ = transform.stamp_;
  }

  TransformHandler::~TransformHandler(){}

  void TransformHandler::setStamp(const std::string& stamp){ this->stamp_ = stamp; }

  const std::string& TransformHandler::getStamp() const { return stamp_; }

  void TransformHandler::setRobotToLaserTransform(const Eigen::Affine3d& robot_to_laser)
  {
    this->robot_to_laser_ = robot_to_laser;
  }

  const Eigen::Affine3d& TransformHandler::getRobotToLaserTransform() const
  {
    return this->robot_to_laser_;
  }

  void TransformHandler::setRobotPose(const Eigen::Affine3d& pose)
  {
    this->robot_pose_ = pose;
  }

  const Eigen::Affine3d& TransformHandler::getRobotPose() const
  {
    return this->robot_pose_;
  }

  void TransformHandler::addRegistrationResult(const geometry_msgs::Transform& transform)
  {
    registration_transform_ = Eigen::Affine3d::Identity();
    transformMessageToEigenTransform(transform, registration_transform_);
    Eigen::Affine3d robot_transform = robot_to_laser_ * registration_transform_ * robot_to_laser_.inverse();

    ROS_DEBUG_STREAM("addRegistrationResult:\n" << robot_transform.matrix());

    robot_pose_ = robot_pose_ * robot_transform;
    global_sensor_pose_ = global_sensor_pose_ * registration_transform_;
  }

  geometry_msgs::Transform TransformHandler::getInitialGuessMsg(
    double x, double y, double z, double roll, double pitch, double yaw)
  {
    geometry_msgs::Transform initial_guess;
    initial_guess.translation.x = x;
    initial_guess.translation.y = y;
    initial_guess.translation.z = z;

    tf2::Quaternion initial_rotation;
    initial_rotation.setRPY(roll, pitch, yaw);
    tf2::convert(initial_rotation, initial_guess.rotation);

    return initial_guess;
  }

  geometry_msgs::Transform TransformHandler::getInitialGuessMsg() const
  {
    Eigen::Affine3d ig = old_odometry_.inverse() * odometry_;
    ig = robot_to_laser_.inverse() * ig * robot_to_laser_;
    geometry_msgs::Transform msg;
    eigenTransformToTransformMessage(ig, msg);

    ROS_DEBUG_STREAM("Initial guess message:\n" << msg);
    return msg;
  }

  geometry_msgs::Transform TransformHandler::getTransformMsg() const
  {
    Eigen::Vector3d trans_position(registration_transform_.translation());
    Eigen::Quaterniond trans_orientation(registration_transform_.rotation());

    geometry_msgs::Transform msg;
    msg.translation.x = trans_position[0];
    msg.translation.y = trans_position[1];
    msg.translation.z = trans_position[2];
    msg.rotation.x = trans_orientation.x();
    msg.rotation.y = trans_orientation.y();
    msg.rotation.z = trans_orientation.z();
    msg.rotation.w = trans_orientation.w();
    ROS_DEBUG_STREAM("Transform message:\n" << msg);
    return msg;
  }

  geometry_msgs::Transform TransformHandler::getGlobalSensorPoseTransformMsg() const
  {
    Eigen::Vector3d trans_position(global_sensor_pose_.translation());
    Eigen::Quaterniond trans_orientation(global_sensor_pose_.rotation());

    geometry_msgs::Transform msg;
    msg.translation.x = trans_position[0];
    msg.translation.y = trans_position[1];
    msg.translation.z = trans_position[2];
    msg.rotation.x = trans_orientation.x();
    msg.rotation.y = trans_orientation.y();
    msg.rotation.z = trans_orientation.z();
    msg.rotation.w = trans_orientation.w();
    ROS_DEBUG_STREAM("Transform message:\n" << msg);
    return msg;
  }
  
  std::string TransformHandler::to_string() const
  {
    Eigen::Vector3d position = robot_pose_.translation();
    Eigen::Quaterniond orientation(robot_pose_.rotation());

    std::stringstream stream;
    stream.precision(15);
    stream << stamp_ << " " << position[0] << " " << 
      position[1] << " "<< position[2] << " "
      << orientation.x() << " " << orientation.y() 
      << " " << orientation.z() << " " << orientation.w();
    return stream.str();
  }

  void TransformHandler::loadStamp(const std::string& file_path, int row)
  {
    std::ifstream input_file;
    input_file.open(file_path, std::ifstream::in);

    if(input_file.is_open())
    {
      gotoLine(input_file, row);
      std::getline(input_file, stamp_, ' ');
      ROS_DEBUG("Stamp retrived: %s", stamp_.c_str());
    }
    else
    {
      ROS_ERROR("Failed to open file: %s", file_path.c_str());
    }
    input_file.close();
  }

  void TransformHandler::loadOdometry(const std::string& file_path, int row)
  {
    std::ifstream input_file;
    input_file.open(file_path, std::ifstream::in);
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
    if(input_file.is_open())
    {
      if(row > 0)
        gotoLine(input_file, row);

      std::getline(input_file, stamp_, ' ');
      std::string input_str = "";
      std::string::size_type accumulator_sz, size;
      std::getline(input_file, input_str);
      position[0] = std::stod(input_str, &size);
      accumulator_sz = size;
      position[1] = std::stod(input_str.substr(accumulator_sz), &size);
      accumulator_sz += size;
      position[2] = std::stod(input_str.substr(accumulator_sz), &size);
      accumulator_sz += size;
      orientation.x() = std::stod(input_str.substr(accumulator_sz), &size);
      accumulator_sz += size;
      orientation.y() = std::stod(input_str.substr(accumulator_sz), &size); 
      accumulator_sz += size;
      orientation.z() = std::stod(input_str.substr(accumulator_sz), &size); 
      accumulator_sz += size;
      orientation.w() = std::stod(input_str.substr(accumulator_sz), &size); 
      
      old_odometry_ = odometry_;
      odometry_ = Eigen::Affine3d(orientation);
      odometry_.translation() = position;
      ROS_INFO_STREAM("Odometry loaded from " << file_path << ":" << row);
    }
    else
    {
      ROS_ERROR("Failed to open file: %s", file_path.c_str());
    }
    input_file.close();
  }

  void TransformHandler::broadcastTransform()
  {
    geometry_msgs::TransformStamped tfStamped = 
      getStampedTransform(robot_pose_, "world", "base_link");
    broadcaster_.sendTransform(tfStamped);

    ROS_DEBUG_STREAM("broadcastTransform()\n" << tfStamped);
    
    broadcastStaticTransform();
  }

  void TransformHandler::broadcastStaticTransform()
  {
    geometry_msgs::TransformStamped tfStamped = 
      getStampedTransform(robot_to_laser_, "base_link", "base_laser");

    ROS_DEBUG_STREAM("broadcastStaticTransform()\n" << tfStamped);

    static_broadcaster_.sendTransform(tfStamped);
  }

  void TransformHandler::transformMessageToEigenTransform(
    const geometry_msgs::Transform& msg, Eigen::Affine3d& transform_out)
  {
    transform_out.translation() << msg.translation.x, msg.translation.y, msg.translation.z;
    Eigen::Quaterniond rotation(msg.rotation.w, msg.rotation.x, msg.rotation.y, msg.rotation.z);
    transform_out.rotate(rotation);
  }

  void TransformHandler::eigenTransformToTransformMessage(
    const Eigen::Affine3d& transform,  geometry_msgs::Transform& msg_out)
  {
    Eigen::Vector3d trans = transform.translation();
    Eigen::Quaterniond rot(transform.rotation());
    
    msg_out.translation.x = trans[0];
    msg_out.translation.y = trans[1];
    msg_out.translation.z = trans[2];
    msg_out.rotation.x = rot.x();
    msg_out.rotation.y = rot.y();
    msg_out.rotation.z = rot.z();
    msg_out.rotation.w = rot.w();
  }

  std::ifstream& TransformHandler::gotoLine(std::ifstream& file, int num)
  {
    file.seekg(std::ios::beg);
    for(int i=0; i < num - 1; ++i){
        file.ignore(std::numeric_limits<std::streamsize>::max(),'\n');
    }
    return file;
  }

  geometry_msgs::TransformStamped TransformHandler::getStampedTransform(
    const Eigen::Affine3d& transform, const std::string& parent_frame, const std::string& child_frame) const
  {
    geometry_msgs::TransformStamped tfStamped;
    Eigen::Vector3d position = transform.translation();
    Eigen::Quaterniond orientation(transform.rotation());

    ros::Time time_stamp;
    if(stamp_.compare("") != 0)
      time_stamp.fromSec(std::stod(stamp_));
    else 
      time_stamp.fromSec(0.0);
    
    tfStamped.header.stamp = time_stamp;
    tfStamped.header.frame_id = parent_frame;
    tfStamped.child_frame_id = child_frame;
    tfStamped.transform.translation.x = position[0];
    tfStamped.transform.translation.y = position[1];
    tfStamped.transform.translation.z = position[2];

    tfStamped.transform.rotation.x = orientation.x();
    tfStamped.transform.rotation.y = orientation.y();
    tfStamped.transform.rotation.z = orientation.z();
    tfStamped.transform.rotation.w = orientation.w();
    return tfStamped;
  }

  void TransformHandler::init()
  {
    this->registration_transform_ = Eigen::Affine3d::Identity();
    this->robot_pose_ = Eigen::Affine3d::Identity();
    this->odometry_ = Eigen::Affine3d::Identity();
    this->old_odometry_ = Eigen::Affine3d::Identity();
    this->robot_to_laser_ = Eigen::Affine3d::Identity();
    this->stamp_ = "";
    this->global_sensor_pose_ = Eigen::Affine3d::Identity();
  }
};