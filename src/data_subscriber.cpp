#include <data_subscriber/data_subscriber.hpp>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <filesystem>
#include <cmath>
#include <image_geometry/pinhole_camera_model.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>

DataSubscriber::DataSubscriber(const rclcpp::NodeOptions &options) : rclcpp::Node("data_subscriber_node", options)
{
  auto qos = rclcpp::SystemDefaultsQoS();

  // Subscribe to the topic
  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", qos,
      std::bind(&DataSubscriber::ScanCallback, this, std::placeholders::_1));

  image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/image", qos,
      std::bind(&DataSubscriber::ImageCallback, this, std::placeholders::_1));

  camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      "/camera_info", qos,
      std::bind(&DataSubscriber::InfoCallback, this, std::placeholders::_1));

  depth_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/depth", qos,
      std::bind(&DataSubscriber::DepthImageCallback, this, std::placeholders::_1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/depth", qos,
      std::bind(&DataSubscriber::DataCallback, this, std::placeholders::_1));

  auto twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", qos);
}

DataSubscriber::~DataSubscriber() {}

void DataSubscriber::ScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
{
  scan_msg_ = scan;
}

void DataSubscriber::ImageCallback(const sensor_msgs::msg::Image::SharedPtr image)
{
  image_msg_ = image;
}

void DataSubscriber::InfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr camera_info)
{
  camera_info_msg_ = camera_info;
}

void DataSubscriber::DepthImageCallback(const sensor_msgs::msg::Image::SharedPtr depth_image)
{
  depth_image_msg_ = depth_image;
}

bool DataSubscriber::CheckNullMsg()
{
  bool flag_return = 0;
  if (nullptr == scan_msg_)
  {
    RCLCPP_INFO(get_logger(), "No laser scan, skipping point cloud processing");
    flag_return = 1;
  }
  if (nullptr == image_msg_)
  {
    RCLCPP_INFO(get_logger(), "No image, skipping point cloud processing");
    flag_return = 1;
  }
  if (nullptr == camera_info_msg_)
  {
    RCLCPP_INFO(get_logger(), "No camera info, skipping point cloud processing");
    flag_return = 1;
  }

  if (nullptr == depth_image_msg_)
  {
    RCLCPP_INFO(get_logger(), "No depth camera info, skipping point cloud processing");
    flag_return = 1;
  }

  if (image_msg_->height != depth_image_msg_->height || image_msg_->width != depth_image_msg_->width)
  {
    RCLCPP_INFO(get_logger(), "Difference size of image, skipping point cloud processing");
    flag_return = 1;
  }

  return flag_return;
}

void DataSubscriber::DataCallback(const nav_msgs::msg::Odometry::SharedPtr odom)
{
  if (odom_msg_ == nullptr)
  {
    if (CheckNullMsg())
      return;

    // Start taking data and spinning
    count_++;
    odom_msg_ = odom;

    //  Create folder
    //  Get the current date and time
    auto now = std::chrono::system_clock::now();
    auto now_t = std::chrono::system_clock::to_time_t(now);

    // Convert the time_t value to a tm structure
    std::tm *now_tm = std::localtime(&now_t);

    // Create a stringstream to hold the formatted date and time
    std::stringstream ss;
    ss << std::put_time(now_tm, "%d%m%Y_%H%M");

    folder = "save_data/" + ss.str();

    try
    {
      // Create the folder
      std::filesystem::create_directory(folder);
      std::filesystem::create_directory(folder + "/rgb");
      std::filesystem::create_directory(folder + "/depth");
      std::filesystem::create_directory(folder + "/scan");
      RCLCPP_INFO(this->get_logger(), "Folders created successfully.");
    }
    catch (std::exception &e)
    {
      // Handle any exceptions that occur
      RCLCPP_INFO(this->get_logger(), "Error creating folder: %s", e.what());
    }
    SaveCameraInfo();
    SaveImage();
    SaveDepthImage();
    SaveLaserScan();
    SaveOdometry();

    // start spinning
    twist_pub_->publish(TwistSet(0.0, 0.0, 0.0, 0.0, 0.0, 1.0));
  }
  else
  {
    // Check if angle spin is 22.5 or not
    tf2::Quaternion quat;
    tf2::fromMsg(odom->pose.pose.orientation, quat);
    double roll, pitch, yaw;
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    tf2::fromMsg(odom_msg_->pose.pose.orientation, quat);
    double roll_prev, pitch_prev, yaw_prev;
    tf2::Matrix3x3(quat).getRPY(roll_prev, pitch_prev, yaw_prev);

    double delta_yaw = yaw - yaw_prev;
    if (yaw > M_PI / 2 && yaw_prev < -M_PI / 2)
      delta_yaw -= 2*M_PI;
    else if (yaw < -M_PI / 2 && yaw_prev > M_PI / 2)
      delta_yaw += 2*M_PI;

    if (std::fabs(std::fabs(delta_yaw) - M_PI / 16) < M_PI / 128)
    {
      SaveImage();
      SaveDepthImage();
      SaveLaserScan();
      SaveOdometry();
    }
    else
    {
      twist_pub_->publish(TwistSet(0.0, 0.0, 0.0, 0.0, 0.0, 1.0));
    }
  }
}
void DataSubscriber::SaveImage()
{
  // Convert the sensor_msgs::Image to cv::Mat
  cv::Mat image(image_msg_->height, image_msg_->width, CV_8UC3, const_cast<unsigned char *>(image_msg_->data.data()), image_msg_->step);
  cv::cvtColor(image, image, cv::COLOR_RGB2BGR);
  std::string filePath = folder + "/rgb/" + std::to_string(count_) + ".png ";
  cv::imwrite(filePath, image);
  RCLCPP_INFO(this->get_logger(), "Image saved: %s", filePath.c_str());
}
void DataSubscriber::SaveDepthImage()
{
  cv::Mat depth_image;
  if (depth_image_msg_->encoding == "32FC1")
    depth_image = cv::Mat(depth_image_msg_->height, depth_image_msg_->width, CV_32FC1, const_cast<float *>(reinterpret_cast<const float *>(&depth_image_msg_->data[0])), depth_image_msg_->step);
  else if (depth_image_msg_->encoding == "16UC1")
    depth_image = cv::Mat(depth_image_msg_->height, depth_image_msg_->width, CV_16UC1, const_cast<uint16_t *>(reinterpret_cast<const uint16_t *>(&depth_image_msg_->data[0])), depth_image_msg_->step);
  // Save the depth values as a CSV file
  std::string filePath = folder + "/depth/" + std::to_string(count_) + ".csv ";
  std::ofstream outFile(filePath);
  for (int i = 0; i < depth_image.rows; ++i)
  {
    for (int j = 0; j < depth_image.cols; ++j)
    {
      if (depth_image_msg_->encoding == "32FC1")
        outFile << static_cast<double>(depth_image.at<float>(i, j)) << ",";
      else if (depth_image_msg_->encoding == "16UC1")
        outFile << static_cast<double>(depth_image.at<uint16_t>(i, j)) * 0.001 << ",";
    }
    outFile << "\n";
  }
  outFile.close();

  RCLCPP_INFO(this->get_logger(), "Depth image saved: %s", filePath.c_str());
}

void DataSubscriber::SaveLaserScan()
{
  // Save the scan data as a CSV file
  std::string filePath = folder + "/scan/" + std::to_string(count_) + ".csv ";
  std::ofstream outFile(filePath);
  for (size_t i = 0; i < scan_msg_->ranges.size(); ++i)
  {
    outFile << scan_msg_->ranges[i];
    outFile << "\n";
  }
  outFile.close();

  RCLCPP_INFO(this->get_logger(), "Laser scan data saved: %s", filePath.c_str());
}

void DataSubscriber::SaveCameraInfo()
{
  image_geometry::PinholeCameraModel cam_model_;
  cam_model_.fromCameraInfo(camera_info_msg_);
  // Save the scan data as a CSV file
  std::string filePath = folder + "CamInfo.csv ";
  std::ofstream outFile(filePath);
  outFile << "width," << cam_model_.fullResolution().width << "\n";
  outFile << "height," << cam_model_.fullResolution().height << "\n";
  outFile << "fx," << cam_model_.fx() << "\n";
  outFile << "fy," << cam_model_.fy() << "\n";
  outFile << "cx," << cam_model_.cx() << "\n";
  outFile << "cy," << cam_model_.cy() << "\n";
  outFile.close();

  RCLCPP_INFO(this->get_logger(), "Camera Info data saved: %s", filePath.c_str());
}

void DataSubscriber::SaveOdometry()
{
  std::string filePath = folder + "Odom.csv";
  std::ofstream outFile(filePath);

  tf2::Quaternion quat;
  tf2::fromMsg(odom_msg_->pose.pose.orientation, quat);
  double roll, pitch, yaw;
  tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

  outFile << count_ << ","
          << odom_msg_->pose.pose.position.x << ","
          << odom_msg_->pose.pose.position.y << ","
          << yaw << "\n";
  outFile.close();
}

geometry_msgs::msg::Twist DataSubscriber::TwistSet(double x, double y, double z, double roll, double pitch, double yaw)
{
  // Create a Twist message
  geometry_msgs::msg::Twist twist;
  // Set the linear and angular velocities
  twist.linear.x = x;
  twist.linear.y = y;
  twist.linear.z = z;
  twist.angular.x = roll;
  twist.angular.y = pitch;
  twist.angular.z = yaw;
  return twist;
}
