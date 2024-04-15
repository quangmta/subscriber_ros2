#include <subscriber/data_subscriber.hpp>

DataSubscriber::DataSubscriber(const rclcpp::NodeOptions &options) : rclcpp::Node("data_subscriber", options)
{
  auto qos = rclcpp::SystemDefaultsQoS();

  // Subscribe to the topic
  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", qos,
      std::bind(&PointCloudProcessingNode::ScanCallback, this, std::placeholders::_1));

  image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/image", qos,
      std::bind(&PointCloudProcessingNode::ImageCallback, this, std::placeholders::_1));

  camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      "/camera_info", qos,
      std::bind(&PointCloudProcessingNode::InfoCallback, this, std::placeholders::_1));

  depth_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/depth", qos,
      std::bind(&PointCloudProcessingNode::DepthImageCallback, this, std::placeholders::_1));

  odom_sub_ = this->create_subscription<sensor_msgs::msg::Odometry>(
      "/depth", qos,
      std::bind(&PointCloudProcessingNode::DataCallback, this, std::placeholders::_1));
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

void DepthImageSubscriber::ImageCallback(const sensor_msgs::msg::Image::SharedPtr depth_image)
{
  depth_image_msg_ = depth_image;
}

void DepthImageSubscriber::DataCallback(const sensor_msgs::msg::Odometry::SharedPtr odom)
{
  if (odom_msg_ == nullptr)
  {
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

    // Start taking data and spinning
    odom_msg_ = odom;
    

  }
}

// int main(int argc, char **argv)
// {
//     if (argc < 8)
//     {
//         std::cout << "Usage: topic x y z roll pitch yaw\n";
//         return 1;
//     }

//     float speed[6];
//     for (int i = 0; i < 6; i++)
//     {
//         speed[i] = std::stof(argv[i + 2]);
//     }

//     rclcpp::init(argc, argv);
//     auto node = rclcpp::Node::make_shared("my_node");

//     // Create a publisher for the Twist message
//     auto publisher = node->create_publisher<geometry_msgs::msg::Twist>(argv[1], 10);

//     // Create a Twist message
//     auto twist = std::make_shared<geometry_msgs::msg::Twist>();

//     while (rclcpp::ok())
//     {
//         // Set the linear velocity
//         twist->linear.x = speed[0];
//         twist->linear.y = speed[1];
//         twist->linear.z = speed[2];

//         // Set the angular velocity
//         twist->angular.x = speed[3];
//         twist->angular.y = speed[4];
//         twist->angular.z = speed[5];

//         // Publish the Twist message
//         publisher->publish(*twist);

//         RCLCPP_INFO(node->get_logger(), "%s: %f %f %f %f %f %f", argv[1], speed[0], speed[1], speed[2], speed[3], speed[4], speed[5]);
//         rclcpp::spin_some(node);
//     }
//     // rclcpp::shutdown();
//     return 0;
// }