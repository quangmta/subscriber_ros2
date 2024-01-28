#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <fstream>

class LaserScanSubscriber : public rclcpp::Node
{
public:
  LaserScanSubscriber(const std::string& topic_name) : Node("laser_scan_subscriber")
  {
    // Subscribe to the laser scan topic
    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      topic_name,
      rclcpp::QoS(10),
      std::bind(&LaserScanSubscriber::laserScanCallback, this, std::placeholders::_1)
    );
  }

private:
  void laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    // Save the scan data as a CSV file
    std::string filePath = "save_data/laser_scan.csv";
    std::ofstream outFile(filePath);
    for (size_t i = 0; i < msg->ranges.size(); ++i)
    {
      outFile << msg->ranges[i];
      outFile << "\n";
    }
    outFile.close();

    RCLCPP_INFO(this->get_logger(), "Laser scan data saved: %s", filePath.c_str());

    // Shutdown the node after receiving the laser scan data
    rclcpp::shutdown();
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
};

int main(int argc, char** argv)
{
  if (argc < 2) {
    printf("Usage: laser_scan_subscriber <topic_name>\n");
    return 1;
  }

  rclcpp::init(argc, argv);
  auto node = std::make_shared<LaserScanSubscriber>(argv[1]);
  rclcpp::spin(node);
  return 0;
}