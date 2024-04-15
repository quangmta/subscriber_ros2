#include <data_subscriber/data_subscriber.hpp>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DataSubscriber>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}