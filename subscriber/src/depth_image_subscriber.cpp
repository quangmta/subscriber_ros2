#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <fstream>
#include <opencv2/opencv.hpp>

class DepthImageSubscriber : public rclcpp::Node
{
public:
  DepthImageSubscriber(const std::string& topic_name) : Node("depth_image_subscriber")
  {
    // Subscribe to the depth image topic
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      topic_name,
      rclcpp::QoS(10),
      std::bind(&DepthImageSubscriber::depthImageCallback, this, std::placeholders::_1)
    );
  }

private:
  void depthImageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    // Convert the sensor_msgs::Image to cv::Mat
    cv::Mat depth_image(msg->height, msg->width, CV_32FC1, const_cast<float*>(reinterpret_cast<const float*>(&msg->data[0])), msg->step);

    // Save the depth values as a CSV file
    std::string filePath = "save_data/depth_image.csv";
    std::ofstream outFile(filePath);
    for (int i = 0; i < depth_image.rows; ++i)
    {
      for (int j = 0; j < depth_image.cols; ++j)
      {
        outFile << depth_image.at<float>(i, j) << ",";
      }
      outFile << "\n";
    }
    outFile.close();

    RCLCPP_INFO(this->get_logger(), "Depth image saved: %s", filePath.c_str());

    // Shutdown the node after receiving the depth image
    rclcpp::shutdown();
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

int main(int argc, char** argv)
{
  if (argc < 2) {
    printf("Usage: depth_image_subscriber <topic_name>\n");
    return 1;
  }

  rclcpp::init(argc, argv);
  auto node = std::make_shared<DepthImageSubscriber>(argv[1]);
  rclcpp::spin(node);
  return 0;
}