#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <fstream>
#include <opencv2/opencv.hpp>

class ImageSubscriber : public rclcpp::Node
{
public:
  ImageSubscriber(const std::string &topic_name) : Node("image_subscriber")
  {
    // Subscribe to the image topic
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        topic_name,
        rclcpp::QoS(10),
        std::bind(&ImageSubscriber::imageCallback, this, std::placeholders::_1));
  }

private:
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    // Convert the sensor_msgs::Image to cv::Mat
    cv::Mat image(msg->height, msg->width, CV_8UC3, const_cast<unsigned char *>(msg->data.data()), msg->step);
    cv::cvtColor(image, image, cv::COLOR_RGB2BGR);
    std::string filePath = "save_data/image.png";
    cv::imwrite(filePath, image);
    RCLCPP_INFO(this->get_logger(), "Image saved: %s", filePath.c_str());

    rclcpp::shutdown();
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ImageSubscriber>(argv[1]);
  rclcpp::spin(node);
  return 0;
}