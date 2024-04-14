#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/twist.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include "nav_msgs/msg/odometry.hpp"

#include <fstream>
#include <opencv2/opencv.hpp>

class DataSubscriber : public rclcpp::Node
{
public:
    explicit DataSubscriberNode(const rclcpp::NodeOptions &options);
    ~DataSubscriberNode();

private:
    void ScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan);
    void ImageCallback(const sensor_msgs::msg::Image::SharedPtr image);
    void InfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr camera_info);
    void DepthImageCallback(const sensor_msgs::msg::Image::SharedPtr depth);
    void DataCallback(const sensor_msgs::msg::Odometry::SharedPtr odom);

    sensor_msgs::msg::LaserScan::SharedPtr scan_msg_;
    sensor_msgs::msg::Image::SharedPtr image_msg_;
    sensor_msgs::msg::Image::SharedPtr depth_image_msg_;
    sensor_msgs::msg::CameraInfo::SharedPtr camera_info_msg_;
    sensor_msgs::msg::Odometry::SharedPtr odom_msg_;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Odometry>::SharedPtr odom_sub_;
}