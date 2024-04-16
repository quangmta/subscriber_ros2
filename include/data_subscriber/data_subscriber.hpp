#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/twist.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include "nav_msgs/msg/odometry.hpp"

#include <fstream>
#include <opencv2/opencv.hpp>
#define YAW_VEL 0.25
#define DELAY_TIME 1000

class DataSubscriber : public rclcpp::Node
{
public:
    explicit DataSubscriber(const rclcpp::NodeOptions &options);
    ~DataSubscriber();

private:
    void ScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan);
    void ImageCallback(const sensor_msgs::msg::Image::SharedPtr image);
    void InfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr camera_info);
    void DepthImageCallback(const sensor_msgs::msg::Image::SharedPtr depth);
    void DataCallback(const nav_msgs::msg::Odometry::SharedPtr odom);
    bool CheckNullMsg();
    void SaveImage();
    void SaveDepthImage();
    void SaveLaserScan();
    void SaveCameraInfo();
    void SaveOdometry();
    geometry_msgs::msg::Twist::SharedPtr TwistSet(double x, double y, double z, double roll, double pitch, double yaw);

    sensor_msgs::msg::LaserScan::SharedPtr scan_msg_;
    sensor_msgs::msg::Image::SharedPtr image_msg_;
    sensor_msgs::msg::Image::SharedPtr depth_image_msg_;
    sensor_msgs::msg::CameraInfo::SharedPtr camera_info_msg_;
    nav_msgs::msg::Odometry::SharedPtr prev_odom_msg_;
    nav_msgs::msg::Odometry::SharedPtr take_odom_msg_;
    nav_msgs::msg::Odometry::SharedPtr init_odom_msg_;
    int count_ = 0;
    double init_yaw, take_yaw;
    double total_yaw = 0;
    bool save_data = false;
    bool count_data[3] = {false,false,false};
    std::string folder;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
};