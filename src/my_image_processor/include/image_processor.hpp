
#ifndef IMAGE_PROCESSOR_HPP
#define IMAGE_PROCESSOR_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>

class ImageProcessor : public rclcpp::Node {
public:
    ImageProcessor();

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
    void track_green_object(cv::Mat &cv_image);
    
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr receivedImage_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr cordinatesImage_;
    double frame_width;
};

#endif // IMAGE_PROCESSOR_HPP
