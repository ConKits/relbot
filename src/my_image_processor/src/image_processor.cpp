
#include "image_processor.hpp"

ImageProcessor::ImageProcessor() : Node("image_processor") {
    receivedImage_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/output/moving_camera", 1, std::bind(&ImageProcessor::image_callback, this, std::placeholders::_1));

    cordinatesImage_=this->create_publisher<geometry_msgs::msg::PointStamped>("/green_object_position",1);

    RCLCPP_INFO(this->get_logger(), "ImageProcessor node started. Waiting for images...");
}



void ImageProcessor::track_green_object(cv::Mat &cv_image) {
    // Convert the image from BGR to HSV color space
    cv::Mat hsv_image;
    cv::cvtColor(cv_image, hsv_image, cv::COLOR_BGR2HSV);

    // Define the lower and upper bounds for green color in HSV
    cv::Scalar lower_green(35, 50, 50);  // Adjust these values if needed
    cv::Scalar upper_green(85, 255, 255);

    // Threshold the image to extract only green regions
    cv::Mat mask;
    cv::inRange(hsv_image, lower_green, upper_green, mask);

    // Find contours in the masked image
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(mask, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // Draw contours on the original image (for visualization)
    if (!contours.empty()) {
        // Find the largest contour (assumed to be the greenest object)
        size_t largest_contour_idx = 0;
        double max_area = 0.0;
        for (size_t i = 0; i < contours.size(); i++) {
            double area = cv::contourArea(contours[i]);
            if (area > max_area) {
                max_area = area;
                largest_contour_idx = i;
            }
        }

        // Draw bounding box around the largest green object
        cv::Rect bounding_box = cv::boundingRect(contours[largest_contour_idx]);
        cv::rectangle(cv_image, bounding_box, cv::Scalar(0, 255, 0), 2);

        // Mark the center of the detected green object
        cv::Point center(bounding_box.x + bounding_box.width / 2, bounding_box.y + bounding_box.height / 2);
        cv::circle(cv_image, center, 3, cv::Scalar(0, 0, 255), -1);        
        
        //Publishing the cordinates of the detected green object
        geometry_msgs::msg::PointStamped center_msg;
        center_msg.point.x = center.x; //X cordinate of the tracked object
        center_msg.point.y = frame_width/2; //Center of the frame at the x direction
        double box_area = bounding_box.area();
        center_msg.point.z = box_area; //Bonding box's area
        center_msg.header.stamp = this->now();

        // Publish the coordinates of the detected object
        cordinatesImage_->publish(center_msg);
    }
}

void ImageProcessor::image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    try {
        // Convert ROS2 Image message to OpenCV format
        cv::Mat cv_image = cv_bridge::toCvCopy(msg, "bgr8")->image;
        frame_width = msg->width;

        // Process the image to track the green object
        track_green_object(cv_image);

        // Display the processed image
        cv::imshow("Green Object Tracking", cv_image);
        cv::waitKey(1);
    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Image processing failed: %s", e.what());
    }
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageProcessor>());
    rclcpp::shutdown();
    return 0;
}
