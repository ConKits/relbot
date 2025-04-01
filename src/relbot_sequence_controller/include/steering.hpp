#ifndef STEER_RELBOT_HPP_
#define STEER_RELBOT_HPP_

// CPP library headers
#include <cmath>
// ROS Client Library CPP
#include "rclcpp/rclcpp.hpp"

// message type for velocity
#include "example_interfaces/msg/float64.hpp"
#include "geometry_msgs/msg/point.hpp"




class SteerRelbot : public rclcpp::Node
{
public:
    /**
     * @brief Construct a new steering object
     */
    SteerRelbot();

    const double DEFAULT_SETPOINT_STREAM = 30;  // How often the velocities are published per second
  
private:
    // Topics
    rclcpp::Publisher<example_interfaces::msg::Float64>::SharedPtr left_wheel_topic_;
    rclcpp::Publisher<example_interfaces::msg::Float64>::SharedPtr right_wheel_topic_;
    //rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr ;
    
    // Timer
    rclcpp::TimerBase::SharedPtr timer_;
    
    // Attributes    
    double tol = 0.08;
    double radius=8.0;
    double wheelDistance=4.0;

    // Velocity attributes
    double angularVelocity = 1.0; // Angular velocity (v = r * ω)
    double left_velocity;
    double right_velocity;

    
    // Times for moving straight and turning
    double straight_time=5.0;
    double turning_time = (M_PI) / (2 * angularVelocity); // Time to turn 90 degrees (angle = ω * time)

    enum Shape {line, circle, bend, square};
    //Navigating Methosds. Such as moveStraiaght and rotate.
    void moveStraight();
    void rotate();

    // methods
    void create_topics();
    void timer_callback();
    void calculate_velocity();
   /**
   * @brief Handles receiving of received webcam images
   *
   * @param msg_cam_img webcam image, in sensor_msgs::msg::Image format
   */
  //void webcam_topic_callback(const sensor_msgs::msg::Image::SharedPtr msg_cam_img);

};

#endif /*STEER_RELBOT_HPP_*/