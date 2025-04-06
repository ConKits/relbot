#ifndef STEER_RELBOT_HPP_
#define STEER_RELBOT_HPP_

// CPP library headers
#include <cmath>
// ROS Client Library CPP
#include "rclcpp/rclcpp.hpp"

// message type for velocity
#include "example_interfaces/msg/float64.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"




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
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr object_cordinates_;
    
    // Timer
    rclcpp::TimerBase::SharedPtr timer_;
    
    // Attributes    
    double tol = 0.08;
    double radius=100; // Radius of the wheels
    double wheelDistance=200; // Distance between the wheels

    // Velocity attributes
    double angularVelocity = 1/timeConstant; // Angular velocity (v = r * ω)
    double left_velocity;
    double right_velocity;

    //Object's cordinates from the image processor.
    double x_object;
    double y_object;
    double area_object;

    // Thresholds
    double threshold_area=1000.0; // Area threshold for object detection
    double minimum_area=500.0; // Minimum area for object detection
    double x_center=150.0; // Center of the image
    double y_center=100.0; // Center of the image
    double x_error;
    bool idleState=true;  

    // Times for moving straight and turning
    double timeConstant=1.0;
   
    
    //Navigating Methosds. Such as moveStraiaght and rotate.
    void idle();
    void moveStraight();
    void rotate(int direction);

    // methods
    void create_topics();
    void timer_callback();
    void calculate_velocity();

    void position_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
  
};

#endif /*STEER_RELBOT_HPP_*/