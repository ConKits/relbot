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
    double buffer_zone = 1; // Buffer zone where the robot is idle
    double wheelDistance=1; // Distance between the wheels
    // Times for moving straight and turning
    double timeConstant=5.0;
    // Velocity attributes
    double maxVelocity = 10.0/timeConstant; // Maximum velocity of the robot
    double left_velocity;
    double right_velocity;
    double linear_velocity;
    double th_velocity;

    //Object's cordinates from the image processor.
    double x_object;
    double area_object;
    double x_center; // Center of the image
    double x_error;
    double th_error;
    bool idleState=true; 

    // Thresholds
    double threshold_area=100.0; // Area threshold for object detection
    double minimum_area=50.0; // Minimum area for object detection
 

   
   
    
    //Navigating Methosds. Such as moveStraiaght and rotate.
    void idle();
    void moveStraight(double error);
    void rotate(double error);

    // methods
    void create_topics();
    void timer_callback();
    void calculate_velocity();

    void position_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
    
};

#endif /*STEER_RELBOT_HPP_*/