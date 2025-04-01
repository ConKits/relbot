#include <cmath>
#include "steering.hpp"

SteerRelbot::SteerRelbot() : Node("steer_relbot") {
    RCLCPP_INFO(this->get_logger(), "Init");

    // initialize topics
    create_topics();
    RCLCPP_INFO(this->get_logger(), "Created Topics");

    // initialize timer
    timer_ = this->create_wall_timer(std::chrono::duration<double>(1/DEFAULT_SETPOINT_STREAM),
                                     std::bind(&SteerRelbot::timer_callback, this));
}

void SteerRelbot::create_topics() {
    left_wheel_topic_ = this->create_publisher<example_interfaces::msg::Float64>(
        "/input/left_motor/setpoint_vel", 1);

    right_wheel_topic_ = this->create_publisher<example_interfaces::msg::Float64>(
        "/input/right_motor/setpoint_vel", 1);
    
    object_cordinates_ = this->create_subscription<geometry_msgs::msg::Point>("/green_object_position", 10,
            std::bind(&SteerRelbot::position_callback, this, std::placeholders::_1)) ;
}


// Navigation methods are working with constant FPS. 
void SteerRelbot::moveStraight() {
    // Moves the robot straight
    // This method is calculating the velocities for each wheel to move straight.
    left_velocity = -(angularVelocity * radius);
    right_velocity = angularVelocity * radius;
}

void SteerRelbot::rotate() {
    // Rotates the robot
    // This method is calculating the velocities for each wheel to rotate.
    left_velocity = -angularVelocity*(radius - (wheelDistance/2));
    right_velocity = angularVelocity*(radius + (wheelDistance/2));
     
    
}

void SteerRelbot::calculate_velocity() {
    //Signing time attributes to track the time
    static rclcpp::Time last_time = this->get_clock()->now(); // Store the last update time
    rclcpp::Time current_time = this->get_clock()->now();    // Get the current time

    // Calculate the elapsed time in seconds
    double elapsed_time = (current_time - last_time).seconds(); // Convert to seconds

    Shape shape{square}; //Shape of the robot's path
    //Switching between different shapes
    switch (shape) {
        case line:
            //Creates a straight line
            moveStraight();
            break;

        case circle:
            //Creates a circle
            rotate();
            break;

        case bend:
            //Creates a bend
            //The if statement shifts between rotate to straight to create a bend based on the turning time
            if (elapsed_time > straight_time && elapsed_time <= (turning_time + straight_time)) {
                rotate();
            } else {
                moveStraight();
            }
            break;

        case square:
            //Creates a square
            //The if statement shifts between rotate to straight to create a square based on the turning time
            if (elapsed_time > straight_time && elapsed_time <= (turning_time + straight_time)) {
                rotate();
                //Overide the last time for the next cycle of bending
                if ((elapsed_time - straight_time) >= turning_time - tol) {
                    last_time = current_time;
                    break;
                }
            } else {
                moveStraight();
            }
            break;
    }
}

void SteerRelbot::timer_callback() {
    // calculate velocity
    //calculate_velocity();

    // publish velocity to simulator
    example_interfaces::msg::Float64 left_wheel;
    left_wheel.data = left_velocity;
    example_interfaces::msg::Float64 right_wheel;
    right_wheel.data = right_velocity;
    left_wheel_topic_->publish(left_wheel);
    right_wheel_topic_->publish(right_wheel);
}

void SteerRelbot::position_callback(const geometry_msgs::msg::Point::SharedPtr cord){
    
    
    static rclcpp::Time last_time_object = this->get_clock()->now(); // Store the last update time
    rclcpp::Time current_time_object = this->get_clock()->now();    // Get the current time
    double time_object=(current_time_object-last_time_object).seconds();

    //Check for valid position data for the idle mode.
    if (time_object>2.0){
        idleState=false;
        x_object=cord->x;
        y_object=cord->y;
        RCLCPP_INFO(this->get_logger(), "Received Green Object Position -> x: %.2f, y: %.2f", x_object-2.0, y_object-3.0);
    }


}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SteerRelbot>());
    rclcpp::shutdown();
    return 0;
}
/*
Pseudo Code to control the relbot:
-Must have two states/modes, the idle mode and action mode.

-Idle mode: The logic in the idle mode must do with timing of the next values of objects cordinates.
    *Less than a sertain amount of time it should start calculate the error between the actual position and the object's.

-Action mode: In this mode the calculations of the wheels velocities are made by the given equations in the manual.
    *The checker is the error between the two spots, and if that equals zero the robot should be in the idle mode.
    *If the error is not equal zero then the robot should be in the action mode.
    *Assuming that the center of the camera/image is the 0,0 origin and then move the robot in such a way to reduce the error
    close to zero. It will be useful to use tollerance to estimate the positions.
    *So error=

*/